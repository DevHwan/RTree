//
// RTree.h
//
#ifndef RTREE_H
#define RTREE_H

#include <cassert>
#include <cmath>
#include <cstdio>

#include <array>
#include <algorithm>
#include <functional>
#include <type_traits>

#define RTreeAssert assert // RTree uses RTreeAssert( condition )

#define RTREE_DONT_USE_MEMPOOLS         // This version does not contain a fixed memory allocator, fill in lines with EXAMPLE to implement one.

// Because there is not stream support, this is a quick and dirty file I/O helper.
// Users will likely replace its usage with a Stream implementation from their favorite API.
class RTFileStream {
public:
    RTFileStream() = default;

    ~RTFileStream() {
        Close();
    }

    bool OpenRead(const char* a_fileName) {
        m_file = fopen(a_fileName, "rb");
        if (!m_file) {
            return false;
        }
        return true;
    }

    bool OpenWrite(const char* a_fileName) {
        m_file = fopen(a_fileName, "wb");
        if (!m_file) {
            return false;
        }
        return true;
    }

    void Close() {
        if (m_file) {
            fclose(m_file);
            m_file = nullptr;
        }
    }

    template< typename TYPE >
    size_t Write(const TYPE& a_value) {
        RTreeAssert(m_file);
        return fwrite((void*)&a_value, sizeof(a_value), 1, m_file);
    }

    template< typename TYPE >
    size_t WriteArray(const TYPE* a_array, int a_count) {
        RTreeAssert(m_file);
        return fwrite((void*)a_array, sizeof(TYPE) * a_count, 1, m_file);
    }

    template< typename TYPE >
    size_t Read(TYPE& a_value) {
        RTreeAssert(m_file);
        return fread((void*)&a_value, sizeof(a_value), 1, m_file);
    }

    template< typename TYPE >
    size_t ReadArray(TYPE* a_array, int a_count) {
        RTreeAssert(m_file);
        return fread((void*)a_array, sizeof(TYPE) * a_count, 1, m_file);
    }

private:
    FILE* m_file = nullptr;
};

/// \class RTree
/// Implementation of RTree, a multidimensional bounding rectangle tree.
/// Example usage: For a 3-dimensional tree use RTree<Object*, float, 3> myTree;
///
/// This modified, templated C++ version by Greg Douglas at Auran (http://www.auran.com)
///
/// _DataType Referenced data, should be int, void*, obj* etc. no larger than sizeof<void*> and simple type
/// _ElementType Type of element such as int or float
/// _NumDimensions Number of dimensions such as 2 or 3
/// _ElementTypeReal Type of element that allows fractional and large values such as float or double, for use in volume calcs
///
/// NOTES: Inserting and removing data requires the knowledge of its constant Minimal Bounding Rectangle.
///        This version uses new/delete for nodes, I recommend using a fixed size allocator for efficiency.
///        Instead of using a callback function for returned results, I recommend and efficient pre-sized, grow-only memory
///        array similar to MFC CArray or STL Vector for returning search query result.
///
template<class _DataType, class _ElementType, int _NumDimensions, 
    class _ElementTypeReal = _ElementType, int _MaxNodeCount = 8, int _MinNodeCount = _MaxNodeCount / 2, bool _UseSphericalVolume = true>
class RTree {
    static_assert(1 < _NumDimensions, "_NumDimensions must be larger than 1");
    static_assert(0 < _MinNodeCount, "_MinNodeCount must be larger than 0");
    static_assert(_MinNodeCount < _MaxNodeCount, "_MaxNodeCount must be larger than _MinNodeCount");
    static_assert(std::is_floating_point<_ElementTypeReal>::value, "_ElementTypeReal must be floating point type");
protected:

    struct Node;  // Fwd decl.  Used by other internal structs and iterator

public:
    using DataType = _DataType;
    using ElementType = _ElementType;
    using ElementTypeReal = _ElementTypeReal;
    using Element = ElementType[_NumDimensions];

    constexpr static const int kNumDimensions = _NumDimensions;
    constexpr static const int kMaxNodeCount = _MaxNodeCount;
    constexpr static const int kMinNodeCount = _MinNodeCount;
    constexpr static const bool kUseSphericalVolume = _UseSphericalVolume;

    template<typename ValueType>
    constexpr static ElementType CastElementType(const ValueType val) noexcept {
        return static_cast<ElementType>(val);
    }
    template<typename ValueType>
    constexpr static ElementTypeReal CastElementTypeReal(const ValueType val) noexcept {
        return static_cast<ElementTypeReal>(val);
    }
    constexpr static const ElementTypeReal kElementTypeRealOne = CastElementTypeReal(1.0);
    constexpr static const ElementTypeReal kElementTypeRealZero = CastElementTypeReal(0.0);

    RTree() {
        // Precomputed volumes of the unit spheres for the first few dimensions
        constexpr const ElementTypeReal kUnitSphereVolumes[] = {
          CastElementTypeReal(0.000000), CastElementTypeReal(2.000000), CastElementTypeReal(3.141593), // Dimension  0,1,2
          CastElementTypeReal(4.188790), CastElementTypeReal(4.934802), CastElementTypeReal(5.263789), // Dimension  3,4,5
          CastElementTypeReal(5.167713), CastElementTypeReal(4.724766), CastElementTypeReal(4.058712), // Dimension  6,7,8
          CastElementTypeReal(3.298509), CastElementTypeReal(2.550164), CastElementTypeReal(1.884104), // Dimension  9,10,11
          CastElementTypeReal(1.335263), CastElementTypeReal(0.910629), CastElementTypeReal(0.599265), // Dimension  12,13,14
          CastElementTypeReal(0.381443), CastElementTypeReal(0.235331), CastElementTypeReal(0.140981), // Dimension  15,16,17
          CastElementTypeReal(0.082146), CastElementTypeReal(0.046622), CastElementTypeReal(0.025807), // Dimension  18,19,20 
        };

        this->m_root = this->AllocNode();
        this->m_root->m_level = 0;
        this->m_unitSphereVolume = kUnitSphereVolumes[kNumDimensions];
    }

    RTree(const RTree& other) : RTree() {
        this->CopyRec(this->m_root, other.m_root);
    }

    ~RTree() {
        this->Reset();
    }

    /// Insert entry
    /// \param minPos Min of bounding rect
    /// \param maxPos Max of bounding rect
    /// \param dataId Positive Id of data.  Maybe zero, but negative numbers not allowed.
    void Insert(const Element& minPos, const Element& maxPos, const DataType& dataId) {
#ifdef _DEBUG
        for (int index = 0; index < kNumDimensions; ++index) {
            RTreeAssert(minPos[index] <= maxPos[index]);
        }
#endif //_DEBUG

        Branch branch;
        branch.m_data = dataId;
        branch.m_child = nullptr;

        for (int axis = 0; axis < kNumDimensions; ++axis) {
            branch.m_rect.m_min[axis] = minPos[axis];
            branch.m_rect.m_max[axis] = maxPos[axis];
        }

        this->InsertRect(branch, &this->m_root, 0);
    }

    /// Remove entry
    /// \param minPos Min of bounding rect
    /// \param maxPos Max of bounding rect
    /// \param dataId Positive Id of data.  Maybe zero, but negative numbers not allowed.
    void Remove(const Element& minPos, const Element& maxPos, const DataType& dataId) {
#ifdef _DEBUG
        for (int index = 0; index < kNumDimensions; ++index) {
            RTreeAssert(minPos[index] <= maxPos[index]);
        }
#endif //_DEBUG

        Rect rect;

        for (int axis = 0; axis < kNumDimensions; ++axis) {
            rect.m_min[axis] = minPos[axis];
            rect.m_max[axis] = maxPos[axis];
        }

        this->RemoveRect(&rect, dataId, &this->m_root);
    }

    /// Find all within search rectangle
    /// \param minPos Min of search bounding rect
    /// \param maxPos Max of search bounding rect
    /// \param callback Callback function to return result.  Callback should return 'true' to continue searching
    /// \return Returns the number of entries found
    int Search(const Element& minPos, const Element& maxPos, std::function<bool(const DataType&)> callback) const {
#ifdef _DEBUG
        for (int index = 0; index < kNumDimensions; ++index) {
            RTreeAssert(minPos[index] <= maxPos[index]);
        }
#endif //_DEBUG

        Rect rect;

        for (int axis = 0; axis < kNumDimensions; ++axis) {
            rect.m_min[axis] = minPos[axis];
            rect.m_max[axis] = maxPos[axis];
        }

        // NOTE: May want to return search result another way, perhaps returning the number of found elements here.

        int foundCount = 0;
        this->Search(this->m_root, &rect, foundCount, callback);

        return foundCount;
    }

    /// Remove all entries from tree
    void RemoveAll() {

        // Delete all existing nodes
        this->Reset();

        m_root = this->AllocNode();
        m_root->m_level = 0;
    }

    /// Count the data elements in this container.  This is slow as no internal counter is maintained.
    int Count() const {

        int count = 0;
        this->CountRec(m_root, count);

        return count;
    }

    /// Load tree contents from file
    bool Load(const char* fileName) {
        this->RemoveAll(); // Clear existing tree

        RTFileStream stream;
        if (!stream.OpenRead(fileName)) {
            return false;
        }

        bool result = this->Load(stream);

        stream.Close();

        return result;
    }
    /// Load tree contents from stream
    bool Load(RTFileStream& stream) {
        // Write some kind of header
        int _dataFileId = ('R' << 0) | ('T' << 8) | ('R' << 16) | ('E' << 24);
        int _dataSize = sizeof(DataType);
        int _dataNumDims = kNumDimensions;
        int _dataElemSize = sizeof(ElementType);
        int _dataElemRealSize = sizeof(ElementTypeReal);
        int _dataMaxNodes = kMaxNodeCount;
        int _dataMinNodes = kMinNodeCount;

        int dataFileId = 0;
        int dataSize = 0;
        int dataNumDims = 0;
        int dataElemSize = 0;
        int dataElemRealSize = 0;
        int dataMaxNodes = 0;
        int dataMinNodes = 0;

        stream.Read(dataFileId);
        stream.Read(dataSize);
        stream.Read(dataNumDims);
        stream.Read(dataElemSize);
        stream.Read(dataElemRealSize);
        stream.Read(dataMaxNodes);
        stream.Read(dataMinNodes);

        bool result = false;

        // Test if header was valid and compatible
        if ((dataFileId == _dataFileId)
            && (dataSize == _dataSize)
            && (dataNumDims == _dataNumDims)
            && (dataElemSize == _dataElemSize)
            && (dataElemRealSize == _dataElemRealSize)
            && (dataMaxNodes == _dataMaxNodes)
            && (dataMinNodes == _dataMinNodes)) {
            // Recursively load tree
            result = this->LoadRec(this->m_root, stream);
        }

        return result;
    }


    /// Save tree contents to file
    bool Save(const char* fileName) const {

        RTFileStream stream;
        if (!stream.OpenWrite(fileName)) {
            return false;
        }

        bool result = this->Save(stream);

        stream.Close();

        return result;
    }
    /// Save tree contents to stream
    bool Save(RTFileStream& stream) const {

        // Write some kind of header
        int dataFileId = ('R' << 0) | ('T' << 8) | ('R' << 16) | ('E' << 24);
        int dataSize = sizeof(DataType);
        int dataNumDims = kNumDimensions;
        int dataElemSize = sizeof(ElementType);
        int dataElemRealSize = sizeof(ElementTypeReal);
        int dataMaxNodes = kMaxNodeCount;
        int dataMinNodes = kMinNodeCount;

        stream.Write(dataFileId);
        stream.Write(dataSize);
        stream.Write(dataNumDims);
        stream.Write(dataElemSize);
        stream.Write(dataElemRealSize);
        stream.Write(dataMaxNodes);
        stream.Write(dataMinNodes);

        // Recursively save tree
        bool result = this->SaveRec(this->m_root, this->a_stream);

        return result;
    }

    /// Iterator is not remove safe.
    class Iterator {
    public:
        constexpr static const int kMaxStackSize = 32;  //  Max stack size. Allows almost n^32 where n is number of branches in node

    private:

        struct StackElement {
            Node* m_node = nullptr;
            int m_branchIndex = 0;
        };

    public:

        Iterator() {
            this->Init();
        }

        ~Iterator() = default;

        /// Is iterator invalid
        bool IsNull() const noexcept {
            return (this->m_tos <= 0);
        }

        /// Is iterator pointing to valid data
        bool IsNotNull() const noexcept {
            return (this->m_tos > 0);
        }

        /// Access the current data element. Caller must be sure iterator is not NULL first.
        DataType& operator*() {
            RTreeAssert(this->IsNotNull());
            const auto& curTos = this->m_stack[m_tos - 1];
            return curTos.m_node->m_branch[curTos.m_branchIndex].m_data;
        }

        /// Access the current data element. Caller must be sure iterator is not NULL first.
        const DataType& operator*() const {
            RTreeAssert(this->IsNotNull());
            const auto& curTos = this->m_stack[m_tos - 1];
            return curTos.m_node->m_branch[curTos.m_branchIndex].m_data;
        }

        /// Find the next data element
        bool operator++() {
            return this->FindNextData();
        }

        /// Get the bounds for this node
        void GetBounds(Element& minPos, Element& maxPos) {
            RTreeAssert(this->IsNotNull());
            const auto& curTos = this->m_stack[m_tos - 1];
            const auto& curBranch = curTos.m_node->m_branch[curTos.m_branchIndex];

            for (int index = 0; index < kNumDimensions; ++index)
            {
                minPos[index] = curBranch.m_rect.m_min[index];
                maxPos[index] = curBranch.m_rect.m_max[index];
            }
        }

    private:

        /// Reset iterator
        void Init() {
            this->m_tos = 0;
        }

        /// Find the next data element in the tree (For internal use only)
        bool FindNextData() {
            for (;;) {
                if (this->m_tos <= 0) {
                    return false;
                }
                const auto curTos = this->Pop(); // Copy stack top cause it may change as we use it

                if (curTos.m_node->IsLeaf()) {
                    // Keep walking through data while we can
                    if (curTos.m_branchIndex + 1 < curTos.m_node->m_count) {
                        // There is more data, just point to the next one
                        this->Push(curTos.m_node, curTos.m_branchIndex + 1);
                        return true;
                    }
                    // No more data, so it will fall back to previous level
                } else {
                    if (curTos.m_branchIndex + 1 < curTos.m_node->m_count) {
                        // Push sibling on for future tree walk
                        // This is the 'fall back' node when we finish with the current level
                        this->Push(curTos.m_node, curTos.m_branchIndex + 1);
                    }
                    // Since cur node is not a leaf, push first of next level to get deeper into the tree
                    const auto nextLevelnode = curTos.m_node->m_branch[curTos.m_branchIndex].m_child;
                    this->Push(nextLevelnode, 0);

                    // If we pushed on a new leaf, exit as the data is ready at TOS
                    if (nextLevelnode->IsLeaf()) {
                        return true;
                    }
                }
            }
        }

        /// Push node and branch onto iteration stack (For internal use only)
        void Push(Node* node, int branchIndex) {
            this->m_stack[m_tos].m_node = node;
            this->m_stack[m_tos].m_branchIndex = branchIndex;
            ++this->m_tos;
            RTreeAssert(this->m_tos <= kMaxStackSize);
        }

        /// Pop element off iteration stack (For internal use only)
        StackElement& Pop() {
            RTreeAssert(this->m_tos > 0);
            --this->m_tos;
            return this->m_stack[m_tos];
        }

        StackElement m_stack[kMaxStackSize];          ///< Stack as we are doing iteration instead of recursion
        int m_tos = 0;                                ///< Top Of Stack index

        friend class RTree; // Allow hiding of non-public functions while allowing manipulation by logical owner
    };

    /// Get 'first' for iteration
    void GetFirst(Iterator& iter) const {
        iter.Init();
        auto first = this->m_root;
        while (first) {
            if (first->IsInternalNode() && first->m_count > 1) {
                iter.Push(first, 1); // Descend sibling branch later
            } else if (first->IsLeaf()) {
                if (first->m_count) {
                    iter.Push(first, 0);
                }
                break;
            }
            first = first->m_branch[0].m_child;
        }
    }

    /// Get Next for iteration
    void GetNext(Iterator& iter) const {
        ++iter;
    }

    /// Is iterator NULL, or at end?
    bool IsNull(const Iterator& iter) const {
        return iter.IsNull();
    }

    /// Get object at iterator position
    DataType& GetAt(Iterator& iter) {
        return *iter;
    }

    /// Get object at iterator position
    const DataType& GetAt(Iterator& iter) const {
        return *iter;
    }

protected:

    /// Minimal bounding rectangle (n-dimensional)
    struct Rect {
        ElementType m_min[kNumDimensions] = { 0, };                      ///< Min dimensions of bounding box 
        ElementType m_max[kNumDimensions] = { 0, };                      ///< Max dimensions of bounding box 
    };

    /// May be data or may be another subtree
    /// The parents level determines this.
    /// If the parents level is 0, then this is data
    struct Branch {
        Rect m_rect;                                  ///< Bounds
        Node* m_child = nullptr;                      ///< Child node
        DataType m_data;                             ///< Data Id
    };

    /// Node for each branch level
    struct Node {
        bool IsInternalNode() const noexcept { return (m_level > 0); } // Not a leaf, but a internal node
        bool IsLeaf() const noexcept { return (m_level == 0); }        // A leaf, contains data

        int m_count = 0;                              ///< Count
        int m_level = 0;                              ///< Leaf is zero, others positive
        Branch m_branch[_MaxNodeCount];               ///< Branch
    };

    /// A link list of nodes for reinsertion after a delete operation
    struct ListNode {
        ListNode* m_next = nullptr;                   ///< Next in list
        Node* m_node = nullptr;                       ///< Node
    };

    /// Variables for finding a split partition
    struct PartitionVars {
        constexpr static const int kNotTaken = -1; // indicates that position

        int m_partition[_MaxNodeCount + 1] = { 0, };
        int m_total = 0;
        int m_minFill = 0;
        int m_count[2] = { 0, };
        Rect m_cover[2];
        ElementTypeReal m_area[2] = { kElementTypeRealZero, };

        Branch m_branchBuf[_MaxNodeCount + 1];
        int m_branchCount = 0;
        Rect m_coverSplit;
        ElementTypeReal m_coverSplitArea = kElementTypeRealZero;
    };

    Node* AllocNode() {
        Node* newNode = nullptr;
#ifdef RTREE_DONT_USE_MEMPOOLS
        newNode = new Node();
#else // RTREE_DONT_USE_MEMPOOLS
        // EXAMPLE
#endif // RTREE_DONT_USE_MEMPOOLS
        this->InitNode(newNode);
        return newNode;
    }

    void FreeNode(Node* node) {
        RTreeAssert(node);

#ifdef RTREE_DONT_USE_MEMPOOLS
        delete node;
#else // RTREE_DONT_USE_MEMPOOLS
        // EXAMPLE
#endif // RTREE_DONT_USE_MEMPOOLS
    }

    void InitNode(Node* node) {
        node->m_count = 0;
        node->m_level = -1;
    }

    void InitRect(Rect* rect) {
        for (int index = 0; index < kNumDimensions; ++index) {
            rect->m_min[index] = CastElementType(0);
            rect->m_max[index] = CastElementType(0);
        }
    }

    // Inserts a new data rectangle into the index structure.
    // Recursively descends tree, propagates splits back up.
    // Returns 0 if node was not split.  Old node updated.
    // If node was split, returns 1 and sets the pointer pointed to by
    // new_node to point to the new node.  Old node updated to become one of two.
    // The level argument specifies the number of steps up from the leaf
    // level to insert; e.g. a data rectangle goes in at level = 0.
    bool InsertRectRec(const Branch& branch, Node* node, Node** newnode, int level) {
        RTreeAssert(node && newnode);
        RTreeAssert(level >= 0 && level <= node->m_level);

        // recurse until we reach the correct level for the new record. data records
        // will always be called with level == 0 (leaf)
        if (node->m_level > level) {
            // Still above level for insertion, go down tree recursively
            Node* otherNode;

            // find the optimal branch for this record
            int index = this->PickBranch(&branch.m_rect, node);

            // recursively insert this record into the picked branch
            bool childWasSplit = this->InsertRectRec(branch, node->m_branch[index].m_child, &otherNode, level);

            if (!childWasSplit) {
                // Child was not split. Merge the bounding box of the new record with the
                // existing bounding box
                node->m_branch[index].m_rect = this->CombineRect(&branch.m_rect, &(node->m_branch[index].m_rect));
                return false;
            } else {
                // Child was split. The old branches are now re-partitioned to two nodes
                // so we have to re-calculate the bounding boxes of each node
                node->m_branch[index].m_rect = this->NodeCover(node->m_branch[index].m_child);
                Branch branch;
                branch.m_child = otherNode;
                branch.m_rect = this->NodeCover(otherNode);

                // The old node is already a child of node. Now add the newly-created
                // node to node as well. node might be split because of that.
                return this->AddBranch(&branch, node, newnode);
            }
        } else if (node->m_level == level) {
            // We have reached level for insertion. Add rect, split if necessary
            return this->AddBranch(&branch, node, newnode);
        } else {
            // Should never occur
            RTreeAssert(false);
            return false;
        }
    }

    // Insert a data rectangle into an index structure.
    // InsertRect provides for splitting the root;
    // returns 1 if root was split, 0 if it was not.
    // The level argument specifies the number of steps up from the leaf
    // level to insert; e.g. a data rectangle goes in at level = 0.
    // InsertRect2 does the recursion.
    bool InsertRect(const Branch& branch, Node** root, int level) {
        RTreeAssert(root);
        RTreeAssert(level >= 0 && level <= (*root)->m_level);
#ifdef _DEBUG
        for (int index = 0; index < kNumDimensions; ++index) {
            RTreeAssert(branch.m_rect.m_min[index] <= branch.m_rect.m_max[index]);
        }
#endif //_DEBUG  

        Node* newNode = nullptr;

        if (this->InsertRectRec(branch, *root, &newNode, level)) { // Root split
            // Grow tree taller and new root
            Node* newRoot = this->AllocNode();
            newRoot->m_level = (*root)->m_level + 1;

            Branch branch;

            // add old root node as a child of the new root
            branch.m_rect = this->NodeCover(*root);
            branch.m_child = *root;
            this->AddBranch(&branch, newRoot, nullptr);

            // add the split node as a child of the new root
            branch.m_rect = this->NodeCover(newNode);
            branch.m_child = newNode;
            this->AddBranch(&branch, newRoot, nullptr);

            // set the new root as the root node
            *root = newRoot;

            return true;
        }

        return false;
    }

    // Find the smallest rectangle that includes all rectangles in branches of a node.
    Rect NodeCover(Node* node) {
        RTreeAssert(node);

        Rect rect = node->m_branch[0].m_rect;
        for (int index = 1; index < node->m_count; ++index) {
            rect = this->CombineRect(&rect, &(node->m_branch[index].m_rect));
        }

        return rect;
    }

    // Add a branch to a node.  Split the node if necessary.
    // Returns 0 if node not split.  Old node updated.
    // Returns 1 if node split, sets *new_node to address of new node.
    // Old node updated, becomes one of two.
    bool AddBranch(const Branch* branch, Node* node, Node** newnode) {

        RTreeAssert(branch);
        RTreeAssert(node);

        if (node->m_count < _MaxNodeCount)  // Split won't be necessary
        {
            node->m_branch[node->m_count] = *branch;
            ++node->m_count;

            return false;
        } else {
            RTreeAssert(newnode);

            this->SplitNode(node, branch, newnode);
            return true;
        }
    }

    // Disconnect a dependent node.
    // Caller must return (or stop using iteration index) after this as count has changed
    void DisconnectBranch(Node* node, int index) {
        RTreeAssert(node && (index >= 0) && (index < _MaxNodeCount));
        RTreeAssert(node->m_count > 0);

        // Remove element by swapping with the last element to prevent gaps in array
        node->m_branch[index] = node->m_branch[node->m_count - 1];

        --node->m_count;
    }

    // Pick a branch.  Pick the one that will need the smallest increase
    // in area to accomodate the new rectangle.  This will result in the
    // least total area for the covering rectangles in the current node.
    // In case of a tie, pick the one which was smaller before, to get
    // the best resolution when searching.
    int PickBranch(const Rect* rect, Node* node) {
        RTreeAssert(rect && node);

        bool firstTime = true;
        ElementTypeReal increase = CastElementTypeReal(0);
        ElementTypeReal bestIncr = CastElementTypeReal(-1);
        ElementTypeReal area = CastElementTypeReal(0);
        ElementTypeReal bestArea = CastElementTypeReal(0);
        int best = 0;
        Rect tempRect;

        for (int index = 0; index < node->m_count; ++index) {
            Rect* curRect = &node->m_branch[index].m_rect;
            area = this->CalcRectVolume(curRect);
            tempRect = this->CombineRect(rect, curRect);
            increase = this->CalcRectVolume(&tempRect) - area;
            if ((increase < bestIncr) || firstTime) {
                best = index;
                bestArea = area;
                bestIncr = increase;
                firstTime = false;
            } else if ((increase == bestIncr) && (area < bestArea)) {
                best = index;
                bestArea = area;
                bestIncr = increase;
            }
        }
        return best;
    }

    // Combine two rectangles into larger one containing both
    Rect CombineRect(const Rect* rectA, const Rect* rectB) {
        RTreeAssert(rectA && rectB);

        Rect newRect;

        for (int index = 0; index < kNumDimensions; ++index) {
            newRect.m_min[index] = (std::min)(rectA->m_min[index], rectB->m_min[index]);
            newRect.m_max[index] = (std::max)(rectA->m_max[index], rectB->m_max[index]);
        }

        return newRect;
    }

    // Split a node.
    // Divides the nodes branches and the extra one between two nodes.
    // Old node is one of the new ones, and one really new one is created.
    // Tries more than one method for choosing a partition, uses best result.
    void SplitNode(Node* node, const Branch* branch, Node** newnode) {
        RTreeAssert(node);
        RTreeAssert(branch);

        // Could just use local here, but member or external is faster since it is reused
        PartitionVars localVars;
        PartitionVars* parVars = &localVars;

        // Load all the branches into a buffer, initialize old node
        this->GetBranches(node, branch, parVars);

        // Find partition
        this->ChoosePartition(parVars, _MinNodeCount);

        // Create a new node to hold (about) half of the branches
        *newnode = this->AllocNode();
        (*newnode)->m_level = node->m_level;

        // Put branches from buffer into 2 nodes according to the chosen partition
        node->m_count = 0;
        this->LoadNodes(node, *newnode, parVars);

        RTreeAssert((node->m_count + (*newnode)->m_count) == parVars->m_total);
    }

    // The exact volume of the bounding sphere for the given Rect
    ElementTypeReal RectSphericalVolume(Rect* rect) const {
        RTreeAssert(rect);

        ElementTypeReal sumOfSquares = kElementTypeRealZero;

        for (int index = 0; index < kNumDimensions; ++index) {
            const auto halfExtent = (CastElementTypeReal(rect->m_max[index]) - CastElementTypeReal(rect->m_min[index])) * CastElementTypeReal(0.5);
            sumOfSquares += halfExtent * halfExtent;
        }

        const auto radius = CastElementTypeReal(std::sqrt(sumOfSquares));

        // Pow maybe slow, so test for common dims like 2,3 and just use x*x, x*x*x.
        if (kNumDimensions == 3) {
            return (radius * radius * radius * this->m_unitSphereVolume);
        } else if (kNumDimensions == 2) {
            return (radius * radius * this->m_unitSphereVolume);
        } else {
            return CastElementTypeReal(std::pow(radius, kNumDimensions) * this->m_unitSphereVolume);
        }
    }

    // Calculate the n-dimensional volume of a rectangle
    ElementTypeReal RectVolume(Rect* rect) const {
        RTreeAssert(rect);

        auto volume = kElementTypeRealOne;

        for (int index = 0; index < kNumDimensions; ++index) {
            volume *= rect->m_max[index] - rect->m_min[index];
        }

        RTreeAssert(volume >= kElementTypeRealZero);

        return volume;
    }

    // Use one of the methods to calculate retangle volume
    ElementTypeReal CalcRectVolume(Rect* rect) const {
        if constexpr (kUseSphericalVolume) {
            return RectSphericalVolume(rect); // Slower but helps certain merge cases
        } else {
            return RectVolume(rect); // Faster but can cause poor merges
        }
    }

    // Load branch buffer with branches from full node plus the extra branch.
    void GetBranches(Node* node, const Branch* branch, PartitionVars* parVars) {
        RTreeAssert(node);
        RTreeAssert(branch);

        RTreeAssert(node->m_count == _MaxNodeCount);

        // Load the branch buffer
        for (int index = 0; index < _MaxNodeCount; ++index) {
            parVars->m_branchBuf[index] = node->m_branch[index];
        }
        parVars->m_branchBuf[_MaxNodeCount] = *branch;
        parVars->m_branchCount = _MaxNodeCount + 1;

        // Calculate rect containing all in the set
        parVars->m_coverSplit = parVars->m_branchBuf[0].m_rect;
        for (int index = 1; index < _MaxNodeCount + 1; ++index) {
            parVars->m_coverSplit = this->CombineRect(&parVars->m_coverSplit, &parVars->m_branchBuf[index].m_rect);
        }
        parVars->m_coverSplitArea = this->CalcRectVolume(&parVars->m_coverSplit);
    }

    // Method #0 for choosing a partition:
    // As the seeds for the two groups, pick the two rects that would waste the
    // most area if covered by a single rectangle, i.e. evidently the worst pair
    // to have in the same group.
    // Of the remaining, one at a time is chosen to be put in one of the two groups.
    // The one chosen is the one with the greatest difference in area expansion
    // depending on which group - the rect most strongly attracted to one group
    // and repelled from the other.
    // If one group gets too full (more would force other group to violate min
    // fill requirement) then other group gets the rest.
    // These last are the ones that can go in either group most easily.
    void ChoosePartition(PartitionVars* parVars, int minFill) {
        RTreeAssert(parVars);

        ElementTypeReal biggestDiff = CastElementTypeReal(-1);
        int group, chosen = 0, betterGroup = 0;

        this->InitParVars(parVars, parVars->m_branchCount, minFill);
        this->PickSeeds(parVars);

        while (((parVars->m_count[0] + parVars->m_count[1]) < parVars->m_total) &&
                (parVars->m_count[0] < (parVars->m_total - parVars->m_minFill)) &&
                (parVars->m_count[1] < (parVars->m_total - parVars->m_minFill))) {
            biggestDiff = CastElementTypeReal(-1);
            for (int index = 0; index < parVars->m_total; ++index) {
                if (PartitionVars::kNotTaken == parVars->m_partition[index]) {
                    Rect* curRect = &parVars->m_branchBuf[index].m_rect;
                    Rect rect0 = this->CombineRect(curRect, &parVars->m_cover[0]);
                    Rect rect1 = this->CombineRect(curRect, &parVars->m_cover[1]);
                    ElementTypeReal growth0 = this->CalcRectVolume(&rect0) - parVars->m_area[0];
                    ElementTypeReal growth1 = this->CalcRectVolume(&rect1) - parVars->m_area[1];
                    ElementTypeReal diff = growth1 - growth0;
                    if (diff >= 0) {
                        group = 0;
                    } else {
                        group = 1;
                        diff = -diff;
                    }

                    if (diff > biggestDiff) {
                        biggestDiff = diff;
                        chosen = index;
                        betterGroup = group;
                    } else if ((diff == biggestDiff) && (parVars->m_count[group] < parVars->m_count[betterGroup])) {
                        chosen = index;
                        betterGroup = group;
                    }
                }
            }
            this->Classify(chosen, betterGroup, parVars);
        }

        // If one group too full, put remaining rects in the other
        if ((parVars->m_count[0] + parVars->m_count[1]) < parVars->m_total) {
            if (parVars->m_count[0] >= parVars->m_total - parVars->m_minFill) {
                group = 1;
            } else {
                group = 0;
            }
            for (int index = 0; index < parVars->m_total; ++index) {
                if (PartitionVars::kNotTaken == parVars->m_partition[index]) {
                    this->Classify(index, group, parVars);
                }
            }
        }

        RTreeAssert((parVars->m_count[0] + parVars->m_count[1]) == parVars->m_total);
        RTreeAssert((parVars->m_count[0] >= parVars->m_minFill) && 
            (parVars->m_count[1] >= parVars->m_minFill));
    }

    // Copy branches from the buffer into two nodes according to the partition.
    void LoadNodes(Node* nodeA, Node* nodeB, PartitionVars* parVars) {
        RTreeAssert(nodeA);
        RTreeAssert(nodeB);
        RTreeAssert(parVars);

        for (int index = 0; index < parVars->m_total; ++index) {
            RTreeAssert(parVars->m_partition[index] == 0 || parVars->m_partition[index] == 1);

            int targetNodeIndex = parVars->m_partition[index];
            std::array<Node*, 2> targetNodes{ nodeA, nodeB };

            // It is assured that AddBranch here will not cause a node split. 
            bool nodeWasSplit = this->AddBranch(&parVars->m_branchBuf[index], targetNodes[targetNodeIndex], nullptr);
            RTreeAssert(!nodeWasSplit);
        }
    }

    // Initialize a PartitionVars structure.
    void InitParVars(PartitionVars* parVars, int maxRects, int minFill) const {
        RTreeAssert(parVars);

        parVars->m_count[0] = parVars->m_count[1] = 0;
        parVars->m_area[0] = parVars->m_area[1] = kElementTypeRealZero;
        parVars->m_total = maxRects;
        parVars->m_minFill = minFill;
        for (int index = 0; index < maxRects; ++index) {
            parVars->m_partition[index] = PartitionVars::kNotTaken;
        }
    }

    void PickSeeds(PartitionVars* parVars) {
        int seed0 = 0, seed1 = 0;
        ElementTypeReal worst = kElementTypeRealZero;
        ElementTypeReal waste = kElementTypeRealZero;
        std::array<ElementTypeReal, _MaxNodeCount + 1> area{ kElementTypeRealZero, };

        for (int index = 0; index < parVars->m_total; ++index) {
            area[index] = this->CalcRectVolume(&parVars->m_branchBuf[index].m_rect);
        }

        worst = -parVars->m_coverSplitArea - 1;
        for (int indexA = 0; indexA < parVars->m_total - 1; ++indexA) {
            for (int indexB = indexA + 1; indexB < parVars->m_total; ++indexB) {
                auto oneRect = CombineRect(&parVars->m_branchBuf[indexA].m_rect, &parVars->m_branchBuf[indexB].m_rect);
                waste = this->CalcRectVolume(&oneRect) - area[indexA] - area[indexB];
                if (waste > worst) {
                    worst = waste;
                    seed0 = indexA;
                    seed1 = indexB;
                }
            }
        }

        this->Classify(seed0, 0, parVars);
        this->Classify(seed1, 1, parVars);
    }

    // Put a branch in one of the groups.
    void Classify(int index, int group, PartitionVars* parVars) {
        RTreeAssert(parVars);
        RTreeAssert(PartitionVars::kNotTaken == parVars->m_partition[index]);

        parVars->m_partition[index] = group;

        // Calculate combined rect
        if (parVars->m_count[group] == 0) {
            parVars->m_cover[group] = parVars->m_branchBuf[index].m_rect;
        } else {
            parVars->m_cover[group] = this->CombineRect(&parVars->m_branchBuf[index].m_rect, &parVars->m_cover[group]);
        }

        // Calculate volume of combined rect
        parVars->m_area[group] = this->CalcRectVolume(&parVars->m_cover[group]);

        ++parVars->m_count[group];
    }

    // Delete a data rectangle from an index structure.
    // Pass in a pointer to a Rect, the tid of the record, ptr to ptr to root node.
    // Returns 1 if record not found, 0 if success.
    // RemoveRect provides for eliminating the root.
    bool RemoveRect(Rect* rect, const DataType& id, Node** root) {
        RTreeAssert(rect && root);
        RTreeAssert(*root);

        ListNode* reInsertList{ nullptr };

        if (!this->RemoveRectRec(rect, id, *root, &reInsertList)) {
            // Found and deleted a data item
            // Reinsert any branches from eliminated nodes
            while (reInsertList) {
                Node* tempNode = reInsertList->m_node;

                for (int index = 0; index < tempNode->m_count; ++index) {
                    // TODO go over this code. should I use (tempNode->m_level - 1)?
                    this->InsertRect(tempNode->m_branch[index],
                        root,
                        tempNode->m_level);
                }

                ListNode* remLNode = reInsertList;
                reInsertList = reInsertList->m_next;

                this->FreeNode(remLNode->m_node);
                this->FreeListNode(remLNode);
            }

            // Check for redundant root (not leaf, 1 child) and eliminate TODO replace
            // if with while? In case there is a whole branch of redundant roots...
            if ((*root)->m_count == 1 && (*root)->IsInternalNode()) {
                auto tempNode = (*root)->m_branch[0].m_child;

                RTreeAssert(tempNode);
                this->FreeNode(*root);
                *root = tempNode;
            }
            return false;
        } else {
            return true;
        }
    }

    // Delete a rectangle from non-root part of an index structure.
    // Called by RemoveRect.  Descends tree recursively,
    // merges branches on the way back up.
    // Returns 1 if record not found, 0 if success.
    bool RemoveRectRec(Rect* rect, const DataType& id, Node* node, ListNode** listNode) {
        RTreeAssert(rect && node && listNode);
        RTreeAssert(node->m_level >= 0);

        if (node->IsInternalNode())  // not a leaf node
        {
            for (int index = 0; index < node->m_count; ++index) {
                if (this->Overlap(rect, &(node->m_branch[index].m_rect))) {
                    if (!this->RemoveRectRec(rect, id, node->m_branch[index].m_child, listNode)) {
                        if (node->m_branch[index].m_child->m_count >= _MinNodeCount) {
                            // child removed, just resize parent rect
                            node->m_branch[index].m_rect = NodeCover(node->m_branch[index].m_child);
                        } else {
                            // child removed, not enough entries in node, eliminate node
                            this->ReInsert(node->m_branch[index].m_child, listNode);
                            this->DisconnectBranch(node, index); // Must return after this call as count has changed
                        }
                        return false;
                    }
                }
            }
            return true;
        } else // A leaf node
        {
            for (int index = 0; index < node->m_count; ++index) {
                if (node->m_branch[index].m_data == id) {
                    this->DisconnectBranch(node, index); // Must return after this call as count has changed
                    return false;
                }
            }
            return true;
        }
    }

    // Allocate space for a node in the list used in DeletRect to
    // store Nodes that are too empty.
    ListNode* AllocListNode() {
#ifdef RTREE_DONT_USE_MEMPOOLS
        return new ListNode();
#else // RTREE_DONT_USE_MEMPOOLS
        // EXAMPLE
#endif // RTREE_DONT_USE_MEMPOOLS
    }

    void FreeListNode(ListNode* listNode) {
#ifdef RTREE_DONT_USE_MEMPOOLS
        delete listNode;
#else // RTREE_DONT_USE_MEMPOOLS
        // EXAMPLE
#endif // RTREE_DONT_USE_MEMPOOLS
    }

    // Decide whether two rectangles overlap.
    bool Overlap(Rect* rectA, Rect* rectB) const {
        RTreeAssert(rectA && rectB);

        for (int index = 0; index < kNumDimensions; ++index) {
            if (rectA->m_min[index] > rectB->m_max[index] ||
                rectB->m_min[index] > rectA->m_max[index]) {
                return false;
            }
        }
        return true;
    }

    // Add a node to the reinsertion list.  All its branches will later
    // be reinserted into the index structure.
    void ReInsert(Node* node, ListNode** listNode) {
        ListNode* newListNode{ nullptr };

        newListNode = this->AllocListNode();
        newListNode->m_node = node;
        newListNode->m_next = *listNode;
        *listNode = newListNode;
    }

    // Search in an index tree or subtree for all data retangles that overlap the argument rectangle.
    bool Search(Node* node, Rect* rect, int& foundCount, std::function<bool(const DataType&)> callback) const {
        RTreeAssert(node);
        RTreeAssert(node->m_level >= 0);
        RTreeAssert(rect);

        if (node->IsInternalNode()) {
            // This is an internal node in the tree
            for (int index = 0; index < node->m_count; ++index) {
                if (this->Overlap(rect, &node->m_branch[index].m_rect)) {
                    if (!this->Search(node->m_branch[index].m_child, rect, foundCount, callback)) {
                        // The callback indicated to stop searching
                        return false;
                    }
                }
            }
        } else {
            // This is a leaf node
            for (int index = 0; index < node->m_count; ++index) {
                if (this->Overlap(rect, &node->m_branch[index].m_rect)) {
                    const auto& id = node->m_branch[index].m_data;
                    ++foundCount;

                    if (callback && !callback(id)) {
                        return false; // Don't continue searching
                    }
                }
            }
        }

        return true; // Continue searching
    }

    void RemoveAllRec(Node* node) {
        RTreeAssert(node);
        RTreeAssert(node->m_level >= 0);

        if (node->IsInternalNode()) // This is an internal node in the tree
        {
            for (int index = 0; index < node->m_count; ++index) {
                this->RemoveAllRec(node->m_branch[index].m_child);
            }
        }
        this->FreeNode(node);
    }

    void Reset() {
#ifdef RTREE_DONT_USE_MEMPOOLS
        // Delete all existing nodes
        this->RemoveAllRec(m_root);
#else // RTREE_DONT_USE_MEMPOOLS
        // Just reset memory pools.  We are not using complex types
        // EXAMPLE
#endif // RTREE_DONT_USE_MEMPOOLS
    }

    void CountRec(Node* node, int& count) const {
        if (node->IsInternalNode()) { // not a leaf node
            for (int index = 0; index < node->m_count; ++index) {
                this->CountRec(node->m_branch[index].m_child, count);
            }
        } else { // A leaf node
            count += node->m_count;
        }
    }

    bool SaveRec(Node* node, RTFileStream& stream) const {
        stream.Write(node->m_level);
        stream.Write(node->m_count);

        if (node->IsInternalNode()) { // not a leaf node
            for (int index = 0; index < node->m_count; ++index) {
                const auto& curBranch = node->m_branch[index];

                stream.WriteArray(curBranch->m_rect.m_min, kNumDimensions);
                stream.WriteArray(curBranch->m_rect.m_max, kNumDimensions);

                this->SaveRec(curBranch->m_child, stream);
            }
        } else { // A leaf node
            for (int index = 0; index < node->m_count; ++index) {
                const auto& curBranch = &node->m_branch[index];

                stream.WriteArray(curBranch->m_rect.m_min, kNumDimensions);
                stream.WriteArray(curBranch->m_rect.m_max, kNumDimensions);

                stream.Write(curBranch->m_data);
            }
        }

        return true; // Should do more error checking on I/O operations
    }

    bool LoadRec(Node* node, RTFileStream& stream) {
        stream.Read(node->m_level);
        stream.Read(node->m_count);

        if (node->IsInternalNode()) { // not a leaf node
            for (int index = 0; index < node->m_count; ++index) {
                Branch* curBranch = &node->m_branch[index];

                stream.ReadArray(curBranch->m_rect.m_min, kNumDimensions);
                stream.ReadArray(curBranch->m_rect.m_max, kNumDimensions);

                curBranch->m_child = AllocNode();
                this->LoadRec(curBranch->m_child, stream);
            }
        } else { // A leaf node
            for (int index = 0; index < node->m_count; ++index) {
                Branch* curBranch = &node->m_branch[index];

                stream.ReadArray(curBranch->m_rect.m_min, kNumDimensions);
                stream.ReadArray(curBranch->m_rect.m_max, kNumDimensions);

                stream.Read(curBranch->m_data);
            }
        }

        return true; // Should do more error checking on I/O operations
    }

    void CopyRec(Node* current, Node* other) {
        current->m_level = other->m_level;
        current->m_count = other->m_count;

        if (current->IsInternalNode()) { // not a leaf node
            for (int index = 0; index < current->m_count; ++index) {
                auto& currentBranch = current->m_branch[index];
                const auto& otherBranch = other->m_branch[index];

                std::copy(otherBranch.m_rect.m_min,
                    otherBranch.m_rect.m_min + kNumDimensions,
                    currentBranch.m_rect.m_min);

                std::copy(otherBranch.m_rect.m_max,
                    otherBranch.m_rect.m_max + kNumDimensions,
                    currentBranch.m_rect.m_max);

                currentBranch.m_child = AllocNode();
                this->CopyRec(currentBranch.m_child, otherBranch.m_child);
            }
        } else { // A leaf node
            for (int index = 0; index < current->m_count; ++index) {
                auto& currentBranch = current->m_branch[index];
                const auto& otherBranch = other->m_branch[index];

                std::copy(otherBranch.m_rect.m_min,
                    otherBranch.m_rect.m_min + kNumDimensions,
                    currentBranch.m_rect.m_min);

                std::copy(otherBranch.m_rect.m_max,
                    otherBranch.m_rect.m_max + kNumDimensions,
                    currentBranch.m_rect.m_max);

                currentBranch.m_data = otherBranch.m_data;
            }
        }
    }

    Node* m_root = nullptr;                                         ///< Root of tree
    ElementTypeReal m_unitSphereVolume = kElementTypeRealZero;      ///< Unit sphere constant for required number of dimensions
};

#endif //RTREE_H
