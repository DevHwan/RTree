//
// MemoryTest.cpp
//
// This demonstrates a use of RTree
//

// RTree
#include <RTree.h>

#include <stdio.h>
#include <memory.h>
#ifdef WIN32
#include <crtdbg.h>
#endif //WIN32

#include <random>

// Use CRT Debug facility to dump memory leaks on app exit
#ifdef WIN32
  // These two are for MSVS 2005 security consciousness until safe std lib funcs are available
#pragma warning(disable : 4996) // Deprecated functions
#define _CRT_SECURE_NO_DEPRECATE // Allow old unsecure standard library functions, Disable some 'warning C4996 - function was deprecated'

// The following macros set and clear, respectively, given bits
// of the C runtime library debug flag, as specified by a bitmask.
#ifdef   _DEBUG
#define  SET_CRT_DEBUG_FIELD(a) \
              _CrtSetDbgFlag((a) | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG))
#define  CLEAR_CRT_DEBUG_FIELD(a) \
              _CrtSetDbgFlag(~(a) & _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG))
#else
#define  SET_CRT_DEBUG_FIELD(a)   ((void) 0)
#define  CLEAR_CRT_DEBUG_FIELD(a) ((void) 0)
#endif
#endif //WIN32

//
// Get a random float b/n two values
// The returned value is >= min && < max (exclusive of max)
//
static std::random_device sRandomDevice;
static std::mt19937 sMT(sRandomDevice());
static float RandFloat(float a_min, float a_max)
{
    std::uniform_real_distribution<float> distribution(a_min, a_max);
    return distribution(sMT);
}


/// Simplify handling of 3 dimensional coordinate
struct Vec3 {
    /// Default constructor
    Vec3() noexcept = default;

    /// Construct from three elements
    constexpr Vec3(float x, float y, float z) noexcept : fVal{ x, y, z } {}

    /// Add two vectors and return result
    Vec3 operator+ (const Vec3& other) const noexcept {
        return Vec3(fVal[0] + other.fVal[0],
            fVal[1] + other.fVal[1],
            fVal[2] + other.fVal[2]);
    }

    float fVal[3] = { 0.0f, };     ///< 3 float components for axes or dimensions
};


static bool BoxesIntersect(const Vec3& boxMinA, const Vec3& boxMaxA,
    const Vec3& boxMinB, const Vec3& boxMaxB)
{
    if (boxMinA.fVal[0] > boxMaxB.fVal[0] || boxMaxA.fVal[0] < boxMinB.fVal[0])
        return false;
    if (boxMinA.fVal[1] > boxMaxB.fVal[1] || boxMaxA.fVal[1] < boxMinB.fVal[1])
        return false;
    if (boxMinA.fVal[2] > boxMaxB.fVal[2] || boxMaxA.fVal[2] < boxMinB.fVal[2])
        return false;
    return true;
}


/// A user type to test with, instead of a simple type such as an 'int'
struct SomeThing {
    SomeThing() {
        ++sOutstandingAllocs;
    }
    ~SomeThing() {
        --sOutstandingAllocs;
    }

    int fCreationCounter = 0;                       ///< Just a number for identifying within test program
    Vec3 fMin, fMax;                                ///< Minimal bounding rect, values must be known and constant in order to remove from RTree

    static int sOutstandingAllocs;                 ///< Count how many outstanding objects remain
};

/// Init static
int SomeThing::sOutstandingAllocs = 0;


/// A callback function to obtain query results in this implementation
bool QueryResultCallback(SomeThing* data)
{
    printf("search found %d\n", data->fCreationCounter);

    return true;
}


int main(int argc, char* argv[])
{
    constexpr const int kNumObjects = 40;       // Number of objects in test set
    constexpr const int kFracObjects = 4;
    static_assert(kNumObjects > kFracObjects, "kNumObjects must be bigger than kFracObjects for this test");

    constexpr const float kMaxWorldSize = 10.0f;
    constexpr const float kFracWorldSize = kMaxWorldSize / 2;

    // typedef the RTree useage just for conveniance with iteration
    using SomeThingTree = RTree<SomeThing*, float, 3>;

    SomeThing* thingArray[kNumObjects * 2] = { nullptr, }; // Store objects in another container to test with, sized larger than we need

    // Create intance of RTree

    SomeThingTree tree;


    // Add some nodes
    int counter = 0;
    for (int index = 0; index < kNumObjects; ++index)
    {
        auto newThing = new SomeThing{};

        newThing->fCreationCounter = counter++;
        newThing->fMin = Vec3(RandFloat(-kMaxWorldSize, kMaxWorldSize), RandFloat(-kMaxWorldSize, kMaxWorldSize), RandFloat(-kMaxWorldSize, kMaxWorldSize));
        Vec3 extent = Vec3(RandFloat(0, kFracWorldSize), RandFloat(0, kFracWorldSize), RandFloat(0, kFracWorldSize));
        newThing->fMax = newThing->fMin + extent;

        thingArray[counter - 1] = newThing;

        tree.Insert(newThing->fMin.fVal, newThing->fMax.fVal, newThing);
        printf("inserting %d\n", newThing->fCreationCounter);
    }

    printf("tree count = %d\n", tree.Count());

    int numToDelete = kNumObjects / kFracObjects;
    int numToStep = kFracObjects;

    // Delete some nodes
    for (int index = 0; index < kNumObjects; index += numToStep)
    {
        auto curThing = thingArray[index];

        if (curThing)
        {
            tree.Remove(curThing->fMin.fVal, curThing->fMax.fVal, curThing);
            printf("removing %d\n", curThing->fCreationCounter);

            delete curThing;
            thingArray[index] = nullptr;
        }
    }

    printf("tree count = %d\n", tree.Count());

    // Add some more nodes
    for (int index = 0; index < numToDelete; ++index)
    {
        auto newThing = new SomeThing{};

        newThing->fCreationCounter = counter++;
        newThing->fMin = Vec3(RandFloat(-kMaxWorldSize, kMaxWorldSize), RandFloat(-kMaxWorldSize, kMaxWorldSize), RandFloat(-kMaxWorldSize, kMaxWorldSize));
        Vec3 extent = Vec3(RandFloat(0, kFracWorldSize), RandFloat(0, kFracWorldSize), RandFloat(0, kFracWorldSize));
        newThing->fMax = newThing->fMin + extent;

        thingArray[counter - 1] = newThing;

        tree.Insert(newThing->fMin.fVal, newThing->fMax.fVal, newThing);
        printf("inserting %d\n", newThing->fCreationCounter);
    }

    printf("tree count = %d\n", tree.Count());

    Vec3 searchMin(0, 0, 0);
    Vec3 searchMax(kFracWorldSize, kFracWorldSize, kFracWorldSize);
    tree.Search(searchMin.fVal, searchMax.fVal, &QueryResultCallback);

    // NOTE: Even better than just dumping text, it would be nice to render the 
    // tree contents and search result for visualization.


    // List values.  Iterator is NOT delete safe
    SomeThingTree::Iterator it;
    for (tree.GetFirst(it); !tree.IsNull(it); tree.GetNext(it))
    {
        SomeThing* curThing = tree.GetAt(it);

        if (BoxesIntersect(searchMin, searchMax, curThing->fMin, curThing->fMax))
        {
            printf("brute found %d\n", curThing->fCreationCounter);
        }
    }

    // Delete our nodes, NOTE, we are NOT deleting the tree nodes, just our data
    // of course the tree will now contain invalid pointers that must not be used any more.
    for (tree.GetFirst(it); !tree.IsNull(it); tree.GetNext(it))
    {
        SomeThing* removeElem = tree.GetAt(it);
        if (removeElem)
        {
            printf("deleting %d\n", removeElem->fCreationCounter);
            delete removeElem;
        }
    }

    // Remove all contents (This would have happened automatically during destructor)
    tree.RemoveAll();

    if (SomeThing::sOutstandingAllocs > 0)
    {
        printf("Memory leak!\n");
        printf("s_outstandingAllocs = %d\n", SomeThing::sOutstandingAllocs);
    }
    else
    {
        printf("No memory leaks detected by app\n");
    }

#ifdef WIN32
    // Use CRT Debug facility to dump memory leaks on app exit
    SET_CRT_DEBUG_FIELD(_CRTDBG_LEAK_CHECK_DF);
#endif //WIN32

    return 0;
}

