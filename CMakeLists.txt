# CMake version check
cmake_minimum_required(VERSION 3.10.0)

# Check build directory
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CMakeFiles/ directory. Then build out-of-source.")
endif()

# RTree project
project(
    RTree
    VERSION 0.0.1
)

# Test file
add_executable(Test_RTree
    Test.cpp
)
target_include_directories(Test_RTree
    PRIVATE
        ${RTree_SOURCE_DIR}
)
target_compile_features(Test_RTree
    PUBLIC
        cxx_std_11
)
if(MSVC)
    target_compile_options(Test_RTree
        PRIVATE
            /permissive-
            /sdl
            /Zc:wchar_t
    )
    target_compile_definitions(Test_RTree
        PRIVATE
            _CRT_SECURE_NO_WARNINGS
    )
endif()


# Memory Test file
add_executable(Test_Memory_RTree
    MemoryTest.cpp
)
target_include_directories(Test_Memory_RTree
    PRIVATE
        ${RTree_SOURCE_DIR}
)
target_compile_features(Test_Memory_RTree
    PUBLIC
        cxx_std_11
)
if(MSVC)
    target_compile_options(Test_Memory_RTree
        PRIVATE
            /permissive-
            /sdl
            /Zc:wchar_t
    )
    target_compile_definitions(Test_Memory_RTree
        PRIVATE
            _CRT_SECURE_NO_WARNINGS
    )
endif()

# Bad Data Test file
add_executable(Test_BadData_RTree
    TestBadData.cpp
)
target_include_directories(Test_BadData_RTree
    PRIVATE
        ${RTree_SOURCE_DIR}
)
target_compile_features(Test_BadData_RTree
    PUBLIC
        cxx_std_11
)
if(MSVC)
    target_compile_options(Test_BadData_RTree
        PRIVATE
            /permissive-
            /sdl
            /Zc:wchar_t
    )
    target_compile_definitions(Test_BadData_RTree
        PRIVATE
            _CRT_SECURE_NO_WARNINGS
    )
endif()
