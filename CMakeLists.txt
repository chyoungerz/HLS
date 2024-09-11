cmake_minimum_required(VERSION 3.27.0)
project(CCVRP VERSION 0.1.0)

# 为当前路径以及子目录的源文件加入由-D预编译定义
#IF (CMAKE_BUILD_TYPE STREQUAL Debug)
#    ADD_DEFINITIONS(-DDEBUG)
#ENDIF()

# 优先级
ADD_DEFINITIONS(-DPR)

# 设置C++编译参数(CMAKE_CXX_FLAGS是全局变量)
set(CMAKE_CXX_FLAGS "-Wall -std=gnu++23")

# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定为C++23 版本
# set(CMAKE_CXX_STANDARD 23)

#编译选项
set(CMAKE_CXX_FLAGS_Debug "${CMAKE_CXX_FLAGS} -O0 -g")
set(CMAKE_CXX_FLAGS_Release "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

#头文件目录
#include_directories()

# 添加子目录
#add_subdirectory()

#源文件目录
#aux_source_directory(. DIR_SRCS)


# 分别设置Debug和Release输出目录
set(CMAKE_LIBRATY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug/lib)
set(CMAKE_LIBRATY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release/bin)

# 指定生成目标 
#add_executable(vrp ${DIR_SRCS})
add_executable(vrp main.cpp operator.cpp solution.cpp NSearch.cpp heuristic.cpp)


# 测试
enable_testing()

set(TESTS_A
  "A-n32-k5" "A-n33-k5" "A-n33-k6" "A-n34-k5" "A-n36-k5" "A-n37-k5" "A-n37-k6" "A-n38-k5" "A-n39-k5" "A-n39-k6"
  "A-n44-k6" "A-n45-k6" "A-n45-k7" "A-n46-k7" "A-n48-k7" "A-n53-k7" "A-n54-k7" "A-n55-k9" "A-n60-k9" "A-n61-k9"
  "A-n62-k8" "A-n63-k9" "A-n63-k10" "A-n64-k9" "A-n65-k9" "A-n69-k9" "A-n80-k10"
)
set(TESTS_B
  "B-n31-k5" "B-n34-k5" "B-n35-k5" "B-n38-k6" "B-n39-k5" "B-n41-k6" "B-n43-k6" "B-n44-k7" "B-n45-k5" "B-n45-k6" "B-n50-k7" 
  "B-n50-k8" "B-n51-k7" "B-n52-k7" "B-n56-k7" "B-n57-k7" "B-n57-k9" "B-n63-k10" "B-n64-k9" "B-n66-k9" "B-n67-k10" "B-n68-k9"
)
set(TESTS_E
  "E-n22-k4" "E-n23-k3" "E-n30-k3" "E-n33-k4" "E-n51-k5" "E-n76-k7" "E-n76-k8" "E-n76-k10" "E-n76-k14" "E-n101-k8" "E-n101-k14"
)
set(TESTS_P
  "P-n16-k8" "P-n19-k2" "P-n20-k2" "P-n21-k2" "P-n22-k8" "P-n23-k8" "P-n40-k5" "P-n45-k5" "P-n50-k7" "P-n50-k8" "P-n50-k10" 
  "P-n51-k10" "P-n55-k7" "P-n55-k8" "P-n55-k10" "P-n60-k10" "P-n60-k15" "P-n65-k10" "P-n70-k10" "P-n76-k4" "P-n76-k5" "P-n101-k4"
)

foreach(TEST IN LISTS TESTS_A)
  add_test(NAME ${TEST} COMMAND vrp a/${TEST}-Prk.vrp 30 CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)
endforeach()
foreach(TEST IN LISTS TESTS_B)
  add_test(NAME ${TEST} COMMAND vrp b/${TEST}-Prk.vrp 30 CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)
endforeach()
foreach(TEST IN LISTS TESTS_E)
  add_test(NAME ${TEST} COMMAND vrp e/${TEST}-Prk.vrp 30 CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)
endforeach()
foreach(TEST IN LISTS TESTS_P)
  add_test(NAME ${TEST} COMMAND vrp p/${TEST}-Prk.vrp 30 CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing)
endforeach()

#set_tests_properties(p01 PROPERTIES PASS_CONFIGURE "0")