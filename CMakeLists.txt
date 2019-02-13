cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_NORMAL)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable (PCL_NORMAL PCL_NORMAL.cpp)
target_link_libraries (PCL_NORMAL ${PCL_LIBRARIES})

add_executable (PCL_NORMAL2 PCL_NORMAL2.cpp)
target_link_libraries (PCL_NORMAL2 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL3 PCL_NORMAL3.cpp)
target_link_libraries (PCL_NORMAL3 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL4 PCL_NORMAL4.cpp)
target_link_libraries (PCL_NORMAL4 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL5 PCL_NORMAL5.cpp)
target_link_libraries (PCL_NORMAL5 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL6 PCL_NORMAL6.cpp)
target_link_libraries (PCL_NORMAL6 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL7 PCL_NORMAL7.cpp)
target_link_libraries (PCL_NORMAL7 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL8 PCL_NORMAL8.cpp)
target_link_libraries (PCL_NORMAL8 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL9 PCL_NORMAL9.cpp)
target_link_libraries (PCL_NORMAL9 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL10 PCL_NORMAL10.cpp)
target_link_libraries (PCL_NORMAL10 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL11 PCL_NORMAL11.cpp)
target_link_libraries (PCL_NORMAL11 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL_G2 PCL_NORMAL_G2.cpp)
target_link_libraries (PCL_NORMAL_G2 ${PCL_LIBRARIES})

add_executable (PCL_NORMAL_G3 PCL_NORMAL_G3.cpp)
target_link_libraries (PCL_NORMAL_G3 ${PCL_LIBRARIES})


add_executable (PCL_NORMAL_G4_show_something PCL_NORMAL_G4_show_something.cpp)
target_link_libraries (PCL_NORMAL_G4_show_something ${PCL_LIBRARIES})

