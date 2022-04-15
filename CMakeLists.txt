cmake_minimum_required(VERSION 3.13)
project(test)

include_directories(/home/zed/Project/OSG/OpenSceneGraph/include)

add_executable(test main.cpp elements/DatasetLoader.cpp)
target_link_libraries(test 
                     /home/zed/Project/OSG/OpenSceneGraph/build/lib/libosgViewer.so
                     /home/zed/Project/OSG/OpenSceneGraph/build/lib/libosgDB.so
                     /home/zed/Project/OSG/OpenSceneGraph/build/lib/libosg.so.202)