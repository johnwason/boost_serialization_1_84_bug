cmake_minimum_required(VERSION 3.15.0)
project(boost_serialization_1_84_bug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(GTest REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)

add_executable(test test.cpp cmd.cpp)
target_link_libraries(test GTest::GTest GTest::Main tesseract::tesseract_environment tesseract::tesseract_common
)

