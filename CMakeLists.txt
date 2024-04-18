cmake_minimum_required(VERSION 3.15.0)
project(boost_serialization_1_84_bug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(GTest REQUIRED)

add_executable(test test.cpp)
target_link_libraries(test Boost::boost
Boost::system
Boost::filesystem
Boost::serialization
GTest::GTest GTest::Main)

