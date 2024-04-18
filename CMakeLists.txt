cmake_minimum_required(VERSION 3.15.0)
project(boost_serialization_1_84_bug LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(GTest REQUIRED)

add_library(test_lib SHARED cmd.cpp cmd2.cpp)
target_link_libraries(test_lib Boost::boost
Boost::system
Boost::filesystem
Boost::serialization
GTest::GTest GTest::Main)
set_target_properties(test_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

add_executable(test_lib_prog test.cpp)
target_link_libraries(test_lib_prog test_lib Boost::boost
Boost::system
Boost::filesystem
Boost::serialization
GTest::GTest GTest::Main)

add_executable(test_no_lib_prog test.cpp cmd.cpp cmd2.cpp)
target_link_libraries(test_no_lib_prog Boost::boost
Boost::system
Boost::filesystem
Boost::serialization
GTest::GTest GTest::Main)

enable_testing()

add_test(NAME test_no_lib COMMAND test_no_lib_prog)
add_test(NAME test_lib COMMAND test_lib_prog)
