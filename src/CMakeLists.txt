cmake_minimum_required(VERSION 3.10)

find_package(jsoncpp REQUIRED)

add_executable(sample_based_planner
    main.cc
    planner.cc
    check_valid.cc
    rrt.cc
    rrt_connect.cc
    rrt_star.cc
    prm.cc)

target_include_directories(sample_based_planner PRIVATE
    include)

target_link_libraries(sample_based_planner PRIVATE
    jsoncpp_lib)

