add_library(yocto_raytrace yocto_raytrace.h yocto_raytrace.cpp)

set_target_properties(yocto_raytrace PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yocto_raytrace PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yocto_raytrace yocto)
