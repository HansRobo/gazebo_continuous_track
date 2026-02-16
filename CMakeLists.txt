cmake_minimum_required(VERSION 3.16)
project(gazebo_continuous_track)

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

set(GZ_SIM_TARGET "")
set(GZ_SIM_VERSION "")
foreach(version IN ITEMS 8 7 6)
  find_package(gz-sim${version} QUIET)
  if(gz-sim${version}_FOUND)
    set(GZ_SIM_TARGET gz-sim${version}::gz-sim${version})
    set(GZ_SIM_VERSION ${version})
    break()
  endif()
endforeach()

if(NOT GZ_SIM_TARGET)
  message(FATAL_ERROR "Could not find gz-sim6, gz-sim7, or gz-sim8.")
endif()

add_library(gz_continuous_track_simple_system SHARED
  src/continuous_track_simple_system.cpp
)
target_compile_features(gz_continuous_track_simple_system PRIVATE cxx_std_17)
target_link_libraries(gz_continuous_track_simple_system PRIVATE
  ${GZ_SIM_TARGET}
  gz-plugin2::gz-plugin2
)

add_library(gz_continuous_track_system SHARED
  src/continuous_track_system.cpp
)
target_compile_features(gz_continuous_track_system PRIVATE cxx_std_17)
target_link_libraries(gz_continuous_track_system PRIVATE
  ${GZ_SIM_TARGET}
  gz-plugin2::gz-plugin2
)

install(
  TARGETS
    gz_continuous_track_simple_system
    gz_continuous_track_system
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY urdf_xacro sdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
