cmake_minimum_required(VERSION 3.5)
project(ft_sensor VERSION 0.0.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall )
endif()

#### Find dependencies ####
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(${PROJECT_NAME} SHARED
            src/AdmittanceController.cpp
            src/HindranceDetector.cpp
            src/OnRobotForceTorqueSensor.cpp
            )   

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    pthread  
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
        
install(DIRECTORY include/ DESTINATION include)

# executable for testing
add_executable(testFtSensor test/testFTSensor.cpp)
target_link_libraries(testFtSensor ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()
