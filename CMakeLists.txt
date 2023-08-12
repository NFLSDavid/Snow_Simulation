cmake_minimum_required(VERSION 3.25)
project(Snow_Simulation)

set(CMAKE_CXX_STANDARD 14)
set(GLAD_DIR "external/glad")
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

include_directories(.)
add_library("glad" "${GLAD_DIR}/glad.c" external/glad/glad.h)
add_subdirectory(external/glfw)
add_executable(
        Snow_Simulation
        main.cpp
        library/Grid.cpp
        include/Grid.h
        library/Particle.cpp
        include/Particle.h
        library/plane.cpp
        include/plane.h
        include/shader.h
        include/camera.h
        library/Relate_Math.cpp
        include/Relate_Math.h
)

# setup glad, glfw, glm, and Eigen;
target_link_directories(Snow_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include)
target_link_libraries(Snow_Simulation glfw)
target_include_directories("glad" PUBLIC "${CLAD_DIR}/include")
target_include_directories(Snow_Simulation PUBLIC "${GLAD_DIR}/include")
target_link_libraries(Snow_Simulation "glad" "${CMAKE_DL_LIBS}")
include_directories("external/glm")
target_link_libraries(Snow_Simulation Eigen3::Eigen)