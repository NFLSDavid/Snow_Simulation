## Project Goal
For this Program, I did a snow physics-based simulation for a snow ball colliding the ground.

## Environment
My program runs under Mac 13.4.1 (catalina) and clang-1403.0.22.14.1, with OpenGL version 3. I use CMake to construct my program and use libraries like GLFW, GLM, GLAD ,and Eigen. 
Right-click the CMakeList in CLion and press "Reload CMake Project" to build the program. Then, press the green triangle button to run the program.

## Structure
In the external directory, I put all the external libraries I used in this program.
In the include directory, I put all the header files I used in this program.
In the library directory, I put all the libraries I used in this program.

In the Grid.cpp, all the objective steps are there and its head file is in the include directory. The Grid class is the main class of this program, which is in charge of updating the particles and nodes inside. The Node class is in charge of updating the velocity and position of the nodes. 
In the Particle.cpp, The Particle class is in charge of updating the velocity and position of the particles.
In the Plane.cpp, The Plane class is in charge of updating the collision of the plane.
In the Relate_Math.cpp, the Relate_Math class is in charge of calculating the related math problems such as B_spline and B_spline derivative, or force calculation.

## Acknowledgement
In the main.cpp, I set up the window and the camera, part of the code is from the tutorial in https://learnopengl.com/Getting-started, like shader.h, camera.h and their usage in the main.cpp.

## Objectives
1. Initialize every Node mass and velocity by using B_spline;
2. Calculate the Node volume and density by using B_spline and mass;
3. Calculate the Node forces by using B_spline_derivatives and Physics about particle force calculation;
4. Update the velocity with the external forces like Gravity;
5. Calculate the collision of nodes and particles the with the plane;
6. Update Deform Gradient Matrix and Plasticity;
7. Update FLIP and PIC velocity values;
8. Update the location and reset the Grid and make the algorithm work totally;
