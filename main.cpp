#include <cstdio>
#include <iostream>
#include "external/glad/glad.h"
#include <GLFW/glfw3.h>
#include "include/Grid.h"
#include "include/camera.h"
#include "include/Particle.h"
#include "include/shader.h"

const int GLOBAL_WIDTH = 640.0;
const int GLOBAL_HEIGHT = 480.0;
const int GRID_WIDTH = 30.0;
const float SPACING = 5.f / (float)GRID_WIDTH;
const float DELTA_T = 1e-3;
const float THETA_C = 2.5e-2;
const float THETA_S = 7.5e-3;
const float XI = 10.0;
const float PARTICLE_NUM = 300;
const int FRAME_PER_SEC = 30;
const int VIDEO_LENGTH = 12000;
const float MU = 0.2;
using namespace glm;

int main() {
    glfwInit();
    glfwSetTime(0);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(GLOBAL_WIDTH, GLOBAL_HEIGHT, "Snow Simulation", nullptr, nullptr);
    if (!window) {
        std::cout << "Failed to load the window";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cout << "Failed to init Glad";
        return -1;
    }

    std::unique_ptr<Grid> my_grid = std::make_unique<Grid>(GRID_WIDTH, GRID_WIDTH, GRID_WIDTH, SPACING);
    Camera my_camera {glm::vec3 {0, 0, 7.0}};
    Shader my_shader {"../include/shaders/camera.vert", "../include/shaders/simple.frag"};
    glm::mat4 world_to_model_mat;
    glm::mat4 model_to_world_mat;
    world_to_model_mat = glm::translate(world_to_model_mat, my_grid->_origin);
    model_to_world_mat = glm::translate(model_to_world_mat, -my_grid->_origin);

    // here we generate a random snow ball :
    float radius = pow(3. * PARTICLE_NUM / (16 * PI), 1./3) * SPACING * 1.5;
    my_grid->createParticles(PARTICLE_NUM, radius);

    glm::vec4 pink(1.0f, 0.5f, 1.0f, 1.0f);
    // Make the ground plane
    vec4 ground_color = vec4(0.3f, 0.3f, 0.3f, 1.0f);
    vec3 ground_origin(-my_grid->_origin);
    vec3 axis_x(my_grid->_x_physical, 0, 0);
    vec3 axis_y(0, my_grid->_y_physical, 0);
    vec3 axis_z(0, 0, my_grid->_z_physical);
    Plane* ground_rect = new Plane(ground_origin, axis_x, axis_z, model_to_world_mat, world_to_model_mat, ground_color, MU);
    my_shader.use();
    int num_lines = (my_grid->_z_coord + 1) * ((my_grid->_x_coord + 1) + (my_grid->_y_coord + 1)) + ((my_grid->_x_coord + 1) * (my_grid->_y_coord + 1));
    int num_vertices = num_lines * 2;
    float grid_vertices[num_vertices * 3];
    memset(grid_vertices, 0, num_vertices * 3 * sizeof(float));
    int t = 0;
    for (int z_coord = 0; z_coord <= my_grid->_z_coord; ++z_coord) {
        for (int x_coord = 0; x_coord <= my_grid->_x_coord; ++x_coord) {
            glm::vec3 bottom(float(x_coord), 0, float(z_coord));
            glm::vec3 top(float(x_coord), my_grid->_y_coord, float(z_coord));
            grid_vertices[t + 0] = bottom.x;
            grid_vertices[t + 1] = bottom.y;
            grid_vertices[t + 2] = bottom.z;
            grid_vertices[t + 3] = top.x;
            grid_vertices[t + 4] = top.y;
            grid_vertices[t + 5] = top.z;

            t += 6;
        }

        for (int y_coord = 0; y_coord <= my_grid->_y_coord; ++y_coord) {
            // lines from x=0 to x=res_z at z=0
            glm::vec3 bottom(0.0f, float(y_coord), float(z_coord));
            glm::vec3 top(my_grid->_x_coord, float(y_coord), float(z_coord));
            grid_vertices[t + 0] = bottom.x;
            grid_vertices[t + 1] = bottom.y;
            grid_vertices[t + 2] = bottom.z;
            grid_vertices[t + 3] = top.x;
            grid_vertices[t + 4] = top.y;
            grid_vertices[t + 5] = top.z;

            t += 6;
        }
    }
    for (int x_coord = 0; x_coord <= my_grid->_x_coord; ++x_coord) {
        for (int y_coord = 0; y_coord <= my_grid->_y_coord; ++y_coord) {
            // lines from x=0 to x=res_z at z=0
            glm::vec3 front(float(x_coord), float(y_coord), 0.0);
            glm::vec3 back(float(x_coord), float(y_coord), my_grid->_z_coord);
            grid_vertices[t + 0] = front.x;
            grid_vertices[t + 1] = front.y;
            grid_vertices[t + 2] = front.z;
            grid_vertices[t + 3] = back.x;
            grid_vertices[t + 4] = back.y;
            grid_vertices[t + 5] = back.z;

            t += 6;
        }
    }

    unsigned int grid_VAO, grid_VBO;
    unsigned int particle_VAO, particle_VBO;
    unsigned int grid_force_VAO, grid_force_VBO;
    unsigned int particle_velocity_VAO, particle_velocity_VBO;

    glGenVertexArrays(1, &grid_VAO);
    glGenBuffers(1, &grid_VBO);

    glBindVertexArray(grid_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, grid_VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(grid_vertices), grid_vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    float vs = 0.5;
    GLfloat cube_vertices[] = {
            -vs,-vs,-vs, // triangle 1 : begin
            -vs,-vs, vs,
            -vs, vs, vs, // triangle 1 : end
            vs, vs,-vs, // triangle 2 : begin
            -vs,-vs,-vs,
            -vs, vs,-vs, // triangle 2 : end
            vs,-vs, vs,
            -vs,-vs,-vs,
            vs,-vs,-vs,
            vs, vs,-vs,
            vs,-vs,-vs,
            -vs,-vs,-vs,
            -vs,-vs,-vs,
            -vs, vs, vs,
            -vs, vs,-vs,
            vs,-vs, vs,
            -vs,-vs, vs,
            -vs,-vs,-vs,
            -vs, vs, vs,
            -vs,-vs, vs,
            vs,-vs, vs,
            vs, vs, vs,
            vs,-vs,-vs,
            vs, vs,-vs,
            vs,-vs,-vs,
            vs, vs, vs,
            vs,-vs, vs,
            vs, vs, vs,
            vs, vs,-vs,
            -vs, vs,-vs,
            vs, vs, vs,
            -vs, vs,-vs,
            -vs, vs, vs,
            vs, vs, vs,
            -vs, vs, vs,
            vs,-vs, vs
    };

    glGenVertexArrays(1, &particle_VAO);
    glGenBuffers(1, &particle_VBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(particle_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, particle_VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);



    glGenVertexArrays(1, &grid_force_VAO);
    glGenBuffers(1, &grid_force_VBO);

    glBindVertexArray(grid_force_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, grid_force_VBO);
//  glBufferData(GL_ARRAY_BUFFER, sizeof(grid_vertices), grid_vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);


    glGenVertexArrays(1, &particle_velocity_VAO);
    glGenBuffers(1, &particle_velocity_VBO);

    glBindVertexArray(particle_velocity_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, particle_velocity_VBO);
//  glBufferData(GL_ARRAY_BUFFER, sizeof(grid_vertices), grid_vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    int w_ = 0;
    int h_ = 0;
    glfwGetFramebufferSize(window, &w_, &h_);
    int frame_counter = 0;
    float frame_duration = float(1. / FRAME_PER_SEC);
    float simulation_steps = int(std::round(frame_duration / DELTA_T));
    while (!glfwWindowShouldClose(window) and frame_counter < VIDEO_LENGTH) {
        my_shader.use();
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);

        glEnable(GL_DEPTH_TEST);
        for (int i = 0; i < simulation_steps; i++) {
            // Not sure about the order of these updates:
            my_grid->simulate(XI, DELTA_T, vec3 {0, -9.81, 0}, THETA_C, THETA_S, *ground_rect);
        }
        my_shader.use();
        glm::mat4 projection = glm::perspective(glm::radians(my_camera.Zoom), (float)GLOBAL_WIDTH / (float)GLOBAL_HEIGHT, 0.1f, 100.0f);
        my_shader.setMat4("projection", projection);
        // camera/view transformation
        glm::mat4 view = my_camera.GetViewMatrix();
        my_shader.setMat4("view", view);
        for (const auto &particle : my_grid->_particles) {
            float len = cbrt(particle->_volume);
            glm::mat4 particle_model;
            particle_model = glm::scale(particle_model, glm::vec3(len, len, len));
            particle_model = model_to_world_mat * particle_model;
            particle_model = glm::translate(glm::mat4(), particle->_pos) * particle_model;
            my_shader.setVec4("in_color", glm::vec4(0.7, 0.7, 0.7, 0.7));
            my_shader.setMat4("model", particle_model);
            glBindVertexArray(particle_VAO);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        ground_rect->render(&my_shader);
        glfwPollEvents();
        glfwSwapBuffers(window);
        frame_counter++;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
};