//
// Created by 方文宇 on 2023/8/3.
//

#ifndef SNOW_SIMULATION_PLANE_H
#define SNOW_SIMULATION_PLANE_H
#include "external/glm/glm/glm.hpp"
#include <glm/gtx/norm.hpp>
#include "glm/gtc/matrix_transform.hpp"
#include "include/shader.h"

class Plane {
public:
    glm::vec3 collide(glm::vec3 pos, glm::vec3 &velocity, float delta_t);
    void render(Shader *shader);
    Plane(glm::vec3 &origin, glm::vec3 &u, glm::vec3 &v, glm::mat4 model_to_world, glm::mat4 world_to_model, glm::vec4 color, float mu);
private:
    glm::vec3 _origin;
    glm::vec3 _u;
    glm::vec3 _v;
    glm::mat4 _model_to_world;
    glm::mat4 _world_to_model;
    glm::vec3 _normals;
    float _mu;
    const int static _num_vertices = 4;
    float _vertices[_num_vertices * 3];
    unsigned int _VAO, _VBO;
    glm::vec4 _color;
};



#endif //SNOW_SIMULATION_PLANE_H
