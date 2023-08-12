//
// Created by 方文宇 on 2023/8/3.
//

#include "include/plane.h"
using namespace glm;

vec3 Plane::collide(glm::vec3 pos, glm::vec3 &velocity, float delta_t) {
    vec3 velocity_rel = velocity;
    vec3 origin_model = vec3(_world_to_model * vec4(this->_origin, 1.0));
    vec3 next_origin_model = vec3(_world_to_model * vec4(this->_origin, 1.0));
    vec3 next_position = pos + velocity * delta_t;

    // Check if next_position enters infinite plane
    vec3 next_position_origin = next_position - origin_model;
    float offset = dot(pos - origin_model, _normals);
    float offset_next = dot(next_position - next_origin_model, _normals);
    if (abs(offset) < 0.0001 ||  offset * offset_next < 0) {
        // Check if next_position is within finite bounds

        // Projection to plane
        vec3 next_position_plane = next_position_origin - _normals * dot(next_position_origin, _normals);
        float proj_u = dot(next_position_plane, _u);
        float proj_v = dot(next_position_plane, _v);
        if (proj_u > 0 && proj_u < length2(_u)
            && proj_v > 0 && proj_v < length2(_v)) {
            // Friction collision
            vec3 outward_normal;
            if (dot(pos - origin_model, _normals) > 0) {
                outward_normal = _normals;
            } else {
                outward_normal = -_normals;
            }
            float v_n = dot(velocity_rel, outward_normal);
            vec3 velocity_tangent = velocity_rel - outward_normal * v_n;
            float mag_velocity_tangent = length(velocity_tangent);
            if (mag_velocity_tangent <= -_mu * v_n) {
                // Static friction
                return vec3(0.0);
            } else {
                // Dynamic friction
                return ((1 + _mu * v_n / mag_velocity_tangent) * velocity_tangent);
            }
        }
    }
    // No collision
    return velocity;
}

Plane::Plane(glm::vec3 &origin, glm::vec3 &u, glm::vec3 &v, glm::mat4 model_to_world, glm::mat4 world_to_model,
             glm::vec4 color, float mu)
             : _origin{origin}, _u{u}, _v{v}, _model_to_world{model_to_world},
             _world_to_model{world_to_model}, _color {color}, _mu {mu} {
    if (abs(glm::dot(_u, _v)) > 0) {
        throw "bad rectangle";
    }

    _normals = glm::normalize(glm::cross(_u, _v));

    glm::vec3 point0 = glm::vec3();
    glm::vec3 point1 = _u;
    glm::vec3 point2 = _u + _v;
    glm::vec3 point3 = _v;

    _vertices[0] = point0.x;
    _vertices[1] = point0.y;
    _vertices[2] = point0.z;
    _vertices[3] = point1.x;
    _vertices[4] = point1.y;
    _vertices[5] = point1.z;
    _vertices[6] = point2.x;
    _vertices[7] = point2.y;
    _vertices[8] = point2.z;
    _vertices[9] = point3.x;
    _vertices[10] = point3.y;
    _vertices[11] = point3.z;
    glGenVertexArrays(1, &_VAO);
    glGenBuffers(1, &_VBO);
    glBindVertexArray(_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, _VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_vertices), _vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void Plane::render(Shader *shader) {
    shader->use();
    shader->setVec4("in_color", _color);
    glm::mat4 model = glm::translate(glm::mat4(), _origin);
    shader->setMat4("model", model);
    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, _num_vertices);
}

