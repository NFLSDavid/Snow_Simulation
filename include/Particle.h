//
// Created by 方文宇 on 2023/7/31.
//

#ifndef SNOW_SIMULATION_PARTICLE_H
#define SNOW_SIMULATION_PARTICLE_H
#include "external/glm/glm/glm.hpp"
#include <utility>

const float PARTICLE_MASS = 10;

class Particle {
public:
    float _mass;
    float _volume = 0;
    glm::vec3 _pos = glm::vec3 {0};
    glm::vec3 _velocity = glm::vec3 {0};
    glm::mat3 _deform_gradient_P; // F_P_p
    glm::mat3 _deform_gradient_E; // F_E_p
    std::pair<int, int> x_bound;
    std::pair<int, int> y_bound;
    std::pair<int, int> z_bound;
    Particle(float mass, glm::vec3 pos);
    ~Particle();
};


#endif //SNOW_SIMULATION_PARTICLE_H
