//
// Created by 方文宇 on 2023/7/31.
//

#include "include/Particle.h"

Particle::Particle(float mass, glm::vec3 pos) : _mass{mass}, _pos{pos} {}

Particle::~Particle() = default;
