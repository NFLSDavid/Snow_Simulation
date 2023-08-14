//
// Created by 方文宇 on 2023/7/31.
//

#ifndef SNOW_SIMULATION_GRID_H
#define SNOW_SIMULATION_GRID_H
#include "external/glm/glm/glm.hpp"
#include <vector>
#include "Particle.h"
#include "include/Relate_Math.h"
#include <cmath>
#include "include/plane.h"
#include <glm/gtc/random.hpp>
#include <set>
#include <memory>

const float alpha = 0.95;
const float PI = 3.1415826;

struct Node {
    glm::vec3 _pos;
    glm::vec3 _velocity = glm::vec3 {0};
    glm::vec3 _updated_velocity = glm::vec3 {0};
    glm::vec3 _force = glm::vec3 {0};
    float _mass = 0;
    std::vector<std::shared_ptr<Particle>> _particles;
};

class Grid {
public:
    Grid(int x, int y, int z, float spacing);
    void simulate(float xi, float delta_t, glm::vec3 gravity, float theta_c, float theta_s, Plane &plane);
    void createParticles(int particle_num, float radius);
    ~Grid();

    bool _first_loop = true;
    float _spacing = 1;
    int _x_coord, _y_coord, _z_coord = 0;
    float _x_physical, _y_physical, _z_physical = 0;
    glm::vec3 _origin;
    Relate_Math math_factory;
    std::vector<std::vector<std::vector<std::unique_ptr<Node>>>> _nodes;
    std::vector<std::shared_ptr<Particle>> _particles;

private:
    void set_mass_and_velocities(); // Objective 1;
    void volume_init(); // Objective 2;
    void set_force(float xi); // Objective 3;
    void update_velocities(float delta_t); // Objective 4;
    void set_node_collision(float delta_t, Plane &plane); // Objective 5;
    void update_deform_gradient(float theta_c, float theta_s, float delta_t); // Objective 6;
    void update_flip_and_pic(); // Objective 7;
    void set_particle_collision(float delta_t, Plane &plane); // Objective 5 #2;
    void update_particle_pos(float delta_t); // Objective 8;
    void reset();

    // helpers:
    std::pair<int, int> get_bound(float x, int y) const;
};


#endif //SNOW_SIMULATION_GRID_H
