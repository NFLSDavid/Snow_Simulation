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

struct Node {
    glm::vec3 _pos;
    glm::vec3 _velocity;
    glm::vec3 _force;
    float _mass = 0;
    std::vector<Particle *> _particles;
};

class Grid {
public:
    float _spacing = 1;
    int _x = 0;
    int _y = 0;
    int _z = 0;
    Relate_Math math_factory;
    std::vector<std::vector<std::vector<Node *>>> _nodes;
    Grid(int x, int y, int z, float h);
    ~Grid();
    void set_mass_and_velocities();
    void volume_init();
    void set_force(float xi);
private:
    std::pair<int, int> get_bound(float x, int y) const;
};


#endif //SNOW_SIMULATION_GRID_H
