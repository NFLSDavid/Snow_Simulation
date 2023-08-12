//
// Created by 方文宇 on 2023/7/31.
//

#include "include/Grid.h"

Grid::Grid(int x, int y, int z, float spacing): _x_coord{x}, _y_coord{y}, _z_coord{z}, _spacing{spacing} {
    _x_physical = _spacing * _x_coord;
    _y_physical = _spacing * _y_coord;
    _z_physical = _spacing * _z_coord;
    _origin = glm::vec3(_x_physical / 2, _y_physical / 2, _z_physical / 2);
    for (int i = 0; i < _x_coord; ++i) {
        _nodes.push_back(std::vector<std::vector<std::unique_ptr<Node>>>());
        for (int j = 0; j < _y_coord; ++j) {
            _nodes.at(i).push_back(std::vector<std::unique_ptr<Node>>());
            for (int k = 0; k < _z_coord; ++k) {
                _nodes.at(i).at(j).push_back(std::make_unique<Node>());
            }
        }
    }

    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                _nodes.at(i).at(j).at(k)->_pos = glm::vec3{i, j, k};
                _nodes.at(i).at(j).at(k)->_mass = 0;
            }
        }
    }
}

Grid::~Grid() = default;

std::pair<int, int> Grid::get_bound(float x, int y) const {
    int high = std::min((int)floor(x / _spacing + 2) + 1, y);
    int low = std::max((int)ceil(x / _spacing - 2), 0);
    return std::pair<int, int>{low, high};
}

// Objective 1: initialize every Node mass and velocity by using weight_func;
void Grid::set_mass_and_velocities() {
    for (const auto &particle: _particles) {
        // Calculate the mass and velocity by the
        for (int i = particle->x_bound.first; i < particle->x_bound.second; ++ i) {
            for (int j = particle->y_bound.first; j < particle->y_bound.second; ++ j) {
                for (int k = particle->z_bound.first; k < particle->z_bound.second; ++ k) {
                    glm::vec3 tmp_p {i, j, k};
                    float tmp_weight = Relate_Math::weight_func(particle->_pos, tmp_p, _spacing);

                    // std::cerr << "tmp_weight: " << tmp_weight << std::endl;

                    _nodes.at(i).at(j).at(k)->_mass += tmp_weight * particle->_mass;
                    _nodes.at(i).at(j).at(k)->_velocity += tmp_weight * particle->_mass * particle->_velocity;
                }
            }
        }
    }

    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                if (_nodes.at(i).at(j).at(k)->_mass > 0) {
                    _nodes.at(i).at(j).at(k)->_velocity /= _nodes.at(i).at(j).at(k)->_mass;
                } else {
                    // std::cerr << "Error: Node mass is below zero!" << std::endl;
                }
            }
        }
    }
}

// Objective 2: Calculate the Node volume and density
void Grid::volume_init() {
    for (const auto &particle: _particles) {
        float density = 0;
        // Calculate the mass and velocity by the
        for (int i = particle->x_bound.first; i < particle->x_bound.second; ++ i) {
            for (int j = particle->y_bound.first; j < particle->y_bound.second; ++ j) {
                for (int k = particle->z_bound.first; k < particle->z_bound.second; ++ k) {
                    glm::vec3 tmp_p {i, j, k};
                    float tmp_weight = Relate_Math::weight_func(particle->_pos, tmp_p, _spacing);
                    density += tmp_weight * _nodes.at(i).at(j).at(k)->_mass;
                }
            }
        }
        particle->_volume = particle->_mass * _spacing * _spacing * _spacing / density;
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle : _nodes.at(i).at(j).at(k)->_particles) {
                    float density = 0;
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++ tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++ tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                float tmp_weight = Relate_Math::weight_func(particle->_pos, tmp_p, _spacing);
                                density += tmp_weight * _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_mass;
                            }
                        }
                    }
                    particle->_volume = particle->_mass * _spacing * _spacing * _spacing / density;
                    std::cout << particle->_volume << std::endl;
                }
            }
        }
    }
    */
}


// Objective 3: Calculate the Node forces:
void Grid::set_force(float xi) {
    for (const auto &particle: _particles) {
        glm::mat3 sigma_p = math_factory.get_sigma(xi, particle);
        glm::mat3 tmp_no_weight = particle->_volume * sigma_p;
        for (int i = particle->x_bound.first; i < particle->x_bound.second; ++ i) {
            for (int j = particle->y_bound.first; j < particle->y_bound.second; ++j) {
                for (int k = particle->z_bound.first; k < particle->z_bound.second; ++k) {
                    glm::vec3 tmp_p{i, j, k};
                    glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                    _nodes.at(i).at(j).at(k)->_force -= tmp_weight * tmp_no_weight;
                }
            }
        }
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle : _nodes.at(i).at(j).at(k)->_particles) {
                    glm::mat3 sigma_p = math_factory.get_sigma(xi, particle);
                    glm::mat3 tmp_no_weight = particle->_volume * glm::determinant(particle->_deform_gradient_P) * sigma_p;
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++ tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++ tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_force -= tmp_weight * tmp_no_weight;
                            }
                        }
                    }
                }
            }
        }
    }
    */
}


// Objective 4: Update the velocities:
void Grid::update_velocities(float delta_t, glm::vec3 gravity) {
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                if ( _nodes.at(i).at(j).at(k)->_mass > 0) {
                    _nodes.at(i).at(j).at(k)->_updated_velocity +=
                            delta_t * (_nodes.at(i).at(j).at(k)->_force / _nodes.at(i).at(j).at(k)->_mass + gravity);
                }
            }
        }
    }
}

// Objective 5: Consider collision #1.
void Grid::set_node_collision(float delta_t, Plane &plane) {
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                _nodes.at(i).at(j).at(k)->_updated_velocity = plane.collide(glm::vec3{i, j, k} * _spacing, _nodes.at(i).at(j).at(k)->_updated_velocity, delta_t);
            }
        }
    }
}

// Objective 6: update the deform_gradient_P and deform_gradient_E
void Grid::update_deform_gradient(float theta_c, float theta_s, float delta_t) {
    for (const auto &particle: _particles) {
        glm::mat3 gradient_v_p_next = glm::mat3{0};
        for (int i = particle->x_bound.first; i < particle->x_bound.second; ++ i) {
            for (int j = particle->y_bound.first; j < particle->y_bound.second; ++j) {
                for (int k = particle->z_bound.first; k < particle->z_bound.second; ++k) {
                    glm::vec3 tmp_p{i, j, k};
                    glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                    gradient_v_p_next += glm::outerProduct(_nodes.at(i).at(j).at(k)->_updated_velocity, tmp_weight);
                }
            }
        }
        glm::mat3 deform_gradient_E_p_hat_next = (glm::mat3{1} + delta_t * gradient_v_p_next) * particle->_deform_gradient_E;
        glm::mat3 U_p, V_p, Sigma_p;
        Relate_Math::get_svd(deform_gradient_E_p_hat_next, U_p, Sigma_p, V_p, theta_c, theta_s);
        particle->_deform_gradient_E = U_p * Sigma_p * glm::transpose(V_p);
        particle->_deform_gradient_P = V_p * glm::inverse(Sigma_p) * glm::transpose(U_p) * deform_gradient_E_p_hat_next * particle->_deform_gradient_P;
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {

                    glm::mat3 gradient_v_p_next = glm::mat3{0};
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                gradient_v_p_next += glm::outerProduct(_nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_updated_velocity, tmp_weight);
                            }
                        }
                    }

                    glm::mat3 deform_gradient_E_p_hat_next = (glm::mat3{1} + delta_t * gradient_v_p_next) * particle->_deform_gradient_E;
                    glm::mat3 U_p, V_p, Sigma_p;
                    Relate_Math::get_svd(deform_gradient_E_p_hat_next, U_p, Sigma_p, V_p, theta_c, theta_s);
                    particle->_deform_gradient_E = U_p * Sigma_p * glm::transpose(V_p);
                    particle->_deform_gradient_P = V_p * glm::inverse(Sigma_p) * glm::transpose(V_p) * deform_gradient_E_p_hat_next * particle->_deform_gradient_P;
                }
            }
        }
    }
    */
}


// Objective 7: Update FLIP and PIC velocity values
void Grid::update_flip_and_pic() {
    for (const auto &particle: _particles) {
        glm::vec3 v_next_flip = particle->_velocity;
        glm::vec3 v_next_pic = glm::vec3 {0};
        for (int i = particle->x_bound.first; i < particle->x_bound.second; ++ i) {
            for (int j = particle->y_bound.first; j < particle->y_bound.second; ++j) {
                for (int k = particle->z_bound.first; k < particle->z_bound.second; ++k) {
                    glm::vec3 tmp_p{i, j, k};
                    glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                    v_next_pic += _nodes.at(i).at(j).at(k)->_updated_velocity * tmp_weight;
                    v_next_flip += (_nodes.at(i).at(j).at(k)->_updated_velocity - _nodes.at(i).at(j).at(k)->_velocity)* tmp_weight;
                }
            }
        }
        particle->_velocity = (1 - alpha) * v_next_pic + alpha * v_next_flip;
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {
                    glm::vec3 v_next_flip = particle->_velocity;
                    glm::vec3 v_next_pic = glm::vec3 {0};
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                v_next_pic += _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_updated_velocity * tmp_weight;
                                v_next_flip += (_nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_updated_velocity - _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_velocity)* tmp_weight;
                            }
                        }
                    }
                    particle->_velocity = (1 - alpha) * v_next_pic + alpha * v_next_flip;
                }
            }
        }
    }
    */
}

// Objective 5: Consider collision #2.
void Grid::set_particle_collision(float delta_t, Plane &plane) {
    for (const auto &particle: _particles) {
        particle->_velocity = plane.collide(particle->_pos, particle->_velocity, delta_t);
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {
                    particle->_velocity = plane.collide(glm::vec3{i, j, k} * _spacing, particle->_velocity, delta_t);
                }
            }
        }
    }
     */
}

// Objective 8: Update the location and make the algorithm work totally;
void Grid::update_particle_pos(float delta_t) {
    for (const auto &particle: _particles) {
        particle->_pos += particle->_velocity * delta_t;
    }
    /*
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {
                    particle->_pos += particle->_velocity * delta_t;
                }
            }
        }
    }
    */
}

void Grid::reset() {
    for (int i = 0; i < _x_coord; ++i) {
        for (int j = 0; j < _y_coord; ++j) {
            for (int k = 0; k < _z_coord; ++k) {
                _nodes.at(i).at(j).at(k)->_particles.clear();
                _nodes.at(i).at(j).at(k)->_mass = 0;
                _nodes.at(i).at(j).at(k)->_velocity = glm::vec3{0};
                _nodes.at(i).at(j).at(k)->_force = glm::vec3{0};
                _nodes.at(i).at(j).at(k)->_updated_velocity = glm::vec3{0};
            }
        }
    }

    for (const std::shared_ptr<Particle>& particle : _particles) {
        particle->_pos = glm::max(glm::vec3(0.0), glm::min(particle->_pos,glm::vec3(_x_physical, _y_physical, _z_physical) - glm::vec3(1e-5)));
        glm::ivec3 node_pos = glm::floor(particle->_pos / _spacing);

        // un tested;
        particle->x_bound = get_bound(particle->_pos.x, _x_coord);
        particle->y_bound = get_bound(particle->_pos.y, _y_coord);
        particle->z_bound = get_bound(particle->_pos.z, _z_coord);
        // un tested;

        /*
        for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
            for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++tmp_j) {
                for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++tmp_k) {
                    Node *node = _nodes[tmp_i][tmp_j][tmp_k];
                    if (node == NULL) {
                        node = new Node();
                        node->_pos = glm::ivec3(tmp_i, tmp_j, tmp_k);
                        _nodes[tmp_i][tmp_j][tmp_k] = node;
                    }
                    _none_empty_nodes.insert(node);
                }
            }
        }
         */
        _nodes.at(node_pos.x).at(node_pos.y).at(node_pos.z)->_particles.push_back(particle);
    }
}

void Grid::createParticles(int particle_num, float radius) {
    glm::vec3 tmp_origin {_origin.x, _origin.y * 1.15, _origin.z};
    for (int i = 0; i < particle_num; ++i) {
        std::shared_ptr<Particle> particle = std::make_unique<Particle>(PARTICLE_MASS, tmp_origin + glm::ballRand(radius));
        _particles.push_back(particle);
    }
    reset();
}

void
Grid::simulate(float xi, float delta_t, glm::vec3 gravity, float theta_c, float theta_s, Plane &plane, bool &first_round) {
    reset();
    set_mass_and_velocities();
    if (first_round) {
        volume_init();
        first_round = false;
    }
    set_force(xi);
    update_velocities(delta_t, gravity);
    set_node_collision(delta_t, plane);
    update_deform_gradient(theta_c, theta_s, delta_t);
    update_flip_and_pic();
    set_particle_collision(delta_t, plane);
    update_particle_pos(delta_t);
}