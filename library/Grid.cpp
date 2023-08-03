//
// Created by 方文宇 on 2023/7/31.
//

#include "include/Grid.h"

Grid::Grid(int x, int y, int z, float h): _x{x}, _y{y}, _z{z}, _spacing{h} {
    // initialize the grid and each node here;
    _nodes = std::vector<std::vector<std::vector<Node *>>>(x);
    for (int i = 0; i < x; ++i) {
        _nodes.at(i) = std::vector<std::vector<Node *>>(y);
        for (int j = 0; j < y; ++j) {
            _nodes.at(i).at(j) = std::vector<Node *>(z);
            for (int k = 0; k < z; ++k) {
                _nodes.at(i).at(j).at(k) = new Node();
                _nodes.at(i).at(j).at(k)->_pos = glm::vec3(i, j, k);
            }
        }
    }
}

Grid::~Grid() {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                delete _nodes.at(i).at(j).at(k);
            }
        }
    }
}

std::pair<int, int> Grid::get_bound(float x, int y) const {
    int high = std::min((int)floor(x / _spacing + 2) + 1, y);
    int low = std::max((int)ceil(x / _spacing - 2), 0);
    return std::pair<int, int>{low, high};
}

// Objective 1: initialize every Node mass and velocity by using weight_func;
void Grid::set_mass_and_velocities() {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                for (auto *particle : _nodes.at(i).at(j).at(k)->_particles) {
                    // define the bound of grids and particles that will be affected
                    particle->x_bound = get_bound(particle->_pos.x, _x);
                    particle->y_bound = get_bound(particle->_pos.y, _y);
                    particle->z_bound = get_bound(particle->_pos.z, _z);

                    // Calculate the mass and velocity by the
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++ tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++ tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                float tmp_weight = Relate_Math::weight_func(particle->_pos, tmp_p, _spacing);
                                _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_mass += tmp_weight * particle->_mass;
                                _nodes.at(tmp_i).at(tmp_j).at(tmp_k)->_velocity += tmp_weight * particle->_mass * particle->_velocity;
                            }
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                if (_nodes.at(i).at(j).at(k)->_mass > 0) {
                    _nodes.at(i).at(j).at(k)->_velocity /= _nodes.at(i).at(j).at(k)->_mass;
                }
            }
        }
    }
}

// Objective 2: Calculate the Node volume and density
void Grid::volume_init() {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                for (auto *particle : _nodes.at(i).at(j).at(k)->_particles) {
                    float density = 0;
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++ tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++ tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                float tmp_weight = Relate_Math::weight_func(particle->_pos, tmp_p, _spacing);
                                density += tmp_weight * _nodes.at(i).at(j).at(k)->_mass;
                            }
                        }
                    }
                    particle->_volume = particle->_mass * _spacing * _spacing * _spacing / density;
                }
            }
        }
    }
}


// Objective 3: Calculate the Node forces:
void Grid::set_force(float xi) {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                for (auto *particle : _nodes.at(i).at(j).at(k)->_particles) {
                    glm::mat3 sigma_p = math_factory.get_sigma(xi, particle);
                    glm::mat3 tmp_no_weight = particle->_volume * glm::determinant(particle->_deform_gradient_P) * sigma_p;
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++ tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++ tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                _nodes.at(i).at(j).at(k)->_force -= tmp_weight * tmp_no_weight;
                            }
                        }
                    }
                }
            }
        }
    }
}


// Objective 4: Update the velocities:
void Grid::update_velocities(float delta_t, glm::vec3 gravity) {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                if ( _nodes.at(i).at(j).at(k)->_mass > 0) {
                    _nodes.at(i).at(j).at(k)->_updated_velocity +=
                            delta_t * (_nodes.at(i).at(j).at(k)->_force + gravity * _nodes.at(i).at(j).at(k)->_mass) / _nodes.at(i).at(j).at(k)->_mass;
                }
            }
        }
    }
}

// Objective 5: Consider collision with a pane.

// Objective 6: update the deform_gradient_P and deform_gradient_E
void Grid::update_deform_gradient(float theta_c, float theta_s, float delta_t) {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {

                    glm::mat3 gradient_v_p_next = glm::mat3{0};
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                gradient_v_p_next += glm::outerProduct(_nodes.at(i).at(j).at(k)->_updated_velocity, tmp_weight);
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
}

void Grid::update_flip_and_pic() {
    for (int i = 0; i < _x; ++i) {
        for (int j = 0; j < _y; ++j) {
            for (int k = 0; k < _z; ++k) {
                for (auto *particle: _nodes.at(i).at(j).at(k)->_particles) {
                    glm::vec3 v_next_flip = particle->_velocity;
                    glm::vec3 v_next_pic = glm::vec3 {0};
                    for (int tmp_i = particle->x_bound.first; tmp_i < particle->x_bound.second; ++ tmp_i) {
                        for (int tmp_j = particle->y_bound.first; tmp_j < particle->y_bound.second; ++tmp_j) {
                            for (int tmp_k = particle->z_bound.first; tmp_k < particle->z_bound.second; ++tmp_k) {
                                glm::vec3 tmp_p {tmp_i, tmp_j, tmp_k};
                                glm::vec3 tmp_weight = Relate_Math::weight_func_gradient(particle->_pos, tmp_p, _spacing);
                                v_next_pic += _nodes.at(i).at(j).at(k)->_updated_velocity * tmp_weight;
                                v_next_flip += (_nodes.at(i).at(j).at(k)->_updated_velocity - _nodes.at(i).at(j).at(k)->_velocity)* tmp_weight;
                            }
                        }
                    }
                    particle->_velocity = (1 - alpha) * v_next_pic + alpha * v_next_flip;
                }
            }
        }
    }
}
