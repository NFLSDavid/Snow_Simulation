//
// Created by 方文宇 on 2023/8/2.
//

#ifndef SNOW_SIMULATION_RELATE_MATH_H
#define SNOW_SIMULATION_RELATE_MATH_H
#include "external/glm/glm/glm.hpp"
#include "include/Particle.h"
#include "external/eigen/3.4.0_1/include/eigen3/Eigen/SVD"

const float Initial_Young_Modulus = 140000;
const float Poisson_Ratio = 0.2;

class Relate_Math {
private:
    float lambda_0 = Poisson_Ratio * Initial_Young_Modulus / ((1 + Poisson_Ratio) * (1 - 2 * Poisson_Ratio));
    float mu_0 = Initial_Young_Modulus / (2 * (1 + Poisson_Ratio));
public:
    Relate_Math();
    static float N_x(float x);
    static float N_x_derivative(float x);
    float weight_func(glm::vec3 pos, glm::vec3 grid_index, float spacing);
    glm::vec3 weight_func_gradient(glm::vec3 pos, glm::vec3 grid_index, float spacing);

    float get_mu(float xi, float J_P);
    float get_lambda(float xi, float J_P);
    glm::mat3 eigen_to_glm(const Eigen::Matrix3f& eigen_mat);
    Eigen::Matrix3f glm_to_eigen(const glm::mat3& glm_mat);
    glm::mat3 polar_R(glm::mat3);
    glm::mat3 get_sigma(float xi, Particle* particle);
};


#endif //SNOW_SIMULATION_RELATE_MATH_H
