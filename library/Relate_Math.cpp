//
// Created by 方文宇 on 2023/8/2.
//

#include "include/Relate_Math.h"
#include <iostream>

float Relate_Math::N_x(float x) {
    float abs_x = fabs(x);
    if (abs_x < 1) {
        float ans = 0.5f * abs_x * abs_x * abs_x - x * x + 2.f / 3.f;
        // std::cerr << x << " " << (2 / 3) - x * x << " " << ans << std::endl;
        return ans;
        //return 0.5f * abs_x * abs_x * abs_x - x * x + 2 / 3;
    } else if (abs_x < 2) {
        return -1.f / 6.f * abs_x * abs_x * abs_x + x * x - 2.f * abs_x + 4.f / 3.f;
    } else {
         return 0;
    }
}

float Relate_Math::N_x_derivative(float x) {
    float abs_x = fabs(x);
    if (abs_x < 1) {
        return 1.5f * abs_x * abs_x - 2 * x;
    } else if (abs_x < 2) {
        return - 0.5f * x * x + 2 * abs_x - 2;
    } else {
        return 0;
    }
}

float Relate_Math::weight_func(glm::vec3 pos, glm::vec3 grid_index, float spacing) {
    glm::vec3 scaled = pos / spacing - grid_index;
    float sx = N_x(scaled.x);
    float sy = N_x(scaled.y);
    float sz = N_x(scaled.z);
    float ans = sx * sy * sz;
    if (ans < 0) {
        // std::cerr << sx << ' ' << sy << ' ' << sz << ' ' << ans << std::endl;
        // std::cerr << scaled.x << ' ' << scaled.y << ' ' << scaled.z << ' ' << std::endl;
        // std::cerr << sx << ' ' << sy << ' ' << sz << ' ' << std::endl;
        // std::cerr << std::endl;
    }

    return ans;
    //return N_x(scaled.x) * N_x(scaled.y) * N_x(scaled.z);
}

glm::vec3 Relate_Math::weight_func_gradient(glm::vec3 pos, glm::vec3 grid_index, float spacing) {
    glm::vec3 scaled = (pos - spacing * grid_index) / spacing;
    float N_i_x = N_x_derivative(scaled.x) * N_x(scaled.y) * N_x(scaled.z);
    float N_i_y = N_x_derivative(scaled.y) * N_x(scaled.x) * N_x(scaled.z);
    float N_i_z = N_x_derivative(scaled.z) * N_x(scaled.y) * N_x(scaled.x);
    return glm::vec3 {N_i_x, N_i_y, N_i_z};
}

float Relate_Math::get_mu(float xi, float J_P) const {
    return mu_0 * exp(xi * (1 - J_P));
}

float Relate_Math::get_lambda(float xi, float J_P) const {
    return lambda_0 * exp(xi * (1 - J_P));
}

glm::mat3 Relate_Math::eigen_to_glm(const Eigen::Matrix3f &eigen_mat) {
    glm::mat3 glm_mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm_mat[i][j] = eigen_mat(i, j);
        }
    }
    return glm_mat;
}

Eigen::Matrix3f Relate_Math::glm_to_eigen(const glm::mat3 &glm_mat) {
    Eigen::Matrix3f eigen_mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_mat(i, j) = glm_mat[i][j];
        }
    }
    return eigen_mat;
}

glm::mat3 Relate_Math::polar_R(glm::mat3 deform_gradient_E) {
    Eigen::Matrix3f F_eigen = glm_to_eigen(deform_gradient_E);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(F_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R_E = svd.matrixU() * svd.matrixV().adjoint();
    return eigen_to_glm(R_E);
}

glm::mat3 Relate_Math::get_sigma(float xi, const std::shared_ptr<Particle>& particle) const {
    float J_P = determinant(particle->_deform_gradient_P);
    float J_E = determinant(particle->_deform_gradient_E);
    glm::mat3 R_E_p = polar_R(particle->_deform_gradient_E);
    return 2 * get_mu(xi, J_P) * (particle->_deform_gradient_E - R_E_p) * glm::transpose(particle->_deform_gradient_E) +
            get_lambda(xi, J_P) * (J_E - 1) * J_E * glm::mat3(1.0f);
}

void Relate_Math::get_svd(const glm::mat3 &matrix, glm::mat3 &U_p, glm::mat3 &Sigma_p, glm::mat3 &V_p, float theta_c, float theta_s) {
    Eigen::Matrix3f eigen_matrix = glm_to_eigen(matrix);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(eigen_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U_p = eigen_to_glm(svd.matrixU());
    V_p = eigen_to_glm(svd.matrixV());
    Eigen::Vector3f Sigma_p_hat_vec_eigen = svd.singularValues();
    glm::vec3 Sigma_p_hat_vec = glm::vec3 {Sigma_p_hat_vec_eigen(0), Sigma_p_hat_vec_eigen(1), Sigma_p_hat_vec_eigen(2)};
    glm::vec3 Sigma_p_vec = glm::clamp(Sigma_p_hat_vec, 1 - theta_c, 1 + theta_s);
    Sigma_p = glm::mat3(Sigma_p_vec[0], 0.0, 0.0,
                        0.0, Sigma_p_vec[1], 0.0,
                        0.0, 0.0, Sigma_p_vec[2]);
}

Relate_Math::Relate_Math() = default;
