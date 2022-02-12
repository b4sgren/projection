#include "camera.hpp"

namespace projection {
Camera::Camera() : focal_length_{.015}, pixel_width_{1e-5},
                   pixel_height_{1e-5}, width_{1280}, height_{1024},
                   center_x_{640}, center_y_{512},
                   dist_coeff_{Vector5d::Zero()} {
    K_ << focal_length_/pixel_width_, 0, center_x_,
          0, focal_length_/pixel_height_, center_y_,
          0, 0, 1;
}


Camera::Camera(double f, double ph, double pw, double w, double h, double u,
               double v) : focal_length_{f}, pixel_width_{pw},
                           pixel_height_{ph}, width_{w}, height_{h},
                           center_x_{u}, center_y_{v},
                           dist_coeff_{Vector5d::Zero()} {
    K_ << focal_length_/pixel_width_, 0, center_x_,
          0, focal_length_/pixel_height_, center_y_,
          0, 0, 1;
}

Camera::Camera(double f, double ph, double pw, double w, double h, double u,
               double v, const Vector5d &dist) : focal_length_{f},
                                                 pixel_width_{pw},
                                                 pixel_height_{ph}, width_{w},
                                                 height_{h},
                                                 center_x_{u}, center_y_{v},
                                                 dist_coeff_{dist} {
    K_ << focal_length_/pixel_width_, 0, center_x_,
          0, focal_length_/pixel_height_, center_y_,
          0, 0, 1;
}

Eigen::Vector2i Camera::project(const Eigen::Ref<Eigen::Vector3d> &pt) {
    SE3d T_world_from_camera = SE3d::Identity();
    return project(pt, T_world_from_camera);
}

Eigen::Vector2i Camera::project(const Eigen::Ref<Eigen::Vector3d> &pt,
                                const SE3d &T_world_from_camera) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d temp = K_ * I * T_world_from_camera.transp<double>(pt);
    Eigen::Vector2d pix = (temp.head<2>() / temp(2));
    Eigen::Vector2d delta = calculateDistortion(pix);
    return (pix+delta).cast<int>();
}


Eigen::Vector2d Camera::calculateDistortion(const Eigen::Vector2d &pix) {
    Eigen::Vector2d delta{Eigen::Vector2d::Zero()};
    Eigen::Vector2d center{center_x_, center_y_};
    double r = (center - pix).norm();

    // Radial distortion
    delta = pix * (dist_coeff_(0)*pow(r,2) + dist_coeff_(1)*pow(r,4) +
                        dist_coeff_(2)*pow(r, 6));

    // Tangential
    delta(0) += 2*dist_coeff_(3)*pix.prod() +
                dist_coeff_(4)*(r*r + 2*pix(0)*pix(0));
    delta(1) += dist_coeff_(3)*(r*r + 2*pix(1)*pix(1)) +
                2*dist_coeff_(4)*pix.prod();

    return delta;
}

}  // namespace projection
