#ifndef PROJECTION_CAMERA_HPP_
#define PROJECTION_CAMERA_HPP_

#include <eigen3/Eigen/Dense>
// TODO: Find a way to use rigidTransform library
#include "se3.h"

namespace projection {

class Camera {
    using SE3d = SE3<double>;
    using Vector5d = Eigen::Matrix<double, 5, 1>;
 public:
    Camera();
    Camera(double f, double ph, double pw, double w, double h, double u,
           double v);
    Camera(double f, double ph, double pw, double w, double h, double u,
           double v, const Vector5d &dist);


    inline Eigen::Matrix3d K() const { return K_; }

    Eigen::Vector2i project(const Eigen::Ref<Eigen::Vector3d> &pt);
    Eigen::Vector2i project(const Eigen::Ref<Eigen::Vector3d> &pt,
                            const SE3d &T_world_from_camera);

 private:
    Eigen::Vector2d calculateDistortion(const Eigen::Vector2d &pix);

    double focal_length_;
    double pixel_width_;
    double pixel_height_;
    double width_;
    double height_;
    double center_x_;
    double center_y_;

    Eigen::Matrix3d K_;
    Vector5d dist_coeff_;
};

}  // namespace projection

#endif  // PROJECTION_CAMERA_HPP_
