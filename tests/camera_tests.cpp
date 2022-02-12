#include <gtest/gtest.h>

#include "camera.hpp"
#include "se3.h"

#include <iostream>

namespace rt = rigidTransform;

TEST(DUMMY, dummy) {
    EXPECT_TRUE(true);
}

TEST(Projection, ProjectFeature) {
    projection::Camera camera;
    rt::SE3<double> T_world_from_camera =
        rt::SE3<double>::Identity();
    Eigen::Vector3d pt;
    pt << 0.3, 0.4, 3.0;

    Eigen::Vector2i pix = camera.project(pt);
    Eigen::Vector2i true_pix(790, 712);

    EXPECT_TRUE(pix.isApprox(true_pix));
}

TEST(Projection, ProjectFeatureNotAtOrigin) {
    projection::Camera camera;
    Eigen::Vector3d pos(-1, 0, 0.5);
    rt::SE3<double> T_world_from_camera =
        rt::SE3<double>(0, 0.9, 0.0, pos);
    Eigen::Vector3d pt;
    pt << -.1, -.1, 1.0;

    Eigen::Vector2i pix = camera.project(pt, T_world_from_camera);
    Eigen::Vector2i true_pix(887, 364);

    EXPECT_TRUE(pix.isApprox(true_pix));
}
