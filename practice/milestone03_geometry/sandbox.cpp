#include <Eigen/Core>

#include <iostream>

int main() {
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d camera_center(1.0, 2.0, 3.0);

    std::cout << "Milestone 3 geometry practice\n"
              << "Initial rotation:\n" << rotation << "\n"
              << "Camera center:\n" << camera_center << "\n"
              << "Replace the identity with a real rotation and complete the README exercises.\n";
    return 0;
}
