#include <Eigen/Core>

#include <iostream>

int main() {
    const Eigen::Vector3d world_point(1.0, 2.0, 4.0);
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d translation(0.0, 0.0, 0.0);

    std::cout << "Milestone 5 pinhole practice\n"
              << "World point:\n" << world_point << "\n"
              << "Start with X_camera = R * X_world + t and print every stage.\n";
    (void)rotation;
    (void)translation;
    return 0;
}
