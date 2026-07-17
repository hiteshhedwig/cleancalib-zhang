#include <Eigen/Core>

#include <iostream>
#include <vector>

int main() {
    std::vector<Eigen::Vector3d> object_points;

    std::cout << "Milestone 7 synthetic-data practice\n"
              << "Generate a 2 x 3 board, define two poses, and project each point.\n"
              << "Current object-point count: " << object_points.size() << "\n";
    return 0;
}
