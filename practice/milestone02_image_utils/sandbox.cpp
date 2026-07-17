#include <array>
#include <iostream>

int main() {
    const std::array<unsigned char, 12> rgb = {
        255, 0, 0,
        0, 255, 0,
        0, 0, 255,
        255, 255, 255
    };

    std::cout << "Milestone 2 image-utility practice\n"
              << "The 2 x 2 RGB buffer contains " << rgb.size() << " bytes.\n"
              << "Implement indexing and grayscale conversion here.\n";
    return 0;
}
