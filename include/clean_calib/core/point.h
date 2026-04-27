#pragma once

// -----------------------------------------------------------------------------
// Coordinate conventions used everywhere in clean-calib:
//
//   Image pixel coordinates:
//     - origin at the top-left of the image
//     - +x points to the right
//     - +y points DOWN
//     - (0, 0) is the centre of the top-left pixel
//
//   World coordinates:
//     - right-handed
//     - units are metres unless a function says otherwise
//
//   Camera coordinates:
//     - origin at the camera centre
//     - +x to the right (in the image)
//     - +y down (in the image)
//     - +z forward (out of the camera, into the scene)
// -----------------------------------------------------------------------------

namespace clean_calib {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

}  // namespace clean_calib
