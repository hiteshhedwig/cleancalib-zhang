What Milestone 5 should accomplish :

3D world point
-> transform into camera coordinates using Pose
-> divide by Z to get normalized image coordinates
-> apply camera intrinsics
-> get pixel coordinates

creating new files - 

include/clean_calib/calib/projection.h
src/calib/projection.cpp
tests/test_projection.cpp

need three conceptual functions :
1. transform_world_to_camera(...)
2. project_camera_to_normalized(...)
3. project_normalized_to_pixel(...)


## Milestone 5 — Pinhole projection without distortion
- [x] Add projection module
- [x] Implement world-to-camera transform
- [x] Implement camera-to-normalized perspective divide
- [x] Reject points with Z <= 0
- [x] Implement normalized-to-pixel using fx, fy, cx, cy, skew
- [x] Implement full pinhole project function
- [x] Tests: identity pose
- [x] Tests: translated pose
- [x] Tests: perspective divide
- [x] Tests: intrinsics with zero skew
- [x] Tests: intrinsics with nonzero skew
- [x] Tests: reject point behind camera
