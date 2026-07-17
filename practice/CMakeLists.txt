# Practice programs are intentionally independent of the clean_calib library.
# They may use Eigen, but should not call the production implementation of the
# algorithm being practiced.

function(add_clean_calib_practice target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE Eigen3::Eigen)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

add_clean_calib_practice(
  practice_m01_image_io
  milestone01_image_io/sandbox.cpp
)

add_clean_calib_practice(
  practice_m02_image_utils
  milestone02_image_utils/sandbox.cpp
)

add_clean_calib_practice(
  practice_m03_geometry
  milestone03_geometry/sandbox.cpp
)

add_clean_calib_practice(
  practice_m04_checkerboard
  milestone04_checkerboard/sandbox.cpp
)

add_clean_calib_practice(
  practice_m05_pinhole
  milestone05_pinhole/sandbox.cpp
)

add_clean_calib_practice(
  practice_m06_distortion
  milestone06_distortion/sandbox.cpp
)

add_clean_calib_practice(
  practice_m07_synthetic_data
  milestone07_synthetic_data/sandbox.cpp
)

add_clean_calib_practice(
  practice_m08_sandbox
  milestone08_homography/sandbox.cpp
)
