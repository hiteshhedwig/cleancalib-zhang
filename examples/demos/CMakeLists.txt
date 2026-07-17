function(add_clean_calib_demo target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE clean_calib)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(${target} PRIVATE
    CLEAN_CALIB_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
endfunction()

add_clean_calib_demo(demo_m01_image_io m01_image_io.cpp)
add_clean_calib_demo(demo_m02_grayscale m02_grayscale.cpp)
add_clean_calib_demo(demo_m03_geometry m03_geometry.cpp)
add_clean_calib_demo(demo_m04_checkerboard m04_checkerboard.cpp)
add_clean_calib_demo(demo_m05_projection m05_projection.cpp)
add_clean_calib_demo(demo_m06_distortion m06_distortion.cpp)
add_clean_calib_demo(demo_m07_synthetic_views m07_synthetic_views.cpp)

add_custom_target(clean_calib_demos
  DEPENDS
    demo_m01_image_io
    demo_m02_grayscale
    demo_m03_geometry
    demo_m04_checkerboard
    demo_m05_projection
    demo_m06_distortion
    demo_m07_synthetic_views
)

add_custom_target(run_clean_calib_demos
  COMMAND $<TARGET_FILE:demo_m01_image_io>
  COMMAND $<TARGET_FILE:demo_m02_grayscale>
  COMMAND $<TARGET_FILE:demo_m03_geometry>
  COMMAND $<TARGET_FILE:demo_m04_checkerboard>
  COMMAND $<TARGET_FILE:demo_m05_projection>
  COMMAND $<TARGET_FILE:demo_m06_distortion>
  COMMAND $<TARGET_FILE:demo_m07_synthetic_views>
  DEPENDS clean_calib_demos
  USES_TERMINAL
)
