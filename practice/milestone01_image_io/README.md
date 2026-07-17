# Milestone 1 practice — image representation and I/O

The goal is to recover pixel-layout and ownership intuition without using the
production `Image` or image-I/O wrapper.

## Exercises

1. Define a tiny local image struct with width, height, channels, and a byte
   vector.
2. Construct a `3 × 2` RGB image with distinct values at every pixel.
3. Print each pixel and its byte offset.
4. Write it as a binary PPM file using `std::ofstream`.
5. Validate that the vector contains exactly `width*height*channels` bytes.
6. Deliberately shorten the vector and make validation reject it.
7. Explain how an stb-returned pointer would be copied and freed, but do not
   reimplement an image decoder.

## Exit criteria

You can derive the offset formula, explain interleaved storage and row stride,
and state who owns every buffer. Then compare with `Image` and `image_io.cpp`.
