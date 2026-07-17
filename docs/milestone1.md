# Milestone 1 — Image I/O basics

## Purpose

Load external image bytes into a small owning C++ type and save them again.
Calibration eventually begins with images, but the rest of the codebase should
not need to know about stb's allocation or C API.

## Understand before coding

- Row-major, interleaved pixel storage.
- Width, height, channel count, and row stride.
- The difference between RGB, RGBA, grayscale, and grayscale-plus-alpha.
- Ownership across a C API: allocation, copy, and release.
- Header-only libraries that require one implementation translation unit.
- Lossless PNG versus lossy JPEG round trips.

For interleaved pixels:

```text
offset(x, y, c) = (y * width + x) * channels + c
```

The expected byte count is `width * height * channels`; validate this before
passing an `Image` buffer to a writer.

## Practice before implementation

1. For a `3 × 2` RGB image, list the byte offsets of every red component.
2. Draw the bytes for two pixels `{red, green}` in RGB layout.
3. Explain who owns the pointer returned by `stbi_load` and when it is freed.
4. Predict why loading and re-saving JPEG will not preserve every byte.

## Suggested implementation stages

1. Define an owning `Image` with explicit layout documentation.
2. Wrap `stbi_load` and immediately copy into `std::vector<unsigned char>`.
3. Release stb's buffer on every successful path.
4. Return descriptive load errors through `Result<Image>`.
5. Select the save encoder from a case-insensitive extension.
6. Validate dimensions, channels, multiplication overflow, and byte count.
7. Add CLI metadata and copy commands only after the library API works.

## Tests

- Missing file produces a useful failure.
- Empty or inconsistent image cannot be saved.
- Unsupported extension fails.
- PNG round trip preserves dimensions, channels, and every pixel byte.
- JPEG round trip preserves dimensions but uses an error tolerance for pixels.
- Uppercase extensions behave deliberately.
- Decide explicitly whether two-channel images are supported or converted.

## Common mistakes

- Defining `STB_IMAGE_IMPLEMENTATION` in more than one translation unit.
- Forgetting `stbi_image_free`.
- Assuming stb can only return 1, 3, or 4 channels.
- Trusting `data.size()` without checking dimensions.
- Treating JPEG as lossless.
- Using signed `int` multiplication before converting to `std::size_t`.

## Resources

- stb, [official repository and usage explanation](https://github.com/nothings/stb).
- stb, [`stb_image.h` documentation](https://github.com/nothings/stb/blob/master/stb_image.h):
  read “Basic usage,” `desired_channels`, failure reporting, and ownership.
- stb, [`stb_image_write.h`](https://github.com/nothings/stb/blob/master/stb_image_write.h):
  inspect the signatures and component-count expectations for each encoder.
- C++, [`std::vector`](https://en.cppreference.com/w/cpp/container/vector):
  review ownership, contiguous storage, `size`, and `data`.

## Completion questions

- Why copy stb's allocation into `std::vector`?
- What is a row stride, and why does PNG writing ask for it?
- Which invariants make an `Image` safe to read?
