# Milestone 2 — Basic image utilities

## Purpose

Establish correct pixel indexing and grayscale conversion before implementing
gradients and checkerboard detection. These routines are simple enough that
layout bugs should be eliminated here, not discovered in a Harris response.

## Understand before coding

- Image coordinates: origin at top-left, `x` right, `y` down.
- Interleaved channel indexing and valid coordinate ranges.
- Luma versus a naive arithmetic mean of RGB.
- Alpha as coverage, not another color component.
- The difference between rejecting, clamping, padding, and skipping a border.

The project currently uses the common luma approximation:

```text
Y = 0.299 R + 0.587 G + 0.114 B
```

This is adequate for the educational detector, though it is not a complete
color-management pipeline.

## Practice before implementation

1. Compute the offset of channel 2 at pixel `(3, 4)` for a width-10 RGB image.
2. Convert pure red, green, blue, black, and white by hand.
3. Compare replicate, reflect, zero, and skip-border behavior for a `3 × 3`
   convolution at the top-left pixel.
4. Write a tiny ASCII PGM/PPM image by hand and open it in an image viewer.

## Suggested implementation stages

1. Implement and test `in_bounds`.
2. Implement clamping with a documented empty-image behavior.
3. Implement a single pixel-offset helper.
4. Preserve a valid one-channel image exactly.
5. Convert RGB and RGBA, ignoring alpha deliberately.
6. Reject unsupported or internally inconsistent images.
7. Add a simple PGM/PPM debug writer if later detector work needs it.

## Tests

- All four image boundaries and negative coordinates.
- Known offsets across rows and channels.
- Exact primary-color grayscale values.
- RGBA result independent of alpha.
- Empty, two-channel, and undersized buffers follow the documented policy.
- Grayscale output has exactly `width * height` bytes.

## Common mistakes

- Swapping `x` and `y`.
- Using `height` where the row stride needs `width`.
- Performing weighted arithmetic in `unsigned char`.
- Reading RGB bytes from a malformed or two-channel image.
- Clamping everywhere and thereby duplicating border pixels unintentionally.

## Resources

- stb's [image-loading documentation](https://github.com/nothings/stb/blob/master/stb_image.h)
  for returned component layouts.
- Netpbm, [PGM format specification](https://netpbm.sourceforge.net/doc/pgm.html)
  and [PPM format specification](https://netpbm.sourceforge.net/doc/ppm.html):
  useful for a transparent debug writer.
- ITU-R, [BT.601 recommendation](https://www.itu.int/rec/R-REC-BT.601/):
  background for the familiar luma coefficients; do not turn this milestone
  into a color-science project.

## Completion questions

- Why should grayscale conversion return a one-channel image?
- Why is clamping a policy decision rather than universally correct behavior?
- Which invariant must be checked before the first pixel read?
