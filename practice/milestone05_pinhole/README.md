# Milestone 5 practice — pinhole projection

Rebuild the forward projection as one transparent toy program before reviewing
the production functions.

## Exercises

1. Hard-code a world point, `R`, `t`, and intrinsic matrix `K`.
2. Print `X_c = RX_w+t`.
3. Reject `Z_c <= 0` before division.
4. Print normalized `(x,y)=(X_c/Z_c,Y_c/Z_c)`.
5. Apply `fx,fy,cx,cy,skew` explicitly and print `(u,v)`.
6. Calculate the same result on paper.
7. Try identity, translation, rotation, non-unit depth, nonzero skew, and a
   behind-camera point one at a time.
8. Finally verify the staged result against homogeneous `K[R|t]X`.

Do not include `projection.h`. The point is to reconstruct the equation.

## Exit criteria

You can identify the frame and units at every intermediate line and explain why
image bounds are not part of geometric projection.
