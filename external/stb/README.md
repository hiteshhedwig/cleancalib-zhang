# stb headers

`clean-calib` uses two single-file public-domain headers from
[github.com/nothings/stb](https://github.com/nothings/stb).

Place these two files in this directory:

- `stb_image.h`        — image loader (PNG, JPEG, BMP, …)
- `stb_image_write.h`  — image writer (PNG, JPEG, BMP, TGA)

Direct download URLs:

```
https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
```

Or:

```bash
cd external/stb
curl -O https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
curl -O https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
```

Until both files are present, the build will configure with a warning,
and any image I/O command (`image-info`, `image-copy`) will fail at
runtime with a clear error message.
