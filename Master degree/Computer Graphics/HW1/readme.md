# Yocto/Raytrace: Tiny Raytracer

In this homework, you will learn the basics of image synthesis by
implementing a simple, naive, path tracer. In particular, you will
learn how to

- setup camera and image synthesis loops,
- usa ray-intersection queries,
- write simple shaders,
- write a naive path tracer with simple sampling.

## Framework

The code uses the library [**Yocto/GL**](https://github.com/xelatihy/yocto-gl),
that is included in this project in the directory `yocto`.
We suggest to consult the [**documentation**](https://github.com/xelatihy/yocto-gl)
for the library. Also, since the library is getting improved
during the duration of the course, we suggest that you **star it and watch it**
on Github, so that you can notified as improvements are made.
In particular, we will use

- **yocto_math.h**: collection of math functions
- **yocto_sampling.h**: collection of sampling routines
- **yocto_shading.h**: collection of shading functions
- **yocto_image.{h,cpp}**: image data structure and image loading and saving
- **yocto_commonio.h**: helpers for writing command line apps
- **yocto_gui.{h,cpp}**: helpers for writing simple GUIs

In order to compile the code, you have to install
[Xcode](https://apps.apple.com/it/app/xcode/id497799835?mt=12)
on OsX, [Visual Studio 2019](https://visualstudio.microsoft.com/it/vs/) on Windows,
or a modern version of gcc or clang on Linux,
together with the tools [cmake](www.cmake.org) and [ninja](https://ninja-build.org).
The script `scripts/build.sh` will perform a simple build on OsX.
As discussed in class, we prefer to use
[Visual Studio Code](https://code.visualstudio.com), with
[C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) and
[CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
extensions, that we have configured to use for this course.

You will write your code in the file `yocto_raytrace.cpp` for functions that
are declared in `yocto_raytrace.h`. Your renderer is called by `yscenetrace.cpp`
for a command-line interface and `ysceneitraces.cpp` that show a simple
user interface.

This repository also contains tests that are executed from the command line
as shown in `run.sh`. The rendered images are saved in the `out/` directory.
The results should match the ones in the directory `check/`. High resolution
images are computed by `run-highres.sh`.

## Functionality (26 points)

In this homework you will implement the following features:

- **Main Rendering Loop** in function `trace_samples()`:
  - implement the main rendering loop considering only 1 sample (the loop over
    samples in done in the apps)
  - from the slides this is like the progressive rendering loop but with only
    one sample
  - update the accumulation buffer, number of samples and final image for each
    pixel of the `state` object
  - use `get_trace_shader_func()` to get the shader from the options
  - implement both a simple loop over pixel and a parallel one, as shown in code
  - for each return value from the shader, clamp its color if above `params.clamp`
- **Color Shader** in function `trace_color()`:
  - implement a shader that check for intersection and returns the material color
  - use `intersect_scene_bvh()` for intersection
- **Normal Shader** in function `trace_normal()`:
  - implement a shader that check for intersection and returns the normal as a
    color, with a scale and offset of 0.5 each
  - implement `eval_normal()` for this
- **Texcoord Shader** in function `trace_texcoord()`:
  - implement a shader that check for intersection and returns the texture  
    coordinates as a color in the red-green channels; use `fmod()` to force them
    in the [0, 1] range
  - implement `eval_texcoord()` for this
- **Eyelight Shader** in function `trace_eyelight()`:
  - implement a simple shader that compute diffuse lighting from the camera
    center as in the slides
  - use `eval_normal()` for this
- **Raytrace Shader** in function `trace_raytrace()`:
  - implement a shader that simulates illumination for a variety of materials
    structured following the steps in the lecture notes
  - implement environment lookup in `eval_environment()`
  - get position, normal and texcoords; correct normals for lines
  - get material values by multiply material constants and textures, evaluated
    using `eval_texture()` that you have to implement
  - implement polished transmission, polished metals, rough metals,
    rough plastic, and matte shading in hte order described in the slides
  - you can use any function from Yocto/Shading such as `fresnel_schlick()`,
    `microfacet_distribution()` and `microfacet_shadowing()`

## Extra Credit (8 points)

- **Refraction** in function `trace_raytrace*()`:
  - implement refraction using `refract()` for the direction, 
  - `reflectivity_to_eta()` to get the index of refraction from reflectivity (0.04)
  - and remembering to invert the index of refraction when leaving a surface
- **WYOS**, write your own shader:
  - write additional shders and try them on some 3d models
  - example: [cell shading](https://roystan.net/articles/toon-shader.html)
  - example: [matcap shading 1](http://viclw17.github.io/2016/05/01/MatCap-Shader-Showcase/), 
               [matcap shading 2](https://github.com/hughsk/matcap)

## Submission

To submit the homework, you need to pack a ZIP file that contains the code
you write and the images it generates, i.e. the ZIP with only the
`yocto_raytrace/` and `out/` directories.
The file should be called `<cognome>_<nome>_<numero_di_matricola>.zip`,
i.e. `<lastname>_<firstname>_<studentid>.zip`, and you should exclude
all other directories. Send it on Google Classroom.
