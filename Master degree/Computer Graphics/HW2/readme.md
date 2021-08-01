# Yocto/Pathtrace: Tiny Path Tracer

In this homework, you will learn how to build a simple path tracer with enough
features to make it robust for many scenes. In particular, you will learn how to

- write camera with depth of field,
- write a complex material,
- write a naive path tracer,
- write a path tracer with multiple importance sampling.

## Framework

The code uses the library [Yocto/GL](https://github.com/xelatihy/yocto-gl),
that is included in this project in the directory `yocto`.
We suggest to consult the documentation for the library that you can find
at the beginning of the header files. Also, since the library is getting improved
during the duration of the course, se suggest that you star it and watch it
on Github, so that you can notified as improvements are made.

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

You will write your code in the file `yocto_pathtrace.cpp` for functions that
are declared in `yocto_pathtrace.h`. Your renderer is callsed by `ypathtrace.cpp`
for a command-line interface and `yipathtraces.cpp` that show a simple
user interface.

This repository also contains tests that are executed from the command line
as shown in `run.sh`. The rendered images are saved in the `out/` directory.
The results should match the ones in the directory `check/`.

## Functionality (26 points)

In this homework you will implement the following features:

- **Camera Sampling** in functions `sample_camera()` and `eval_camera()`:
  - implement camera sampling using `sample_disk()` for the lens
  - implement camera ray generation by simulating a thin lens camera
  - follow the slides to understand how to structure the code
- **Naive Path tracing** in function `trace_naive()`:
  - implement a naive path tracer using the product formulation
  - you should handle both delta and non-delta brdfs using `is_delta()`
    and the functions below
  - follow the slides to understand how to structure the code
  - you can use the functions `eval_position()`, `eval_shading_normal()`,
    `eval_emission()`, `eval_brdf()`, `eval_opacity()`
- **Brdf sampling** in function `eval_brdfcos()`, `sample_brdscos()`
  and `sample_brdfcos_pdf()`:
  - implement brdf evaluation and sampling in the above functions
  - the brdf is a sum of the following lobes stored in a brdf objects
    - diffuse lobe with weight `diffuse`
    - specular lobe with weight `specular`, ior `ior`,
      and roughness `roughness`
    - metal lobe with weight `metal`, complex ior `meta` and `metak`,
      and roughness `roughness`
    - transmission lobe with weight `transmission`, ior `ior`,
      and roughness `roughness`
  - you can use all the reflectance functions in Yocto/Shading including `eval_<lobe>()`
    `sample_<lobe>()`, and `sample_<lobe>_pdf()` with lobes
    `<func>_diffuse_reflection()`, `<func>_microfacet_reflection()`,
    `<func>_microfacet_transmission()`
  - `eval_brdfcos()` is just a sum of lobes, but remember to fold in the cosine
  - `sample_brdfcos()` picks a direction based on one of the lobes
  - `sample_brdfcos_pdf()` is the sum of the PDFs using weights `<lobe>_pdf`
    stored in `brdf`
  - follow the slides to understand how to structure the code
- **Delta handling** in function `eval_delta()`, `sample_delta()` and `sample_delta_pdf()`:
  - same as above with the corresponding functions
  - follow the slides to understand how to structure the code
- **Light sampling** in function `sample_lights()` and `sample_lights_pdf()`:
  - implement light sampling for both area lights and environment maps
  - lights and their CDFs are already implemented in `init_lights()`
  - follow the slides to understand how to structure the code
- **Path tracing** in function `trace_path()`:
  - implement a path tracer in the product formulation that uses MIS for
    illumination of the smooth BRDFs
  - the simplest way here is to get naive path tracing to work,
    then cut&paste that code and finally add light sampling using MIS
  - follow the slides to understand how to structure the code

To help out, we left example code in `trace_eyelight()`. You can also check out
Yocto/Trace that implements a similar path tracer; in this case though pay
attention to the various differences. In our opinion, it is probably easier to
follow the slides than to follow Yocto/Trace.

## Extra Credit (8 points)

Here we put options of things you could try to do.
You do not have to do them all, since points are capped to 8.
Choose then ones you want to do. They are all fun!

- **Refraction** in all BRDFs functions (2 points):
  - use the functions in Yocto/Math that directly support refraction
- **Large Scenes** (2 points):
  - render the supplied large scenes at very high sampling rate to test your renderer
- **Denoising** (6 points):
  - add support for denoising using [Intel Open Image Denoise](https://github.com/OpenImageDenoise/oidn)
  - to do this, you need to export an image for the albedo and one for the normals,
    in additions to the rendered images
  - you can either modify your renderer to compute albedo and normals on the fly
    while rendering or write an albedo and normals shader
  - compile Intel OIDN on your machine
  - use the supplied example application to denoise your images
  - submit a comparison between noisy, denoised, and reference images
- **Stratified Sampling 1** (4 points):
  - implement startified sampling in a shader that only sendds rays out
    uniformly in the cosine-weghted hemisphere for environment maps with no recursion
  - compare this sampling to pure random sampling
- **Stratified Sampling 2** (4 points):
  - implement startified sampling to choose environment map pixels as in [Pbrt](http://www.pbr-book.org)
  - compare this sampling to pure random sampling
- **Implicit Surfaces** (8 points):
  - implement a raytracer for implicit surfaces as discussed in [Scratchapixel](https://www.scratchapixel.com/lessons/advanced-rendering/rendering-distance-fields)
  - you can either extend your path tracer with new shape types (we suggest this)
    or create a whole new renderer
  - implement a ray-implicit intersection
  - create a scene to demonstrate your findings; you can take the models from Shadertoy
- **MYOS**, make your own scene (2 points):
  - create additional scenes that you can render from models assembled by you
  - to create new scenes, you can directly edit the json files that are just
    a serialization of the same-name variables in Yocto/SceneIO
  - remember that to get proper lighting yoou should either use environment
    maps or emissive materials
    - you can find high quality environment maps on [HDRIHaven](https://hdrihaven.com)
  - as a starting point you could use one of the test scenes and put new objects and environments
    - for material textures, try to search for "free PBR textures" on Google
    - for 3D models I am not sure; you can try either [CGTrader](http://ccgtrader.com), [SketchFab](http://www.sketchfab.com), [ModelHaven](https://3dmodelhaven.com)
  - you could also try to edit a scene in Blender and export it in glTF, use ysceneproc from Yocto/GL to convert it and edit the resulting Json
    - note though that in general lights and materials are not properly exported

## Submission

To submit the homework, you need to pack a ZIP file that contains the code
you write and the images it generates, i.e. the ZIP _with only the
`yocto_pathtrace/` and `out/` directories_.
The file should be called `<lastname>_<firstname>_<studentid>.zip`
(`<cognome>_<nome>_<matricola>.zip`) and you should exclude
all other directories. Send it on Google Classroom.
