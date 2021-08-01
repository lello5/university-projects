# Yocto/Pathtrace: Tiny Volumetric Path Tracer

In this homework, you will learn how to build a simple path tracer with support
for subdivision surfaces, displacement and subsurface scattering.
In particular, you will learn how to

- handle subdivision surfaces,
- handle normal mapping (we have implemented displacement for you),
- write a path tracer with support for homogeneous volumes.

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
are declared in `yocto_pathtrace.h`. Your renderer is called by `yscenetrace.cpp`
for a command-line interface and `ysceneitraces.cpp` that show a simple
user interface.

This repository also contains tests that are executed from the command line
as shown in `run.sh`. The rendered images are saved in the `out/` directory.
The results should match the ones in the directory `check/`.

## Functionality (26 punti)

In this homework you will implement the following features:

- **Subdivision Surfaces** in functions `subdivide_catmullclark()`:
  - implement Catmull-Clark subdivision following the slides
  - while the solution should match the one in Yocto/Shape, your
    implementation _cannot_ use the Yocto one, neither calling it nor
    adapting the code – I will be able to tell since the Yocto/Shape code
    supports more features
- **Normal Mapping** in function `eval_normalmap()`:
  - implement normal mapping as per the slides
  - you can use `triangle_tangents_fromuv()` to get tangents
- **Volumetric Path Tracing** in function `trace_volpath()`:
  - follow the slides and implement a volumetric path tracer
  - you have to also implement all support functions,
    namely `XXX_transmittance()` and `XXX_scattering()`
  - you can use the corresponding functions in Yocto/Math

## Extra Credit (14 punti)

As usual, here are the extra credit work you can choose to do. As before,
the maximum points sum is the one reported above. But you can do more of these
if desired.

New this time is that if you do extra credit, you have to **prepare a short PDF**
that describes which feature you have implemented and includes images for
those features. **Without the PDF the extra credit will not be graded**.
The PDF should be vrey short and just say what you did succinctly, like with
a bullet points lists.

- **Hair Shading** (8 punti):
  - implement a hair BSDF to shade realistic-looking hairs
  - you can follow the algorithm presented in [pbrt](https://www.pbrt.org/hair.pdf)
    that also includes a full code implementation
  - you can get example hair models from [Bitterli](https://benedikt-bitterli.me/resources/)
- **Volumetric Path Tracing I - Delta Tracking** in path tracer (10 punti):
  - integrate delta tracking for smoke and clouds, based your implementation on
    [pbrt](http://www.pbr-book.org/3ed-2018/Light_Transport_II_Volume_Rendering/Sampling_Volume_Scattering.html)
  - to integrate it, you have to implement a new `sample_transmittance()`
    that both samples the distances and returns the weight
  - implement volumetric textures, for now just using `image::volume`,
    for density and emission
  - also you have to add volumes to the renderer, which for now can be a hack
    to either hardcode a function that makes one or use a procedural
  - you can use volumes from Yocto/Image
- **Volumetric Path Tracing II - Modernized Volumetric Scattering** (14 punti):
  - implement proper volumetric scattering for heterogenous materials
  - implement volumetric textures, for now just using `image::volume`,
    for density and emission
  - add these textures to both the SceneIO loader and the path tracer
  - implement a modern volumetric method from Algorithm 1 of [Miller et al](https://cs.dartmouth.edu/~wjarosz/publications/miller19null.html) --- ignore most of the math here since it is not that helpful
  - you can use the implementation from [pbrt-v4](https://github.com/mmp/pbrt-v4)
  - get examples from OpenVDB
- **Texture Synthesis** (6 points):
  - implement texture synthesis following the [Disney method](http://www.jcgt.org/published/0008/04/02/paper.pdf)
  - [source code](https://benedikt-bitterli.me/histogram-tiling/)
  - for this, just add properties to the `trace::texture` object and change `eval_texture()`
- **SDF Shapes** (10 points max):
  - implement a high quality SDF shape object that is integrated within the path tracer
  - represent SDFs as sparse hash grids; an [example on GPU](https://nosferalatu.com/SimpleGPUHashTable.html)
  - add a new `sdf` to represent the SDF and have `object` hold a pointer to either `sdf` or `shape`
  - change `eval_xxx()` to work for SDFs too
  - add code to load/save SDFs in `sceneio`; just make up your own file format for now
  - get examples from OpenVDB
- **Ray-Patch Intersection** (4 points):
  - implement bilinear patch intersection following this [paper](https://link.springer.com/chapter/10.1007/978-1-4842-4427-2_8)
  - compare it to intersecting two triangles in terms of speed
- **Quad BVH** (8 points):
  - implement a QBVH tree by translating our BVH2 to a BVH4 as described [here](http://jcgt.org/published/0004/04/05/)
  - implement QBVH intersection with only one ray at a time
  - optimize QBVH intersection using SIMD instruction
  - for example code for the optimization look for QBVH on
  - compare it to intersecting the standard BVH in terms of speed
- **Adaptive rendering** (4 points):
  - implement a stropping criterion to focus render resources when more needed
  - technique is described [here](https://jo.dreggn.org/home/2009_stopping.pdf)
  - possible implementation [here](https://github.com/mkanada/yocto-gl)
  - your job here is to integrate really well the code, provide test cases and make it run interactively
- **Better adaptive rendering** (8 points):
  - implement adaptive rendering and reconstruction as described [here](https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.100/institut/Papers/atrousGIfilter.pdf)
  - this will provide better quality than the above method
  - alsop consider the [new variant from NVidia](https://www.highperformancegraphics.org/wp-content/uploads/2017/Papers-Session1/HPG2017_SpatiotemporalVarianceGuidedFiltering.pdf)

## Submission

To submit the homework, you need to pack a ZIP file that contains the code
you write and the images it generates, i.e. the ZIP _with only the
`yocto_pathtrace/` and `out/` directories_. _If doing extra creedits also include
the PDF and and the test scenes._
The file should be called `<lastname>_<firstname>_<studentid>.zip`
(`<cognome>_<nome>_<matricola>.zip`) and you should exclude
all other directories. Send it on Google Classroom.
