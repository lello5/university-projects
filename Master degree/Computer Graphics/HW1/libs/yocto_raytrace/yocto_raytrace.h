//
// # Yocto/RayTrace: Tiny raytracer tracer
//
//
// Yocto/RayTrace is a simple ray tracing library with support for microfacet
// materials, area and environment lights, and advacned sampling.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#ifndef _YOCTO_RAYTRACE_H_
#define _YOCTO_RAYTRACE_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>

#include <functional>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// USING DIRECTIVES
// -----------------------------------------------------------------------------
namespace yocto {

using std::function;

}

// -----------------------------------------------------------------------------
// SCENE AND RENDERING DATA
// -----------------------------------------------------------------------------
namespace yocto {

// BVH tree node containing its bounds, indices to the BVH arrays of either
// primitives or internal nodes, the node element type,
// and the split axis. Leaf and internal nodes are identical, except that
// indices refer to primitives for leaf nodes or other nodes for internal nodes.
struct raytrace_bvh_node {
  bbox3f bbox;
  int    start;
  short  num;
  bool   internal;
  byte   axis;
};

// BVH tree stored as a node array with the tree structure is encoded using
// array indices. BVH nodes indices refer to either the node array,
// for internal nodes, or the primitive arrays, for leaf nodes.
// Application data is not stored explicitly.
struct raytrace_bvh_tree {
  vector<raytrace_bvh_node> nodes      = {};
  vector<int>               primitives = {};
};

// Camera based on a simple lens model. The camera is placed using a frame.
// Camera projection is described in photorgaphics terms. In particular,
// we specify fil size (35mm by default), the lens' focal length, the focus
// distance and the lens aperture. All values are in meters.
// Here are some common aspect ratios used in video and still photography.
// 3:2    on 35 mm:  0.036 x 0.024
// 16:9   on 35 mm:  0.036 x 0.02025 or 0.04267 x 0.024
// 2.35:1 on 35 mm:  0.036 x 0.01532 or 0.05640 x 0.024
// 2.39:1 on 35 mm:  0.036 x 0.01506 or 0.05736 x 0.024
// 2.4:1  on 35 mm:  0.036 x 0.015   or 0.05760 x 0.024 (approx. 2.39 : 1)
// To compute good apertures, one can use the F-stop number from phostography
// and set the aperture to focal_leangth/f_stop.
struct raytrace_camera {
  frame3f frame    = identity3x4f;
  float   lens     = 0.050;
  vec2f   film     = {0.036, 0.024};
  float   focus    = 10000;
  float   aperture = 0;
};

// Texture containing either an LDR or HDR image. HdR images are encoded
// in linear color space, while LDRs are encoded as sRGB.
struct raytrace_texture {
  image<vec4f> hdr = {};
  image<vec4b> ldr = {};
};

// Material for surfaces, lines and triangles.
// For surfaces, uses a microfacet model with thin sheet transmission.
// The model is based on OBJ, but contains glTF compatibility.
// For the documentation on the values, please see the OBJ format.
struct raytrace_material {
  // material
  vec3f emission     = {0, 0, 0};
  vec3f color        = {0, 0, 0};
  float specular     = 0;
  float roughness    = 0;
  float metallic     = 0;
  float ior          = 1.5;
  vec3f spectint     = {1, 1, 1};
  float transmission = 0;
  vec3f scattering   = {0, 0, 0};
  float scanisotropy = 0;
  float trdepth      = 0.01;
  float opacity      = 1;
  bool  thin         = false;

  // textures
  raytrace_texture* emission_tex     = nullptr;
  raytrace_texture* color_tex        = nullptr;
  raytrace_texture* specular_tex     = nullptr;
  raytrace_texture* metallic_tex     = nullptr;
  raytrace_texture* roughness_tex    = nullptr;
  raytrace_texture* transmission_tex = nullptr;
  raytrace_texture* spectint_tex     = nullptr;
  raytrace_texture* scattering_tex   = nullptr;
  raytrace_texture* opacity_tex      = nullptr;
};

// Shape data represented as an indexed meshes of elements.
// May contain either points, lines, triangles and quads.
// Additionally, we support faceavarying primitives where
// each verftex data has its own topology.
struct raytrace_shape {
  // primitives
  vector<int>   points    = {};
  vector<vec2i> lines     = {};
  vector<vec3i> triangles = {};

  // vertex data
  vector<vec3f> positions = {};
  vector<vec3f> normals   = {};
  vector<vec2f> texcoords = {};
  vector<float> radius    = {};

  // computed properties
  raytrace_bvh_tree* bvh = nullptr;

  // cleanup
  ~raytrace_shape();
};

// Instance
struct raytrace_instance {
  frame3f            frame    = identity3x4f;
  raytrace_shape*    shape    = nullptr;
  raytrace_material* material = nullptr;
};

// Environment map
struct raytrace_environment {
  frame3f           frame        = identity3x4f;
  vec3f             emission     = {0, 0, 0};
  raytrace_texture* emission_tex = nullptr;
};

// Scene comprised an array of objects whose memory is owened by the scene.
// All members are optional,Scene objects (camera, instances, environments)
// have transforms defined internally. A scene can optionally contain a
// node hierarchy where each node might point to a camera, instance or
// environment. In that case, the element transforms are computed from
// the hierarchy. Animation is also optional, with keyframe data that
// updates node transformations only if defined.
struct raytrace_scene {
  vector<raytrace_camera*>      cameras      = {};
  vector<raytrace_instance*>    instances    = {};
  vector<raytrace_shape*>       shapes       = {};
  vector<raytrace_material*>    materials    = {};
  vector<raytrace_texture*>     textures     = {};
  vector<raytrace_environment*> environments = {};

  // computed properties
  raytrace_bvh_tree* bvh = nullptr;

  // cleanup
  ~raytrace_scene();
};

// Rendering state
struct raytrace_state {
  image<vec4f>     render       = {};
  image<vec4f>     accumulation = {};
  image<int>       samples      = {};
  image<rng_state> rngs         = {};
};

}  // namespace yocto

// -----------------------------------------------------------------------------
// HIGH LEVEL API
// -----------------------------------------------------------------------------
namespace yocto {

// Add scene elements
raytrace_camera*      add_camera(raytrace_scene* scene);
raytrace_instance*    add_instance(raytrace_scene* scene);
raytrace_texture*     add_texture(raytrace_scene* scene);
raytrace_material*    add_material(raytrace_scene* scene);
raytrace_shape*       add_shape(raytrace_scene* scene);
raytrace_environment* add_environment(raytrace_scene* scene);

// camera properties
void set_frame(raytrace_camera* camera, const frame3f& frame);
void set_lens(raytrace_camera* camera, float lens, float aspect, float film);
void set_focus(raytrace_camera* camera, float aperture, float focus);

// instance properties
void set_frame(raytrace_instance* instance, const frame3f& frame);
void set_material(raytrace_instance* instance, raytrace_material* material);
void set_shape(raytrace_instance* instance, raytrace_shape* shape);

// texture properties
void set_texture(raytrace_texture* texture, const image<vec4b>& img);
void set_texture(raytrace_texture* texture, const image<vec4f>& img);

// material properties
void set_emission(raytrace_material* material, const vec3f& emission,
    raytrace_texture* emission_tex = nullptr);
void set_color(raytrace_material* material, const vec3f& color,
    raytrace_texture* color_tex = nullptr);
void set_specular(raytrace_material* material, float specular = 1,
    raytrace_texture* specular_tex = nullptr);
void set_ior(raytrace_material* material, float ior);
void set_metallic(raytrace_material* material, float metallic,
    raytrace_texture* metallic_tex = nullptr);
void set_transmission(raytrace_material* material, float transmission,
    bool thin, float trdepth, raytrace_texture* transmission_tex = nullptr);
void set_roughness(raytrace_material* material, float roughness,
    raytrace_texture* roughness_tex = nullptr);
void set_opacity(raytrace_material* material, float opacity,
    raytrace_texture* opacity_tex = nullptr);
void set_thin(raytrace_material* material, bool thin);
void set_scattering(raytrace_material* material, const vec3f& scattering,
    float scanisotropy, raytrace_texture* scattering_tex = nullptr);

// shape properties
void set_points(raytrace_shape* shape, const vector<int>& points);
void set_lines(raytrace_shape* shape, const vector<vec2i>& lines);
void set_triangles(raytrace_shape* shape, const vector<vec3i>& triangles);
void set_positions(raytrace_shape* shape, const vector<vec3f>& positions);
void set_normals(raytrace_shape* shape, const vector<vec3f>& normals);
void set_texcoords(raytrace_shape* shape, const vector<vec2f>& texcoords);
void set_radius(raytrace_shape* shape, const vector<float>& radius);

// environment properties
void set_frame(raytrace_environment* environment, const frame3f& frame);
void set_emission(raytrace_environment* environment, const vec3f& emission,
    raytrace_texture* emission_tex = nullptr);

// Type of tracing algorithm
enum struct raytrace_shader_type {
  // clang-format on
  raytrace,  // path tracing
  eyelight,  // eyelight rendering
  normal,    // normals
  texcoord,  // texcoords
  color,     // colors
             // clang-format off
   toon,
};

// Default trace seed
const auto default_seed = 961748941ull;

// Options for trace functions
struct raytrace_params {
  int             resolution = 720;
  raytrace_shader_type     shader     = raytrace_shader_type::raytrace;
  int             samples    = 512;
  int             bounces    = 4;
  float           clamp      = 100000;
  uint64_t        seed       = default_seed;
  bool            noparallel = false;
  int             pratio     = 8;
};

const auto raytrace_shader_names = vector<string>{
    "raytrace", "eyelight", "normal", "texcoord", "color", "toon"};

// Progress report callback
using progress_callback =
    function<void(const string& message, int current, int total)>;

// Build the bvh acceleration structure.
void init_bvh(raytrace_scene* scene, const raytrace_params& params,
    progress_callback progress_cb = {});

// Initialize the rendering state
struct state;
void init_state(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const raytrace_params& params);

// Progressively computes an image.
void render_samples(raytrace_state* state, 
    const raytrace_scene* scene, const raytrace_camera* camera,
    const raytrace_params& params);

}  // namespace yocto

// -----------------------------------------------------------------------------
// INTERSECTION
// -----------------------------------------------------------------------------
namespace yocto {

// Results of intersect functions that include hit flag, the instance id,
// the shape element id, the shape element uv and intersection distance.
// Results values are set only if hit is true.
struct raytrace_intersection {
  int   instance   = -1;
  int   element  = -1;
  vec2f uv       = {0, 0};
  float distance = 0;
  bool  hit      = false;
};

// Intersect ray with a bvh returning either the first or any intersection
// depending on `find_any`. Returns the ray distance , the instance id,
// the shape element index and the element barycentric coordinates.
raytrace_intersection intersect_scene_bvh(const raytrace_scene* scene, 
    const ray3f& ray, bool find_any = false, bool non_rigid_frames = true);
raytrace_intersection intersect_instance_bvh(const raytrace_instance* instance,
    const ray3f& ray, bool find_any = false, bool non_rigid_frames = true);

}  // namespace yocto

#endif
