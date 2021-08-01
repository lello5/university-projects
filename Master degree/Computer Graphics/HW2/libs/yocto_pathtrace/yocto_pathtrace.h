//
// # Yocto/PathTrace: Tiny pathtracer tracer
//
//
// Yocto/PathTrace is a simple ray tracing library with support for microfacet
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

// TODO: lines
// TODO: opacity
// TODO: plastics
// TODO: progress

#ifndef _YOCTO_PATHTRACE_H_
#define _YOCTO_PATHTRACE_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_sampling.h>

#include <functional>

// -----------------------------------------------------------------------------
// USING DIRECTIVES
// -----------------------------------------------------------------------------
namespace yocto {

using std::function;
using std::string;
using std::vector;

}  // namespace yocto

// -----------------------------------------------------------------------------
// SCENE DATA
// -----------------------------------------------------------------------------
namespace yocto {

// BVH tree node containing its bounds, indices to the BVH arrays of either
// primitives or internal nodes, the node element type,
// and the split axis. Leaf and internal nodes are identical, except that
// indices refer to primitives for leaf nodes or other nodes for internal nodes.
struct pathtrace_bvh_node {
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
struct pathtrace_bvh_tree {
  vector<pathtrace_bvh_node> nodes      = {};
  vector<int>                primitives = {};
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
struct pathtrace_camera {
  frame3f frame    = identity3x4f;
  float   lens     = 0.050;
  vec2f   film     = {0.036, 0.024};
  float   focus    = 10000;
  float   aperture = 0;
};

// Texture containing either an LDR or HDR image. HdR images are encoded
// in linear color space, while LDRs are encoded as sRGB.
struct pathtrace_texture {
  image<vec4f> hdr = {};
  image<vec4b> ldr = {};
};

// Material for surfaces, lines and triangles.
// For surfaces, uses a microfacet model with thin sheet transmission.
// The model is based on OBJ, but contains glTF compatibility.
// For the documentation on the values, please see the OBJ format.
struct pathtrace_material {
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
  pathtrace_texture* emission_tex     = nullptr;
  pathtrace_texture* color_tex        = nullptr;
  pathtrace_texture* specular_tex     = nullptr;
  pathtrace_texture* metallic_tex     = nullptr;
  pathtrace_texture* roughness_tex    = nullptr;
  pathtrace_texture* transmission_tex = nullptr;
  pathtrace_texture* spectint_tex     = nullptr;
  pathtrace_texture* scattering_tex   = nullptr;
  pathtrace_texture* opacity_tex      = nullptr;
};

// Shape data represented as an indexed meshes of elements.
// May contain either points, lines, triangles and quads.
// Additionally, we support faceavarying primitives where
// each verftex data has its own topology.
struct pathtrace_shape {
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
  pathtrace_bvh_tree* bvh = nullptr;

  // cleanup
  ~pathtrace_shape();
};

// Object.
struct pathtrace_instance {
  frame3f             frame    = identity3x4f;
  pathtrace_shape*    shape    = nullptr;
  pathtrace_material* material = nullptr;
};

// Environment map.
struct pathtrace_environment {
  frame3f            frame        = identity3x4f;
  vec3f              emission     = {0, 0, 0};
  pathtrace_texture* emission_tex = nullptr;
};

// Trace lights used during rendering. These are created automatically.
struct pathtrace_light {
  pathtrace_instance*    instance    = nullptr;
  pathtrace_environment* environment = nullptr;
  vector<float>          cdf         = {};
};

// Scene comprised an array of instances whose memory is owened by the scene.
// All members are optional,Scene instances (camera, instances, environments)
// have transforms defined internally. A scene can optionally contain a
// node hierarchy where each node might point to a camera, instance or
// environment. In that case, the element transforms are computed from
// the hierarchy. Animation is also optional, with keyframe data that
// updates node transformations only if defined.
struct pathtrace_scene {
  vector<pathtrace_camera*>      cameras      = {};
  vector<pathtrace_instance*>    instances    = {};
  vector<pathtrace_shape*>       shapes       = {};
  vector<pathtrace_material*>    materials    = {};
  vector<pathtrace_texture*>     textures     = {};
  vector<pathtrace_environment*> environments = {};

  // computed elements
  vector<pathtrace_light*> lights = {};

  // computed properties
  pathtrace_bvh_tree* bvh = nullptr;

  // cleanup
  ~pathtrace_scene();
};

}  // namespace yocto

// -----------------------------------------------------------------------------
// SCENE CREATION
// -----------------------------------------------------------------------------
namespace yocto {

// Add scene elements
pathtrace_camera*      add_camera(pathtrace_scene* scene);
pathtrace_instance*    add_instance(pathtrace_scene* scene);
pathtrace_texture*     add_texture(pathtrace_scene* scene);
pathtrace_material*    add_material(pathtrace_scene* scene);
pathtrace_shape*       add_shape(pathtrace_scene* scene);
pathtrace_environment* add_environment(pathtrace_scene* scene);

// camera properties
void set_frame(pathtrace_camera* camera, const frame3f& frame);
void set_lens(pathtrace_camera* camera, float lens, float aspect, float film);
void set_focus(pathtrace_camera* camera, float aperture, float focus);

// instance properties
void set_frame(pathtrace_instance* instance, const frame3f& frame);
void set_material(pathtrace_instance* instance, pathtrace_material* material);
void set_shape(pathtrace_instance* instance, pathtrace_shape* shape);

// texture properties
void set_texture(pathtrace_texture* texture, const image<vec4b>& img);
void set_texture(pathtrace_texture* texture, const image<vec4f>& img);

// material properties
void set_emission(pathtrace_material* material, const vec3f& emission,
    pathtrace_texture* emission_tex = nullptr);
void set_color(pathtrace_material* material, const vec3f& color,
    pathtrace_texture* color_tex = nullptr);
void set_specular(pathtrace_material* material, float specular = 1,
    pathtrace_texture* specular_tex = nullptr);
void set_ior(pathtrace_material* material, float ior);
void set_metallic(pathtrace_material* material, float metallic,
    pathtrace_texture* metallic_tex = nullptr);
void set_transmission(pathtrace_material* material, float transmission,
    bool thin, float trdepth, pathtrace_texture* transmission_tex = nullptr);
void set_roughness(pathtrace_material* material, float roughness,
    pathtrace_texture* roughness_tex = nullptr);
void set_opacity(pathtrace_material* material, float opacity,
    pathtrace_texture* opacity_tex = nullptr);
void set_thin(pathtrace_material* material, bool thin);
void set_scattering(pathtrace_material* material, const vec3f& scattering,
    float scanisotropy, pathtrace_texture* scattering_tex = nullptr);

// shape properties
void set_points(pathtrace_shape* shape, const vector<int>& points);
void set_lines(pathtrace_shape* shape, const vector<vec2i>& lines);
void set_triangles(pathtrace_shape* shape, const vector<vec3i>& triangles);
void set_positions(pathtrace_shape* shape, const vector<vec3f>& positions);
void set_normals(pathtrace_shape* shape, const vector<vec3f>& normals);
void set_texcoords(pathtrace_shape* shape, const vector<vec2f>& texcoords);
void set_radius(pathtrace_shape* shape, const vector<float>& radius);

// environment properties
void set_frame(pathtrace_environment* environment, const frame3f& frame);
void set_emission(pathtrace_environment* environment, const vec3f& emission,
    pathtrace_texture* emission_tex = nullptr);

// Type of tracing algorithm
enum struct pathtrace_shader_type {
  naive,       // naive path tracing
  path,        // path tracing with mis
  eyelight,    // eyelight rendering
  stratified,  // extracredit
};

// Default trace seed
const auto default_seed = 961748941ull;

// Options for trace functions
struct pathtrace_params {
  int                   resolution = 720;
  pathtrace_shader_type shader     = pathtrace_shader_type::naive;
  int                   samples    = 512;
  int                   bounces    = 8;
  float                 clamp      = 10;
  uint64_t              seed       = default_seed;
  bool                  noparallel = false;
  int                   pratio     = 8;
};

const auto pathtrace_shader_names = vector<string>{
    "naive", "path", "eyelight", "stratified"};

// Progress report callback
using progress_callback =
    function<void(const string& message, int current, int total)>;

// Build the bvh acceleration structure.
void init_bvh(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb = {});

// Rendering state
struct pathtrace_state {
  image<vec4f>     render       = {};
  image<vec4f>     accumulation = {};
  image<int>       samples      = {};
  image<rng_state> rngs         = {};
};

// Initialize the rendering state
void init_state(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const pathtrace_params& params);

// Initialize lights
void init_lights(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb);

// Progressively computes an image.
void render_samples(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const pathtrace_params& params);

}  // namespace yocto

// -----------------------------------------------------------------------------
// INTERSECTION
// -----------------------------------------------------------------------------
namespace yocto {

// Results of intersect functions that include hit flag, the instance id,
// the shape element id, the shape element uv and intersection distance.
// Results values are set only if hit is true.
struct pathtrace_intersection {
  int   instance = -1;
  int   element  = -1;
  vec2f uv       = {0, 0};
  float distance = 0;
  bool  hit      = false;
};

// Intersect ray with a bvh returning either the first or any intersection
// depending on `find_any`. Returns the ray distance , the instance id,
// the shape element index and the element barycentric coordinates.
pathtrace_intersection intersect_scene_bvh(const pathtrace_scene* scene,
    const ray3f& ray, bool find_any = false, bool non_rigid_frames = true);
pathtrace_intersection intersect_instance_bvh(
    const pathtrace_instance* instance, const ray3f& ray, bool find_any = false,
    bool non_rigid_frames = true);

}  // namespace yocto

#endif
