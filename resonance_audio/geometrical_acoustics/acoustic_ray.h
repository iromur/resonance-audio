/*
Copyright 2018 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS-IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef RESONANCE_AUDIO_GEOMETRICAL_ACOUSTICS_ACOUSTIC_RAY_H_
#define RESONANCE_AUDIO_GEOMETRICAL_ACOUSTICS_ACOUSTIC_RAY_H_

#include <array>
#include <vector>

#include "embree3/rtcore.h"
#include "embree3/rtcore_ray.h"
#include "base/constants_and_types.h"

namespace vraudio {

// A class extending Embree's RTCRay (https://embree.github.io/api.html) with
// data needed for acoustic computations.
// It exposes useful fields through accessors.
class RTC_ALIGN(16) AcousticRay : public RTCRayHit {
 public:
  enum class RayType {
    kSpecular,
    kDiffuse,
  };

  // A constant used to indicate that the ray extends to infinity, if
  // ray.t_far() == AcousticRay::kInfinity.
  static const float kInfinity;

  // Used to offset a ray's origin slightly so that it will not
  // intersect with the same geometry/primitive that it was generated from
  // (by reflection, transmission, diffraction, etc.).
  static const float kRayEpsilon;

  // Default constructor. Constructs a ray whose origin is at (0, 0, 0) and
  // points in the +x direction.
  AcousticRay()
      : energies_(), type_(RayType::kSpecular), prior_distance_(0.0f) {
    ray.org_x = 0.0f;
    ray.org_y = 0.0f;
    ray.org_z = 0.0f;
    ray.dir_x = 1.0f;
    ray.dir_y = 0.0f;
    ray.dir_z = 0.0f;
    ray.tnear = 0.0f;
    ray.tfar = kInfinity;
    hit.Ng_x = 0.0f;
    hit.Ng_y = 0.0f;
    hit.Ng_z = 0.0f;
    hit.geomID = RTC_INVALID_GEOMETRY_ID;

    // Members in RTCRay that we do not use (or whose initial values we do not
    // care) are not initialized:
    // align0, align1, align2, time, mask, u, v, primID, instID.
  }

  // Constructor.
  //
  // @param origin Origin of the ray.
  // @param direction Direction of the ray.
  // @param t_near Ray parameter corresponding to the start of the ray.
  // @param t_far Ray parameter corresponding to the end of the ray. Pass in
  //     AcousticRay::kInfinity if there is no end point.
  // @param energies Ray energies for all frequency bands.
  // @param ray_type Type of ray.
  // @param prior_distance Distance traveled before this ray.
  AcousticRay(const float origin[3], const float direction[3], float t_near,
              float t_far,
              const std::array<float, kNumReverbOctaveBands>& energies,
              RayType ray_type, float prior_distance)
      : energies_(energies), type_(ray_type), prior_distance_(prior_distance) {
    ray.org_x = origin[0];
    ray.org_y = origin[1];
    ray.org_z = origin[2];
    ray.dir_x = direction[0];
    ray.dir_y = direction[1];
    ray.dir_z = direction[2];
    ray.tnear = t_near;
    ray.tfar = t_far;
    hit.Ng_x = 0.0f;
    hit.Ng_y = 0.0f;
    hit.Ng_z = 0.0f;
    hit.geomID = RTC_INVALID_GEOMETRY_ID;

    // Members in RTCRay that we do not use (or whose initial values we do not
    // care) are not initialized:
    // align0, align1, align2, time, mask, u, v, primID, instID.
  }

  // Ray origin.
  const float* origin() const { return &ray.org_x; }
  void set_origin(const float origin[3]) {
    ray.org_x = origin[0];
    ray.org_y = origin[1];
    ray.org_z = origin[2];
  }

  // Ray direction.
  const float* direction() const { return &ray.dir_x; }
  void set_direction(const float direction[3]) {
    ray.dir_x = direction[0];
    ray.dir_y = direction[1];
    ray.dir_z = direction[2];
  }

  // Ray parameter t corresponding to the start of the ray segment.
  const float t_near() const { return ray.tnear; }
  void set_t_near(float t_near) { ray.tnear = t_near; }

  // Ray parameter t corresponding to the end of the ray segment.
  const float t_far() const { return ray.tfar; }
  void set_t_far(float t_far) { ray.tfar = t_far; }

  // Functions intersected_*() will only return meaningful results after
  // Intersect() is called, otherwise they return default values as
  // described below.
  //
  // Not normalized geometry normal at the intersection point.
  // Default value: Vec3fa(0, 0, 0).
  const float* intersected_geometry_normal() const { return &hit.Ng_x; }
  void set_intersected_geometry_normal(
      const float intersected_geometry_normal[3]) {
    hit.Ng_x = intersected_geometry_normal[0];
    hit.Ng_y = intersected_geometry_normal[1];
    hit.Ng_z = intersected_geometry_normal[2];
  }

  // Id of the intersected geometry.
  // Default value: kInvalidGeometryId.
  const unsigned int intersected_geometry_id() const { return hit.geomID; }

  // Id of the intersected primitive.
  // Default value: kInvalidPrimitiveId.
  const unsigned int intersected_primitive_id() const { return hit.primID; }

  // Ray energies for all frequency bands.
  const std::array<float, kNumReverbOctaveBands>& energies() const {
    return energies_;
  }
  void set_energies(const std::array<float, kNumReverbOctaveBands>& energies) {
    energies_ = energies;
  }

  // Ray type.
  const RayType type() const { return type_; }
  void set_type(const RayType type) { type_ = type; }

  // Prior distance.
  const float prior_distance() const { return prior_distance_; }
  void set_prior_distance(float prior_distance) {
    prior_distance_ = prior_distance;
  }

  // Finds the first intersection between this ray and a scene. Some fields
  // will be filled/mutated, which can be examined by the following functions:
  // - t_far()
  // - intersected_geometry_normal()
  // - intersected_geometry_id()
  // - intersected_primitive_id()
  //
  // @param scene An RTCScene to test the intersection.
  // @return True if an intersection is found.
  bool Intersect(RTCScene scene) {
    RTCIntersectContext intersectContext;
    rtcInitIntersectContext(&intersectContext);
    rtcIntersect1(scene, &intersectContext, this);
    return hit.geomID != RTC_INVALID_GEOMETRY_ID;
  }

 private:
  // Used to determine early-termination of rays. May also be used to model
  // source strength.
  std::array<float, kNumReverbOctaveBands> energies_;

  // Ray type.
  RayType type_ = RayType::kSpecular;

  // Accumulated distance traveled on the same path before this ray starts.
  float prior_distance_ = 0.0f;
};

}  // namespace vraudio

#endif  // RESONANCE_AUDIO_GEOMETRICAL_ACOUSTICS_ACOUSTIC_RAY_H_
