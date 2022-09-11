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

#include "geometrical_acoustics/sphere.h"
#include "geometrical_acoustics/acoustic_ray.h"

#include <cmath>
#include <utility>

#include "Eigen/Core"

namespace vraudio {

using Eigen::Vector3f;

namespace {
// Fills intersection data in an RTCRay.
//
// @param sphere Sphere that intersects with the ray.
// @param t0 Ray parameter corresponding to the first intersection point.
// @param t1 Ray parameter corresponding to the second intersection point.
// @param ray RTCRay that potentially intersects with |sphere|, whose data
//     fields are to be filled.
inline void FillRaySphereIntersectionData(const Sphere& sphere, float t0,
                                          float t1, RTCRayN* ray, RTCHitN* hit) {
  // For convenience we enforce that t0 <= t1.
  if (t0 > t1) {
    std::swap(t0, t1);
  }

  // In our application we only consider "intersecting" if the ray starts and
  // ends outside the sphere, i.e. only if ray.tnear < t0 and ray.tfar > t1.
  if ((RTCRayN_tnear(ray, 1, 0) >= t0) || (RTCRayN_tfar(ray, 1, 0) <= t1)) {
    return;
  }

  // Set intersection-related data.
  RTCRayN_tfar(ray, 1, 0) = t0;
  RTCHitN_geomID(hit, 1, 0) = sphere.geometry_id;

  // ray.Ng is the normal at the intersection point. For a sphere the normal is
  // always pointing radially outward, i.e. normal = p - sphere.center, where
  // p is the intersection point. Of the two intersection points, We use the
  // first.
  RTCHitN_Ng_x(hit, 1, 0) = RTCRayN_org_x(ray, 1, 0) + t0 * RTCRayN_dir_x(ray, 1, 0) - sphere.center[0];
  RTCHitN_Ng_y(hit, 1, 0) = RTCRayN_org_y(ray, 1, 0) + t0 * RTCRayN_dir_y(ray, 1, 0) - sphere.center[1];
  RTCHitN_Ng_z(hit, 1, 0) = RTCRayN_org_z(ray, 1, 0) + t0 * RTCRayN_dir_z(ray, 1, 0) - sphere.center[2];
}

}  // namespace

void SphereIntersection(const Sphere& sphere, RTCRayHitN* rayhit, unsigned int N) {
  // The intersection is tested by finding if there exists a point p that is
  // both on the ray and on the sphere:
  // - Point on the ray: p = ray.origin + t * ray.direction, where t is a
  //   parameter.
  // - Point on the sphere: || p - sphere.center || = sphere.radius.
  //
  // Solving the above two equations for t leads to solving a quadratic
  // equation:
  //     at^2 + bt + c = 0,
  // where
  //     a = || ray.direction, ray.direction ||^2
  //     b = 2 * Dot(ray.direction, ray.origin - sphere.center)
  //     c = || ray.origin - sphere.center ||^2.
  // The two possible solutions to this quadratic equation are:
  //     t0 = - b - sqrt(b^2 - 4ac) / 2a
  //     t1 = - b + sqrt(b^2 - 4ac) / 2a.
  // The existence of real solutions can be tested if a discriminant value,
  // b^2 - 4ac, is greater than 0 (we treat discriminant == 0, which corresponds
  // to the ray touching the sphere at a single point, as not intersecting).

  // we only deal with ray packets of size 1
  if (N != 1)
    return;

  RTCRayN* ray = RTCRayHitN_RayN(rayhit, 1);
  RTCHitN* hit = RTCRayHitN_HitN(rayhit, 1);
  
  // Vector pointing from the sphere's center to the ray's origin, i.e.
  // (ray.origin - sphere.center) in the above equations.
  const Vector3f center_ray_origin_vector(RTCRayN_org_x(ray, 1, 0) - sphere.center[0],
                                          RTCRayN_org_y(ray, 1, 0) - sphere.center[1],
                                          RTCRayN_org_z(ray, 1, 0) - sphere.center[2]);
  const Vector3f ray_direction(RTCRayN_dir_x(ray, 1, 0),
			       RTCRayN_dir_y(ray, 1, 0),
			       RTCRayN_dir_z(ray, 1, 0));
  const float a = ray_direction.squaredNorm();
  const float b = 2.0f * center_ray_origin_vector.dot(ray_direction);
  const float c =
      center_ray_origin_vector.squaredNorm() - sphere.radius * sphere.radius;
  const float discriminant = b * b - 4.0f * a * c;

  // No intersection; do nothing.
  if (discriminant <= 0.0f) {
    return;
  }

  // Solve for t0 and t1. As suggested in "Physically Based Rendering" by Pharr
  // and Humphreys, directly computing - b +- sqrt(b^2 - 4ac) / 2a gives poor
  // numeric precision when b is close to +- sqrt(b^2 - 4ac), and a more stable
  // form is used instead:
  //    t0 = q / a
  //    t1 = c / q,
  // where
  //    q = -(b - sqrt(b^2 - 4ac)) / 2 for b < 0
  //        -(b + sqrt(b^2 - 4ac)) / 2 otherwise.
  const float sqrt_discriminant = std::sqrt(discriminant);
  const float q = (b < 0.0f) ? -0.5f * (b - sqrt_discriminant)
                             : -0.5f * (b + sqrt_discriminant);
  const float t0 = q / a;
  const float t1 = c / q;

  FillRaySphereIntersectionData(sphere, t0, t1, ray, hit);
}

void SphereIntersection(const Sphere& sphere, AcousticRay* acoustic_ray)
{
  // construct 1-element ray pack for intersection function
  RTCRayHitNp rayhitN;

  RTCRayNp& rayN = rayhitN.ray;
  RTCRay& ray = acoustic_ray->ray;
  rayN.org_x = &ray.org_x;
  rayN.org_y = &ray.org_y;
  rayN.org_z = &ray.org_z;
  rayN.tnear = &ray.tnear;
  rayN.dir_x = &ray.dir_x;
  rayN.dir_y = &ray.dir_y;
  rayN.dir_z = &ray.dir_z;
  rayN.time = &ray.time;
  rayN.tfar = &ray.tfar;
  rayN.mask = &ray.mask;
  rayN.id = &ray.id;
  rayN.flags = &ray.flags;
    
  RTCHitNp& hitN = rayhitN.hit;
  RTCHit& hit = acoustic_ray->hit;
  hitN.Ng_x = &hit.Ng_x;
  hitN.Ng_y = &hit.Ng_y;
  hitN.Ng_z = &hit.Ng_z;
  hitN.u = &hit.u;
  hitN.v = &hit.v;
  hitN.primID = &hit.primID;
  hitN.geomID = &hit.geomID;
  for (unsigned int l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; l++)
    hitN.instID[l] = &hit.instID[l];

  SphereIntersection(sphere, reinterpret_cast<RTCRayHitN*>(&rayhitN), 1);
}

}  // namespace vraudio
