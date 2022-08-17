//
//	Intersections.h
//
#pragma once
#include "Contact.h"

bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact);
bool Intersect(Body* bodyA, Body* bodyB, float dt, contact_t& contact);
bool Intersect(const Body* body_a, const Body* body_b);

bool RaySphere(const Vec3& ray_start, const Vec3& ray_dir, const Vec3& sphere_center, float sphere_radius, float& t1,
               float& t2);

bool SphereSphereDynamic(const ShapeSphere* shape_a, const ShapeSphere* shape_b, const Vec3& pos_a, const Vec3& pos_b,
                         const Vec3& vel_a, const Vec3& vel_b, float dt, Vec3& pt_a, Vec3& pt_b, float& toi);

bool SphereSphereStatic(const ShapeSphere* shape_a, const ShapeSphere* shape_b, const Vec3& pos_a, const Vec3& pos_b,
                        Vec3& pt_a, Vec3& pt_b);
