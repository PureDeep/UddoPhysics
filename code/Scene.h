//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "Physics/Constraints.h"
#include "Physics/Manifold.h"

/*
====================================================
Scene
====================================================
*/
class Scene
{
public:
    Scene() { m_bodies.reserve(128); }
    ~Scene();

    void Reset();
    void Initialize();
    void Update(float dt_sec);

    static bool Intersect(const Body* body_a, const Body* body_b);

    std::vector<Body> m_bodies;
    std::vector<Constraint*> m_constraints;
    ManifoldCollector m_manifolds;
};
