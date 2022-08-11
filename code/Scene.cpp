//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene()
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        delete m_bodies[i].m_shape;
    }
    m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset()
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        delete m_bodies[i].m_shape;
    }
    m_bodies.clear();

    Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
    Body body;
    body.m_position = Vec3(0, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    body.m_position = Vec3(0, 0, -1000);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 1.0f;
    body.m_shape = new ShapeSphere(1000.0f);
    m_bodies.push_back(body);

    // TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        Body* body = &m_bodies[i];

        float mass = 1.0f / body->m_inverseMass;
        Vec3 impulse_Gravity = Vec3(0, 0, -10) * mass * dt_sec;
        body->ApplyImpulseLinear(impulse_Gravity);
    }

    // 检测碰撞
    for (int i = 0; i < m_bodies.size(); i++)
    {
        for (int j = i + 1; j < m_bodies.size(); j++)
        {
            Body* body_a = &m_bodies[i];
            Body* body_b = &m_bodies[j];

            if (body_a->m_inverseMass == 0.0f && body_b->m_inverseMass == 0.0f)
            {
                continue;
            }

            contact_t contact;
            if (Intersect(body_a, body_b, contact))
            {
                ResolveContact(contact);
            }
        }
    }

    for (int i = 0; i < m_bodies.size(); i++)
    {
        // Position update
        m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
    }
}
