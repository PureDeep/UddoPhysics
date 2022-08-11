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
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    body.m_position = Vec3(0, 0, -1000);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 0.0f;
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
    // TODO: Add code

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

            if (Intersect(body_a, body_b))
            {
                body_a->m_linearVelocity.Zero();
                body_b->m_linearVelocity.Zero();
            }
        }
    }

    for (int i = 0; i < m_bodies.size(); i++)
    {
        // Position update
        m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
    }
}

/**
 * \brief 检测碰撞
 * \param body_a BodyA
 * \param body_b BodyB
 * \return 返回是否碰撞
 */
bool Scene::Intersect(const Body* body_a, const Body* body_b)
{
    Vec3 distance_ab = body_a->m_position - body_b->m_position;
    auto sphere_a = static_cast<const ShapeSphere*>(body_a->m_shape);
    auto sphere_b = static_cast<const ShapeSphere*>(body_b->m_shape);

    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = distance_ab.GetLengthSqr();

    if (length_square <= (radius_ab * radius_ab))
        return true;
    return false;
}
