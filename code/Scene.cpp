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
    body.m_position = Vec3(1, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    body.m_position = Vec3(3, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    body.m_position = Vec3(0, 0, -50);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 1.0f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeSphere(50.0f);
    m_bodies.push_back(body);

    // TODO: Add code
}

// 用于在Scene::Update()中比较Contact的发生时间
int CompareContacts(const void* p1, const void* p2)
{
    contact_t a = *(contact_t*)p1;
    contact_t b = *(contact_t*)p2;

    if (a.timeOfImpact < b.timeOfImpact)
    {
        return -1;
    }
    if (a.timeOfImpact == b.timeOfImpact)
    {
        return 0;
    }
    return 1;
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

    int num_contacts = 0;
    const int max_contacts = m_bodies.size() * m_bodies.size();
    auto contacts = static_cast<contact_t*>(alloca(sizeof(contact_t) * max_contacts)); // 用于存放contacts

    // 检测碰撞，收集起来
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
            if (Intersect(body_a, body_b, dt_sec, contact))
            {
                contacts[num_contacts] = contact;
                num_contacts++;
            }
        }
    }

    // 按照碰撞发生的先后顺序（time of impact）对contacts排序
    if (num_contacts > 1)
    {
        qsort(contacts, num_contacts, sizeof(contact_t), CompareContacts);
    }

    // 按照时间排序好的contacts顺序解算contact
    // 每次都只模拟到下一次碰撞的情况，然后重新解算碰撞，跟新contact
    float accumulated_time = 0.0f;
    for (int i = 0; i < num_contacts; i++)
    {
        contact_t& contact = contacts[i];
        const float dt = contact.timeOfImpact - accumulated_time;

        Body* body_a = contact.bodyA;
        Body* body_b = contact.bodyB;

        if (body_a->m_inverseMass == 0.0f && body_b->m_inverseMass == 0.0f)
        {
            continue;
        }

        // 更新位置
        for (int j = 0; j < m_bodies.size(); j++)
        {
            // Position update
            m_bodies[j].Update(dt);
        }

        ResolveContact(contact);
        accumulated_time += dt;
    }

    // 跟新此帧时间剩下的位置
    const float time_remaining = dt_sec - accumulated_time;
    if (time_remaining > 0.0f)
    {
        for (int i = 0; i < m_bodies.size(); i++)
        {
            m_bodies[i].Update(time_remaining);
        }
    }
}
