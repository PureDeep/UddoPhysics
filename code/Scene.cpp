//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

void AddStandardSandBox(std::vector<Body>& bodies)
{
    Body body;

    body.m_position = Vec3(0, 0, 0);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
    bodies.push_back(body);

    body.m_position = Vec3(50, 0, 0);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
    bodies.push_back(body);

    body.m_position = Vec3(-50, 0, 0);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
    bodies.push_back(body);

    body.m_position = Vec3(0, 25, 0);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
    bodies.push_back(body);

    body.m_position = Vec3(0, -25, 0);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_inverseMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
    bodies.push_back(body);
}

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

    int x = 9, y = 9;

    for (int i = 0; i < x; i++)
    {
        for (int j = 0; j < y; j++)
        {
            body.m_position = Vec3(i * 2, j * 2, 10);
            body.m_orientation = Quat(0, 0, 0, 1);
            body.m_inverseMass = 1.0f;
            body.m_elasticity = 0.5f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeSphere(1.0f);
            m_bodies.push_back(body);
        }
    }

    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            float radius = 80.0f;
            float xx = static_cast<float>(x - 1) * radius * 0.25f;
            float yy = static_cast<float>(y - 1) * radius * 0.25f;
            body.m_position = Vec3(xx, yy, -radius);
            body.m_orientation = Quat(0, 0, 0, 1);
            body.m_linearVelocity.Zero();
            body.m_inverseMass = 0.0f;
            body.m_elasticity = 0.99f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeSphere(radius);
            m_bodies.push_back(body);
        }
    }

    //AddStandardSandBox(m_bodies);
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
    // Gravity
    for (int i = 0; i < m_bodies.size(); i++)
    {
        Body* body = &m_bodies[i];

        float mass = 1.0f / body->m_inverseMass;
        Vec3 impulse_Gravity = Vec3(0, 0, -10) * mass * dt_sec;
        body->ApplyImpulseLinear(impulse_Gravity);
    }

    // BroadPhase
    std::vector<collisionPair_t> collision_pairs;
    BroadPhase(m_bodies.data(), static_cast<int>(m_bodies.size()), collision_pairs, dt_sec);


    // NarrowPhase
    int num_contacts = 0;
    const int max_contacts = m_bodies.size() * m_bodies.size();
    auto contacts = static_cast<contact_t*>(alloca(sizeof(contact_t) * max_contacts)); // 用于存放contacts

    for (int i = 0; i < collision_pairs.size(); i++)
    {
        const collisionPair_t& pair = collision_pairs[i];
        Body* body_a = &m_bodies[pair.a];
        Body* body_b = &m_bodies[pair.b];

        // 跳过质量为0的物体（不可动）
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

    // 按照碰撞发生的先后顺序（time of impact）对contacts排序
    if (num_contacts > 1)
    {
        qsort(contacts, num_contacts, sizeof(contact_t), CompareContacts);
    }

    // 施加impluse
    float accumulated_time = 0.0f;
    for (int i = 0; i < num_contacts; i++)
    {
        contact_t& contact = contacts[i];
        const float dt = contact.timeOfImpact - accumulated_time;

        // 更新位置
        for (int j = 0; j < m_bodies.size(); j++)
        {
            m_bodies[j].Update(dt);
        }

        ResolveContact(contact);
        accumulated_time += dt;
    }


    //更新此帧时间剩下的位置
    const float time_remaining = dt_sec - accumulated_time;
    if (time_remaining > 0.0f)
    {
        for (int i = 0; i < m_bodies.size(); i++)
        {
            m_bodies[i].Update(time_remaining);
        }
    }
}

void Scene::AddShapeSphere()
{
    Body body;

    body.m_position = Vec3(0, 0, 15);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_inverseMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeSphere(1.0f);

    m_bodies.push_back(body);
}
