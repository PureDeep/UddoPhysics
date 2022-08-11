//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact)
{
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;

    const Vec3 ab = bodyB->m_position - bodyA->m_position;
    contact.normal = ab;
    contact.normal.Normalize();

    const ShapeSphere* shpere_a = dynamic_cast<ShapeSphere*>(bodyA->m_shape);
    const ShapeSphere* shpere_b = dynamic_cast<ShapeSphere*>(bodyB->m_shape);

    contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * shpere_a->m_radius;
    contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * shpere_b->m_radius;

    const float radius_ab = shpere_a->m_radius + shpere_b->m_radius;
    const float length_square = ab.GetLengthSqr();
    if (length_square <= (radius_ab * radius_ab))
        return true;

    return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact)
{
    // TODO: Add Code

    return false;
}


/**
 * \brief 检测碰撞
 * \param body_a BodyA
 * \param body_b BodyB
 * \return 返回是否碰撞
 */
bool Intersect(const Body* body_a, const Body* body_b)
{
    const Vec3 distance_ab = body_b->m_position - body_a->m_position;
    const auto sphere_a = dynamic_cast<const ShapeSphere*>(body_a->m_shape);
    const auto sphere_b = dynamic_cast<const ShapeSphere*>(body_b->m_shape);

    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = distance_ab.GetLengthSqr();

    if (length_square <= (radius_ab * radius_ab))
        return true;
    return false;
}
