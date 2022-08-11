//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact(contact_t& contact)
{
    Body* body_a = contact.bodyA;
    Body* body_b = contact.bodyB;

    const float inv_mass_a = body_a->m_inverseMass;
    const float inv_mass_b = body_b->m_inverseMass;

    // 计算碰撞处的法线向量
    const Vec3& n = contact.normal;
    const Vec3 v_ab = body_a->m_linearVelocity - body_b->m_linearVelocity;
    const float impulse_j = -2.0f * v_ab.Dot(n) / (inv_mass_a + inv_mass_b); // v_ab.Dot(n)得到相对速度沿碰撞接触点法向的分量
    const Vec3 vector_impulse_j = n * impulse_j;

    body_a->ApplyImpulseLinear(vector_impulse_j * 1.0f);
    body_b->ApplyImpulseLinear(vector_impulse_j * -1.0f);

    // Distance project . 阻止相撞的两个物体相互穿透
    const float t_a = body_a->m_inverseMass / (body_a->m_inverseMass + body_b->m_inverseMass);
    const float t_b = body_b->m_inverseMass / (body_a->m_inverseMass + body_b->m_inverseMass);

    const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    body_a->m_position += ds * t_a;
    body_b->m_position -= ds * t_b;
}
