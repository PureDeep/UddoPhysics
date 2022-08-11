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

    body_a->m_linearVelocity.Zero();
    body_b->m_linearVelocity.Zero();

    // Distance project . 阻止相撞的两个物体相互穿透
    const float t_a = body_a->m_inverseMass / (body_a->m_inverseMass + body_b->m_inverseMass);
    const float t_b = body_b->m_inverseMass / (body_a->m_inverseMass + body_b->m_inverseMass);

    const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    body_a->m_position += ds * t_a;
    body_b->m_position -= ds * t_b;
}
