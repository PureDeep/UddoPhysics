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
}
