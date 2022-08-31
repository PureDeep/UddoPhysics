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

    const float elasticity_a = body_a->m_elasticity;
    const float elasticity_b = body_b->m_elasticity;
    const float elasticity = elasticity_a * elasticity_b;

    const Mat3 inv_inertia_world_a = body_a->GetInverseInertiaTensorWorldSpace();
    const Mat3 inv_inertia_world_b = body_b->GetInverseInertiaTensorWorldSpace();

    // const Vec3 pt_a = contact.ptOnA_WorldSpace;
    // const Vec3 pt_b = contact.ptOnB_WorldSpace;

    const Vec3 pt_a = body_a->BodySpaceToWorldSpace(contact.ptOnA_LocalSpace);
    const Vec3 pt_b = body_b->BodySpaceToWorldSpace(contact.ptOnB_LocalSpace);

    // 计算碰撞处的法线向量
    const Vec3& n = contact.normal;

    const Vec3 ra = pt_a - body_a->GetCenterOfMassWorldSpace();
    const Vec3 rb = pt_b - body_b->GetCenterOfMassWorldSpace();

    const Vec3 angular_j_a = (inv_inertia_world_a * ra.Cross(n)).Cross(ra);
    const Vec3 angular_j_b = (inv_inertia_world_b * rb.Cross(n)).Cross(rb);
    const float angular_factor = (angular_j_a + angular_j_b).Dot(n);

    const Vec3 vel_a = body_a->m_linearVelocity + body_a->m_angularVelocity.Cross(ra);
    const Vec3 vel_b = body_b->m_linearVelocity + body_b->m_angularVelocity.Cross(rb);

    // 速度差（相对速度）
    const Vec3 v_ab = vel_a - vel_b;

    // 计算碰撞产生的冲量
    // v_ab.Dot(n)得到相对速度沿碰撞接触点法向的分量
    const float impulse_j = -(1.0f + elasticity) * v_ab.Dot(n) / (inv_mass_a + inv_mass_b + angular_factor);
    const Vec3 vector_impulse_j = n * impulse_j;

    body_a->ApplyImpulse(pt_a, vector_impulse_j * 1.0f);
    body_b->ApplyImpulse(pt_b, vector_impulse_j * -1.0f);

    // 计算摩擦力产生的冲量
    const float friction_a = body_a->m_friction;
    const float friction_b = body_b->m_friction;
    const float friction = friction_a * friction_b;

    // 速度差在碰撞点所处表面的法线分量
    const Vec3 vel_norm = n * n.Dot(v_ab);

    // 速度差在碰撞点所处表面的切线分量
    const Vec3 vel_tang = v_ab - vel_norm;

    // 获得相对于另一个物体的切向速度分量
    Vec3 relative_vel_tang = vel_tang;
    relative_vel_tang.Normalize();

    const Vec3 inertia_a = (inv_inertia_world_a * ra.Cross(relative_vel_tang)).Cross(ra);
    const Vec3 inertia_b = (inv_inertia_world_b * rb.Cross(relative_vel_tang)).Cross(rb);
    const float inv_inertia = (inertia_a + inertia_b).Dot(relative_vel_tang);

    // 计算摩擦力产生的切向冲量
    const float reduced_mass = 1.0f / (body_a->m_inverseMass + body_b->m_inverseMass + inv_inertia);
    const Vec3 impulse_friction = vel_tang * reduced_mass * friction;

    // 施加摩擦力产生的冲量
    body_a->ApplyImpulse(pt_a, impulse_friction * -1.0f);
    body_b->ApplyImpulse(pt_b, impulse_friction * 1.0f);

    // Distance project . 阻止相撞的两个物体相互穿透
    if (contact.timeOfImpact == 0.0f)
    {
        const Vec3 ds = pt_b - pt_a;

        const float t_a = inv_mass_a / (inv_mass_a + inv_mass_b);
        const float t_b = inv_mass_b / (inv_mass_a + inv_mass_b);

        body_a->m_position += ds * t_a;
        body_b->m_position -= ds * t_b;
    }
}
