//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
    m_position(0.0f),
    m_orientation(0.0f, 0.0f, 0.0f, 1.0f),
    m_linearVelocity(0.0f),
    m_inverseMass(1.0f),
    m_shape(
        nullptr
    )
{
}

/**
 * \brief 获取质心在世界坐标系中的值
 * \return 质心在世界坐标系中的值
 */
Vec3 Body::GetCenterOfMassWorldSpace() const
{
    const Vec3 center_of_mass = m_shape->GetCenterOfMass();
    const Vec3 pos = m_position + m_orientation.RotatePoint(center_of_mass);
    return center_of_mass;
}

/**
 * \brief 获取质心在本地坐标系中的值
 * \return 质心在本地坐标系中的值
 */
Vec3 Body::GetCenterOfMassModelSpace() const
{
    const Vec3 center_of_mass = m_shape->GetCenterOfMass();
    return center_of_mass;
}

/**
 * \brief 将给定世界坐标系中的点转换为本地坐标系下的值
 * \param pt 世界坐标系下的坐标值
 * \return 转换后在本地坐标系下的坐标值
 */
Vec3 Body::WorldSpaceToBodySpace(const Vec3& pt) const
{
    Vec3 tmp = pt - GetCenterOfMassWorldSpace(); // 生成一个世界坐标系下的质心指向形参的零时变量tmp
    const Quat inverse_orient = m_orientation.Inverse(); // 获得反向旋转
    Vec3 body_space = inverse_orient.RotatePoint(tmp);
    return body_space;
}

/**
 * \brief 将给定本地坐标系下的坐标转换为世界坐标系下的坐标
 * \param pt 本地坐标系下的坐标
 * \return 转换后在世界坐标系下的坐标值
 */
Vec3 Body::BodySpaceToWorldSpace(const Vec3& pt) const
{
    Vec3 world_space = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(pt);
    return world_space;
}

/**
 * \brief 获得本地坐标系下的反惯性张量
 * \return 本地坐标系下的反惯性张量
 */
Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
    Mat3 inertia_tensor = m_shape->InertiaTensor();
    Mat3 inv_inertia_tensor = inertia_tensor.Inverse() * m_inverseMass;
    return inv_inertia_tensor;
}

/**
 * \brief 获得世界坐标系下的反惯性张量
 * \return 世界坐标系下的反惯性张量
 */
Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
    Mat3 inertia_tensor = m_shape->InertiaTensor();
    Mat3 inv_inertia_tensor = inertia_tensor.Inverse() * m_inverseMass;
    Mat3 orient = m_orientation.ToMat3();
    inv_inertia_tensor = orient * inv_inertia_tensor * orient.Transpose();
    return inv_inertia_tensor;
}

/**
 * \brief 计算冲量对body速度的影响
 * \param impulse 作用在body上的冲量
 */
void Body::ApplyImpulseLinear(const Vec3& impulse)
{
    if (m_inverseMass == 0.0f)
        return;

    // J : the change of momentum, 动量的改变量
    // p = mv
    // dp = F * t
    // J = m * dv
    // => dv = j / m

    m_linearVelocity += impulse * m_inverseMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
    if (m_inverseMass == 0.0f)
    {
        return;
    }

    // L = I w = r x p
    // dL = I dw = r x J
    // => dw = I^-1 * ( r x J )

    m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

    // 30 rad/s is fast enough.
    const float maxAngularSpeed = 30.0f;
    if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
    {
        m_angularVelocity.Normalize();
        m_angularVelocity *= maxAngularSpeed;
    }
}
