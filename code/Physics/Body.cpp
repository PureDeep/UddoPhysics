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

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
    if (m_inverseMass == 0.0f)
    {
        return;
    }

    // 先计算冲量对线速度的影响
    ApplyImpulseLinear(impulse);

    // 再计算冲量对角速度的影响
    const Vec3& center_of_mass = GetCenterOfMassWorldSpace();
    const Vec3& dL = impulsePoint - center_of_mass;
    ApplyImpulseAngular(dL.Cross(impulse));
}

/**
 * \brief 计算冲量对body线速度的影响
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

/**
 * \brief 计算冲量对body角速度的影响
 * \param impulse 作用在body上的冲量
 */
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

void Body::Update(float dt_sec)
{
    // 更新线速度
    m_position += m_linearVelocity * dt_sec;

    // 更新角速度
    const Vec3 position_cm = GetCenterOfMassWorldSpace();
    Vec3 cm_to_pos = m_position - position_cm;

    // Total Torque = External Applied Torque + Internal Torque
    // T = T_external + omega x I * omega
    // T_external = 0 because it was applied in the collision response function
    // T = Ia = w x I * w
    // a = I^-1 (w x I * w)
    const Mat3 orientation = m_orientation.ToMat3();
    Mat3 inertia_tensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
    Vec3 alpha = inertia_tensor.Inverse() * (m_angularVelocity.Cross(inertia_tensor * m_angularVelocity));
    m_angularVelocity += alpha * dt_sec;

    Vec3 d_angle = m_angularVelocity * dt_sec;
    auto d_q = Quat(d_angle, d_angle.GetMagnitude());
    m_orientation = d_q * m_orientation;
    m_orientation.Normalize();

    m_position = position_cm + d_q.RotatePoint(cm_to_pos);
}
