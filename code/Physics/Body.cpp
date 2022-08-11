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
    m_shape(nullptr)
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
