//
//  Shapes.cpp
//
#include "ShapeBox.h"

/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
生成Box
====================================================
*/
void ShapeBox::Build(const Vec3* pts, const int num)
{
    for (int i = 0; i < num; i++)
    {
        m_bounds.Expand(pts[i]);
    }

    m_points.clear();

    // 利用Bounds中两个点的数据生成 形成Box需要的8个顶点坐标数据
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z));

    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z));

    // Box的中心就是盒体数据的中间点
    m_centerOfMass = (m_bounds.maxs + m_bounds.mins) * 0.5f;
}

/*
====================================================
ShapeBox::Support
====================================================
*/
Vec3 ShapeBox::Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const
{
    // 先找出在dir方向上最远的点
    Vec3 max_pt = orient.RotatePoint(m_points[0]) + pos;
    float max_dist = dir.Dot(max_pt);
    for (int i = 1; i < m_points.size(); i++)
    {
        const Vec3 pt = orient.RotatePoint(m_points[i]) + pos;
        const float dist = dir.Dot(pt);

        if (dist > max_dist)
        {
            max_dist = dist;
            max_pt = pt;
        }
    }

    Vec3 norm = dir;
    norm.Normalize();
    norm *= bias;

    return max_pt + norm;
}

/*
====================================================
ShapeBox::InertiaTensor
Box惯性张量
====================================================
*/
Mat3 ShapeBox::InertiaTensor() const
{
    const float dx = m_bounds.maxs.x - m_bounds.mins.x;
    const float dy = m_bounds.maxs.y - m_bounds.mins.y;
    const float dz = m_bounds.maxs.z - m_bounds.mins.z;

    // 生成默认中心在Box中心的惯性张量，用本地坐标
    Mat3 tensor;
    tensor.Zero();
    tensor.rows[0][0] = (dy * dy + dz * dz) / 12.0f;
    tensor.rows[1][1] = (dx * dx + dz * dz) / 12.0f;
    tensor.rows[2][2] = (dx * dx + dy * dy) / 12.0f;

    // 求质点坐标
    Vec3 cm;
    cm.x = (m_bounds.maxs.x + m_bounds.mins.x);
    cm.y = (m_bounds.maxs.y + m_bounds.mins.y);
    cm.z = (m_bounds.maxs.z + m_bounds.mins.z);

    // 质点到原点的偏移向量
    const Vec3 R = Vec3(0, 0, 0) - cm;
    const float R2 = R.GetLengthSqr();

    Mat3 pat_tensor;
    pat_tensor.rows[0] = Vec3(R2 - R.x * R.x, R.x * R.y, R.x * R.z);
    pat_tensor.rows[1] = Vec3(R.y * R.x, R2 - R.y * R.y, R.y * R.z);
    pat_tensor.rows[2] = Vec3(R.z * R.x, R.z * R.y, R2 - R.z * R.z);

    // 使用平行轴定理
    tensor += pat_tensor;

    return tensor;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds(const Vec3& pos, const Quat& orient) const
{
    // Box的8个角，和Build中一致
    Vec3 corners[8];
    corners[0] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
    corners[1] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
    corners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
    corners[3] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);

    corners[4] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
    corners[5] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
    corners[6] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
    corners[7] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);

    Bounds bounds;
    for (int i = 0; i < 8; i++)
    {
        corners[i] = orient.RotatePoint(corners[i]) + pos;
        bounds.Expand(corners[i]);
    }

    return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
它接收一个方向和物体的角速度，并将在该方向上移动最快的顶点的速度返回给我们。
返回给我们的是在该方向上移动最快的顶点的速度
====================================================
*/
float ShapeBox::FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const
{
    float max_speed = 0.0f;

    for (int i = 0; i < m_points.size(); i++)
    {
        Vec3 r = m_points[i] - m_centerOfMass; // 从质心指向顶点的向量
        Vec3 linear_velocity = angularVelocity.Cross(r); // 求出由于旋转而在该顶点产生的线速度
        float speed = dir.Dot(linear_velocity); // 求出线速度在指定方向dir上的分量
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }

    return max_speed;
}
