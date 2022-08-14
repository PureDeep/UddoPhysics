//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"

/**
 * \brief 寻找在指定方向上分量长度最远的点
 * \param pts m_points数组
 * \param num m_points数组元素数量
 * \param dir 指定方向
 * \return 指定方向上分量长度最远的点
 */
int FindPointFurthestInDir(const Vec3* pts, int num, const Vec3& dir)
{
    int max_idx = 0;
    float max_dist = dir.Dot(pts[0]); // 第一个点在dir方向上分量的值，最为最远距离的初始值
    for (int i = 1; i < num; i++)
    {
        float dist = dir.Dot(pts[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
            max_idx = i;
        }
    }
    return max_idx;
}

/**
 * \brief 求指定点pt到由a，b两点确定的直线的距离
 * \param a 确定直线的一点
 * \param b 确定直线的另一点
 * \param pt 指点点
 * \return 点到直线的距离
 */
float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt)
{
    Vec3 ab = b - a; // b - a 生成直线向量
    ab.Normalize();

    Vec3 ray = pt - a;
    Vec3 projection = ab * ray.Dot(ab); // 求得ray沿ab方向的向量
    Vec3 perpindicular = ray - projection; // 由pt向直线ab做垂线产生的向量
    return perpindicular.GetMagnitude();
}

/**
 * \brief 求m_points中离直线ab最远的一点
 * \param pts m_points数组
 * \param num m_points数组元素数量
 * \param pt_a 直线上一点
 * \param pt_b 直线上另一点
 * \return m_points中离直线ab最远的一点
 */
Vec3 FindPointFurthestFromLine(const Vec3* pts, const int num, const Vec3& pt_a, const Vec3& pt_b)
{
    int max_id = 0;
    float max_dist = DistanceFromLine(pt_a, pt_b, pts[0]);
    for (int i = 1; i < num; i++)
    {
        float dist = DistanceFromLine(pt_a, pt_b, pts[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
            max_id = i;
        }
    }

    return pts[max_id];
}

/**
 * \brief 求点到三角形的距离
 * \param a 三角形顶点a
 * \param b 三角形顶点b
 * \param c 三角形顶点c
 * \param pt 指定点
 * \return 点到三角形的距离
 */
float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt)
{
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 normal = ab.Cross(ac);
    normal.Normalize(); // 垂直与三角形的方向向量

    Vec3 ray = pt - a;
    float dist = ray.Dot(normal);
    return dist;
}

/**
 * \brief 求m_points中离三角形abc最远的一点
 * \param pts m_points数组
 * \param num m_points数组元素数量
 * \param pt_a 三角形顶点a
 * \param pt_b 三角形顶点b
 * \param pt_c 三角形顶点c
 * \return m_points中离三角形abc最远的一点
 */
Vec3 FindPointFurthestFromTriangle(const Vec3* pts, const int num, const Vec3& pt_a, const Vec3& pt_b, const Vec3& pt_c)
{
    int max_id = 0;
    float max_dist = DistanceFromTriangle(pt_a, pt_b, pt_c, pts[0]);
    for (int i = 1; i < num; i++)
    {
        float dist = DistanceFromTriangle(pt_a, pt_b, pt_c, pts[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
            max_id = i;
        }
    }

    return pts[max_id];
}

/**
 * \brief 构建四面体
 * \param verts 顶点数组
 * \param num 顶点数组元素数量
 * \param hull_pts 四面体顶点数组
 * \param hull_tris 四面体三角形数组
 */
void BuildTetrahedron(const Vec3* verts, int num, std::vector<Vec3>& hull_pts, std::vector<tri_t>& hull_tris)
{
    hull_pts.clear();
    hull_tris.clear();

    Vec3 points[4];

    // 第一步，先找到指定方向上距离最远的一个点
    // Step1 : Find a point that is the furthest in a particular direction (later we will call this a support point)
    int idx = FindPointFurthestInDir(verts, num, Vec3(1, 0, 0));
    points[0] = verts[idx];
    // 第二步，在第一步中找出的点的反方向上找一个距离最远的点
    // Step2 : Find another point that is furthest in the opposite direction of the point from step 1
    idx = FindPointFurthestInDir(verts, num, points[0] * -1.0f);
    points[1] = verts[idx];
    // 第三步，找出与第一二步中两点形成的直线的投影距离最远的点
    // Step3 : Find a third point that is furthest from the axis of the points formed from 1 & 2
    points[2] = FindPointFurthestFromLine(verts, num, points[0], points[1]);
    // 第四步，找出与第一二三步中形成的三角形的距离最远的点
    // Step4 : Then we find the point that is furthest from the plane formed from the previous points
    points[3] = FindPointFurthestFromTriangle(verts, num, points[0], points[1], points[2]);
    // 第五步，连接4个顶点
    // Step5 : Finally, we build the “connections” of those points. But really, we’re just recording triangle indices.
    float dist = DistanceFromTriangle(points[0], points[1], points[2], points[3]);
    if (dist > 0.0f)
    {
        std::swap(points[0], points[1]);
    }

    // 构建四面体
    hull_pts.push_back(points[0]);
    hull_pts.push_back(points[1]);
    hull_pts.push_back(points[2]);
    hull_pts.push_back(points[3]);

    // 生成四个面的三角形
    // 我们需要确保四面体的每个面都是按逆时针方向排列的（CCW， counter-clockwise order）
    // 因为我们要用它来计算每个面的法线，而我们的凸面图形有向外的法线
    tri_t tri;
    tri.a = 0;
    tri.b = 1;
    tri.c = 2;
    hull_tris.push_back(tri);

    tri.a = 0;
    tri.b = 2;
    tri.c = 3;
    hull_tris.push_back(tri);

    tri.a = 2;
    tri.b = 1;
    tri.c = 3;
    hull_tris.push_back(tri);

    tri.a = 1;
    tri.b = 0;
    tri.c = 3;
    hull_tris.push_back(tri);
}

/*
========================================================================================================

ShapeConvex

========================================================================================================
*/

void BuildConvexHull(const std::vector<Vec3>& verts, std::vector<Vec3>& hullPts, std::vector<tri_t>& hullTris)
{
    // TODO: Add code
}

/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build(const Vec3* pts, const int num)
{
    // TODO: Add code
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const
{
    Vec3 supportPt;

    // TODO: Add code

    return supportPt;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds(const Vec3& pos, const Quat& orient) const
{
    // 凸包Bounds的8个角，和Build中一致
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
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const
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
