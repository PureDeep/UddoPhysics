//
//  GJK.cpp
//
#include "GJK.h"

/**
 * \brief 找到哪个轴对齐的平面能使单纯形（1D单纯形指线段）的投影面积或长度最大化
 * \param a 线段端点a
 * \param b 线段端点b
 * \return 
 */
Vec2 SignedVolume1D(const Vec3& a, const Vec3& b)
{
    Vec3 ab = b - a; // ab构成的线段
    Vec3 ap = Vec3(0.0f) - a; // 从a指向原点的直线
    Vec3 p0 = a + ab * ab.Dot(ap) / ab.GetLengthSqr(); // ap在投影在ab方向上的分量

    // 找出投影最大的轴
    int idx = 0;
    float mu_max = 0;
    for (int i = 0; i < 3; i++)
    {
        float mu = b[i] - a[i]; // 线段在轴上的投影长度
        if (mu * mu > mu_max * mu_max)
        {
            mu_max = mu;
            idx = i;
        }
    }

    // 
    const float a1 = a[idx];
    const float b1 = b[idx];
    const float p1 = p0[idx];

    // 算出a-p和p-b带符号的距离
    const float C1 = p1 - a1;
    const float C2 = b1 - p1;

    if ((p1 > a1 && p1 < b1) || (p1 > b1 && p1 < a1))
    {
        Vec2 lambdas;
        lambdas[0] = C2 / mu_max;
        lambdas[1] = C1 / mu_max;
        return lambdas;
    }

    if ((a1 <= b1 && p1 <= a1) || (a1 >= b1 && p1 >= a1))
    {
        return Vec2(1.0f, 0.0f);
    }

    return Vec2(0.0f, 1.0f);
}

int CompareSigns(const float& a, const float& b)
{
    if (a * b > 0.0f)
        return 1;
    return 0;
}

/**
 * \brief 
 * \param s1 三角形顶点a
 * \param s2 三角形顶点b
 * \param s3 三角形顶点c
 * \return 
 */
Vec3 SignedVolume2D(const Vec3& s1, const Vec3& s2, const Vec3& s3)
{
    // 求三角形法线
    Vec3 normal = (s2 - s1).Cross(s3 - s1);
    // 从原点指向s1的向量沿法线方向的分量
    Vec3 p0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

    // 找到最大投影面积对应的轴
    // x轴对应yz平面，y轴对应zx平面，z轴对应xy平面
    int idx = 0;
    int area_max = 0;
    for (int i = 0; i < 3; i++)
    {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        auto a = Vec2(s1[j], s1[k]);
        auto b = Vec2(s2[j], s2[k]);
        auto c = Vec2(s3[j], s3[j]);

        Vec2 ab = b - a;
        Vec2 ac = c - a;

        float area = ab.x * ac.y - ab.y * ac.x;
        if (area * area > area_max * area_max)
        {
            area_max = area;
            idx = i;
        }
    }

    // 投影到那个最大投影面最大的面上去
    int x = (idx + 1) % 3;
    int y = (idx + 2) % 3;
    Vec2 s[3];
    s[0] = Vec2(s1[x], s1[y]);
    s[1] = Vec2(s2[x], s2[y]);
    s[2] = Vec2(s3[x], s3[y]);

    // 从原点指向s1的向量沿法线方向的分量在投影面积最大面法线上的投影
    auto p = Vec2(p0[x], p0[y]);

    // 获得投影原点与边形成的子三角形的面积
    Vec3 areas;
    for (int i = 0; i < 3; i++)
    {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        Vec2 a = p;
        Vec2 b = s[j];
        Vec2 c = s[k];
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        areas[i] = ab.x * ac.y - ab.y * ac.x;
    }

    // 如果投影原点在三角形内，则返回这个点的重心坐标
    if (CompareSigns(area_max, areas[0]) > 0 && CompareSigns(area_max, areas[1]) > 0 && CompareSigns(area_max, areas[2])
        > 0)
    {
        Vec3 lambdas = areas / area_max;
        return lambdas;
    }

    // 投影到边上，并求出最近的点
    float dist = 1e10;
    auto lambdas = Vec3(1, 0, 0);
    for (int i = 0; i < 3; i++)
    {
        int k = (i + 1) % 3;
        int l = (i + 2) % 3;

        Vec3 edges_pts[3];
        edges_pts[0] = s1;
        edges_pts[1] = s2;
        edges_pts[2] = s3;

        Vec2 lambda_edge = SignedVolume1D(edges_pts[k], edges_pts[l]);
        Vec3 pt = edges_pts[k] * lambda_edge[0] + edges_pts[l] * lambda_edge[1];
        if (pt.GetLengthSqr() < dist)
        {
            dist = pt.GetLengthSqr();
            lambdas[i] = 0;
            lambdas[k] = lambda_edge[0];
            lambdas[l] = lambda_edge[1];
        }
    }

    return lambdas;
}

/**
 * \brief 确定原点是否在四面体内，如果是，返回这个点的重心坐标。如果不是，则检查每个面与哪个投影最近
 * \param s1 四面体面1
 * \param s2 四面体面2
 * \param s3 四面体面3
 * \param s4 四面体面4
 * \return 原点重心坐标或离原点最近的面
 */
Vec4 SignedVolume3D(const Vec3& s1, const Vec3& s2, const Vec3& s3, const Vec3& s4)
{
    Mat4 M;
    M.rows[0] = Vec4(s1.x, s2.x, s3.x, s4.x);
    M.rows[1] = Vec4(s1.y, s2.y, s3.y, s4.y);
    M.rows[2] = Vec4(s1.z, s2.z, s3.z, s4.z);
    M.rows[3] = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

    Vec4 c4;
    c4[0] = M.Cofactor(3, 0);
    c4[1] = M.Cofactor(3, 1);
    c4[2] = M.Cofactor(3, 2);
    c4[3] = M.Cofactor(3, 3);

    const float det_m = c4[0] + c4[1] + c4[2] + c4[3];

    // 如果原点的重心坐标处于单纯形内，则返回其重心坐标
    if (CompareSigns(det_m, c4[0]) > 0 && CompareSigns(det_m, c4[1]) > 0 && CompareSigns(det_m, c4[2]) > 0 &&
        CompareSigns(det_m, c4[3]) > 0)
    {
        Vec4 lambdas = c4 * (1.0f / det_m);
        return lambdas;
    }

    // 如果原点不在单纯形内，则找到距离它最近的面
    Vec4 lambdas;
    float dist = 1e10;
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        int k = (i + 2) % 4;

        Vec3 face_pts[4];
        face_pts[0] = s1;
        face_pts[1] = s2;
        face_pts[2] = s3;
        face_pts[3] = s4;

        Vec3 lambdas_face = SignedVolume2D(face_pts[i], face_pts[j], face_pts[k]);
        Vec3 pt = face_pts[i] * lambdas_face[0] + face_pts[j] * lambdas_face[1] + face_pts[k] * lambdas_face[2];

        if (pt.GetLengthSqr() < dist)
        {
            dist = pt.GetLengthSqr();
            lambdas.Zero();
            lambdas[i] = lambdas_face[0];
            lambdas[j] = lambdas_face[1];
            lambdas[k] = lambdas_face[2];
        }
    }

    return lambdas;
}

/**
 * \brief 测试：将一个点投影到单纯形，并返回其中心坐标
 */
void TestSignedVolumeProjection()
{
    const Vec3 orgPts[4] = {
        Vec3(0, 0, 0),
        Vec3(1, 0, 0),
        Vec3(0, 1, 0),
        Vec3(0, 0, 1),
    };
    Vec3 pts[4];
    Vec4 lambdas;
    Vec3 v;

    for (int i = 0; i < 4; i++)
    {
        pts[i] = orgPts[i] + Vec3(1, 1, 1);
    }
    lambdas = SignedVolume3D(pts[0], pts[1], pts[2], pts[3]);
    v.Zero();
    for (int i = 0; i < 4; i++)
    {
        v += pts[i] * lambdas[i];
    }
    printf("lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
           lambdas.x, lambdas.y, lambdas.z, lambdas.w,
           v.x, v.y, v.z
    );

    for (int i = 0; i < 4; i++)
    {
        pts[i] = orgPts[i] + Vec3(-1, -1, -1) * 0.25f;
    }
    lambdas = SignedVolume3D(pts[0], pts[1], pts[2], pts[3]);
    v.Zero();
    for (int i = 0; i < 4; i++)
    {
        v += pts[i] * lambdas[i];
    }
    printf("lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
           lambdas.x, lambdas.y, lambdas.z, lambdas.w,
           v.x, v.y, v.z
    );

    for (int i = 0; i < 4; i++)
    {
        pts[i] = orgPts[i] + Vec3(-1, -1, -1);
    }
    lambdas = SignedVolume3D(pts[0], pts[1], pts[2], pts[3]);
    v.Zero();
    for (int i = 0; i < 4; i++)
    {
        v += pts[i] * lambdas[i];
    }
    printf("lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
           lambdas.x, lambdas.y, lambdas.z, lambdas.w,
           v.x, v.y, v.z
    );

    for (int i = 0; i < 4; i++)
    {
        pts[i] = orgPts[i] + Vec3(1, 1, -0.5f);
    }
    lambdas = SignedVolume3D(pts[0], pts[1], pts[2], pts[3]);
    v.Zero();
    for (int i = 0; i < 4; i++)
    {
        v += pts[i] * lambdas[i];
    }
    printf("lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
           lambdas.x, lambdas.y, lambdas.z, lambdas.w,
           v.x, v.y, v.z
    );

    pts[0] = Vec3(51.1996613f, 26.1989613f, 1.91339576f);
    pts[1] = Vec3(-51.0567360f, -26.0565681f, -0.436143428f);
    pts[2] = Vec3(50.8978920f, -24.1035538f, -1.04042661f);
    pts[3] = Vec3(-49.1021080f, 25.8964462f, -1.04042661f);
    lambdas = SignedVolume3D(pts[0], pts[1], pts[2], pts[3]);
    v.Zero();
    for (int i = 0; i < 4; i++)
    {
        v += pts[i] * lambdas[i];
    }
    printf("lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
           lambdas.x, lambdas.y, lambdas.z, lambdas.w,
           v.x, v.y, v.z
    );
}

struct point_t
{
    Vec3 xyz; // 在Minkowski Sum中的点
    Vec3 pt_a; // BodyA上的点
    Vec3 pt_b; // BodyB上的点

    point_t() : xyz(Vec3(0.0f)), pt_a(Vec3(0.0f)), pt_b(Vec3(0.0f))
    {
    }

    const point_t& operator =(const point_t& rhs)
    {
        xyz = rhs.xyz;
        pt_a = rhs.pt_a;
        pt_b = rhs.pt_b;
        return *this;
    }

    bool operator ==(const point_t& rhs)
    {
        if (xyz == rhs.xyz && pt_a == rhs.pt_a && pt_b == rhs.pt_b)
        {
            return true;
        }
        return false;
    }
};

point_t Support(Body* body_a, Body* body_b, Vec3 dir, const float bias)
{
    point_t point;

    // dir方向上最远的点
    point.pt_a = body_a->m_shape->Support(dir, body_a->m_position, body_a->m_orientation, bias);
    dir = dir * -1.0f;
    // 反方向上最远的点
    point.pt_b = body_b->m_shape->Support(dir, body_b->m_position, body_b->m_orientation, bias);
    // Minkowski Sum中dir方向上最远的点
    point.xyz = point.pt_a - point.pt_b;

    return point;
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect(const Body* bodyA, const Body* bodyB)
{
    // TODO: Add code

    return false;
}

/*
================================
GJK_ClosestPoints
================================
*/
void GJK_ClosestPoints(const Body* bodyA, const Body* bodyB, Vec3& ptOnA, Vec3& ptOnB)
{
    // TODO: Add code
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect(const Body* bodyA, const Body* bodyB, const float bias, Vec3& ptOnA, Vec3& ptOnB)
{
    // TODO: Add code

    return false;
}
