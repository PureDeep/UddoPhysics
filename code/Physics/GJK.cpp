//
//  GJK.cpp
//
#include "GJK.h"

#include <windows.h>

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

point_t Support(const Body* body_a, const Body* body_b, Vec3 dir, const float bias)
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

/**
 * \brief 将原点投影到单纯形上以获得新的搜索方向，并检查原点是否在单纯形内
 * \param pts 
 * \param num 
 * \param new_dir 
 * \param lambdas_out 
 * \return 点是否在单纯形内
 */
bool SimplexSignedVolumes(point_t* pts, const int num, Vec3& new_dir, Vec4& lambdas_out)
{
    const float epsilon_f = 0.0001f * 0.0001f;

    lambdas_out.Zero();

    bool does_intersect = false;

    switch (num)
    {
    default:
    case 2:
        {
            Vec2 lambdas = SignedVolume1D(pts[0].xyz, pts[1].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 2; i++)
            {
                v += pts[i].xyz * lambdas[i];
            }

            new_dir = v * -1.0f;
            does_intersect = (v.GetLengthSqr() < epsilon_f);
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
        }
        break;
    case 3:
        {
            Vec3 lambdas = SignedVolume2D(pts[0].xyz, pts[1].xyz, pts[2].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 3; i++)
            {
                v += pts[i].xyz * lambdas[i];
            }

            new_dir = v * -1.0f;
            does_intersect = (v.GetLengthSqr() < epsilon_f);
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
            lambdas_out[2] = lambdas[2];
        }
        break;
    case 4:
        {
            Vec4 lambdas = SignedVolume3D(pts[0].xyz, pts[1].xyz, pts[2].xyz, pts[3].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 4; i++)
            {
                v += pts[i].xyz * lambdas[i];
            }

            new_dir = v * -1.0f;
            does_intersect = (v.GetLengthSqr() < epsilon_f);
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
            lambdas_out[2] = lambdas[2];
            lambdas_out[3] = lambdas[3];
        }
        break;
    }

    return does_intersect;
}

/**
 * \brief 检查点是否已经在单纯形内部
 * \param simplex_points 
 * \param new_pt 
 * \return 
 */
bool HasPoint(const point_t simplex_points[4], const point_t& new_pt)
{
    const float precision = 1e-6f;

    for (int i = 0; i < 4; i++)
    {
        Vec3 delta = simplex_points[i].xyz - new_pt.xyz;
        if (delta.GetLengthSqr() < precision * precision)
        {
            return true;
        }
    }
    return false;
}

/**
 * \brief 将有效的支持点排序到数组
 * \param simplex_points 
 * \param lambdas 
 */
void SortValids(point_t simplex_points[4], Vec4& lambdas)
{
    bool valids[4];
    for (int i = 0; i < 4; i++)
    {
        valids[i] = true;
        if (lambdas[i] == 0.0f)
        {
            valids[i] = false;
        }
    }

    Vec4 valid_lambdas(0.0f);
    int valid_count = 0;
    point_t valid_pts[4];
    memset(valid_pts, 0, sizeof(point_t) * 4);
    for (int i = 0; i < 4; i++)
    {
        if (valids[i])
        {
            valid_pts[valid_count] = simplex_points[i];
            valid_lambdas[valid_count] = lambdas[i];
            valid_count++;
        }
    }

    // 将有效的支持点再拷贝回simplex_points
    for (int i = 0; i < 4; i++)
    {
        simplex_points[i] = valid_pts[i];
        lambdas[i] = valid_lambdas[i];
    }
}

static int NumValids(const Vec4& lambdas)
{
    int num = 0;

    for (int i = 0; i < 4; i++)
    {
        if (lambdas[i] != 0.0f)
        {
            num++;
        }
    }

    return num;
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect(const Body* bodyA, const Body* bodyB)
{
    const Vec3 origin(0.0f);

    int num_pts = 1;
    point_t simplex_points[4];
    // 随机找一个support点
    simplex_points[0] = Support(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

    float closest_dist = 1e10f;
    bool does_contain_origin = false;
    // 选择一个新的搜索方向
    Vec3 new_dir = simplex_points[0].xyz * -1.0f;

    do
    {
        // 获得反向的新support点
        point_t new_pt = Support(bodyA, bodyB, new_dir, 0.0f);

        // 如果新点与原来的点一样，则不能再扩张了，中断寻找（如果已经有此support点了，则返回false）
        if (HasPoint(simplex_points, new_pt))
        {
            break;
        }

        simplex_points[num_pts] = new_pt;
        num_pts++;

        // 如果这个点和原点在同一边，则没有发生碰撞
        float dot_dot = new_dir.Dot(new_pt.xyz - origin);
        if (dot_dot < 0.0f)
        {
            break;
        }

        // 检查原点是否在单纯形内，如果是，返回true，说明发生了碰撞
        Vec4 lambdas;
        does_contain_origin = SimplexSignedVolumes(simplex_points, num_pts, new_dir, lambdas);
        if (does_contain_origin)
        {
            break;
        }

        // 检查原点在单纯形上的投影是否比之前的投影更近，如果不靠近原点，则不相交，返回false
        float dist = new_dir.GetLengthSqr();
        if (dist >= closest_dist)
        {
            break;
        }
        closest_dist = dist;

        //从单纯形上移除任何不支持原点投影到单纯形上的点
        SortValids(simplex_points, lambdas);
        // 如果有四个单纯形点，则有一个交点
        num_pts = NumValids(lambdas);
        does_contain_origin = (num_pts == 4);
    }
    while (!does_contain_origin);

    return does_contain_origin;
}

Vec3 BarycentricCoordinates(Vec3 s1, Vec3 s2, Vec3 s3, const Vec3& pt)
{
    s1 = s1 - pt;
    s2 = s2 - pt;
    s3 = s3 - pt;

    Vec3 normal = (s2 - s1).Cross(s3 - s1);
    Vec3 p0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

    //
    int idx = 0;
    float area_max = 0;
    for (int i = 0; i < 3; i++)
    {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        auto a = Vec2(s1[j], s1[k]);
        auto b = Vec2(s2[j], s2[k]);
        auto c = Vec2(s3[j], s3[k]);

        Vec2 ab = b - a;
        Vec2 ac = c - a;

        float area = ab.x * ac.y - ab.y * ac.x;
        if (area * area > area_max * area_max)
        {
            idx = i;
            area_max = area;
        }
    }

    // 投影到合适的轴
    int x = (idx + 1) % 3;
    int y = (idx + 2) % 3;
    Vec2 s[3]; // 三个顶点在某个面上的坐标
    s[0] = Vec2(s1[x], s1[y]);
    s[1] = Vec2(s2[x], s2[y]);
    s[2] = Vec2(s3[x], s3[y]);

    auto p = Vec2(p0[x], p0[y]);

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

    Vec3 lambdas = areas / area_max;
    if (!lambdas.IsValid())
    {
        lambdas = Vec3(1, 0, 0);
    }

    return lambdas;
}

/**
 * \brief 求三角形法线向量
 * \param tri 三角形tri
 * \param points 顶点数组
 * \return 三角形法线向量
 */
Vec3 NormalDirection(const tri_t& tri, const std::vector<point_t>& points)
{
    const Vec3& a = points[tri.a].xyz;
    const Vec3& b = points[tri.b].xyz;
    const Vec3& c = points[tri.c].xyz;

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 normal = ab.Cross(ac);
    normal.Normalize();
    return normal;
}

/**
 * \brief 求点到三角形的带符号距离
 * \param tri 三角形顶点数据
 * \param pt 指定点
 * \param points 顶点数组
 * \return 指定点到三角形所在面的距离
 */
float SignedDistanceToTriangle(const tri_t& tri, const Vec3& pt, const std::vector<point_t>& points)
{
    const Vec3 normal = NormalDirection(tri, points);
    const Vec3& a = points[tri.a].xyz;
    const Vec3 a_to_pt = pt - a;
    const float dist = normal.Dot(a_to_pt);
    return dist;
}

/**
 * \brief 找到距离原点最近的三角形
 * \param triangles 三角形数组
 * \param points 顶点数组
 * \return 距离原点最近的三角形的索引
 */
int ClosestTriangle(const std::vector<tri_t>& triangles, const std::vector<point_t>& points)
{
    float min_dist_sqr = 1e10;

    int idx = -1;

    for (int i = 0; i < triangles.size(); i++)
    {
        const tri_t& tri = triangles[i];

        float dist = SignedDistanceToTriangle(tri, Vec3(0.0f), points);
        float dist_sqr = dist * dist;
        if (dist_sqr < min_dist_sqr * min_dist_sqr)
        {
            idx = i;
            min_dist_sqr = dist_sqr;
        }
    }

    return idx;
}

/**
 * \brief 检查指定点是否在Minkowisk差中
 * \param pt 指定点
 * \param triangles 三角形数组
 * \param points 顶点数组
 * \return 点是否在Minkowisk差中
 */
bool HasPoint(const Vec3& pt, const std::vector<tri_t>& triangles, const std::vector<point_t>& points)
{
    float epsilon = 0.0001f * 0.0001f;
    Vec3 delta;

    for (int i = 0; i < triangles.size(); i++)
    {
        const tri_t& tri = triangles[i];

        delta = pt - points[tri.a].xyz;
        if (delta.GetLengthSqr() < epsilon)
        {
            return true;
        }

        delta = pt - points[tri.b].xyz;
        if (delta.GetLengthSqr() < epsilon)
        {
            return true;
        }

        delta = pt - points[tri.c].xyz;
        if (delta.GetLengthSqr() < epsilon)
        {
            return true;
        }
    }
    return false;
}

/**
 * \brief 移除所有面向该点的三角形
 * \param pt 给定点
 * \param triangles 三角形数组
 * \param points 顶点数组
 * \return 被移除的三角形的数量
 */
int RemoveTrianglesFacingPoint(const Vec3& pt, std::vector<tri_t>& triangles, const std::vector<point_t>& points)
{
    int num_removed = 0;
    for (int i = 0; i < triangles.size(); i++)
    {
        const tri_t& tri = triangles[i];

        float dist = SignedDistanceToTriangle(tri, pt, points);
        if (dist > 0.0f)
        {
            // dist > 0 表示三角形面向该点，则删除该三角形
            triangles.erase(triangles.begin() + i);
            i--;
            num_removed ++;
        }
    }

    return num_removed;
}

/**
 * \brief 找到悬空的边
 * \param dangling_edges 存放找到的悬空的边的数组
 * \param triangles 三角形数组
 */
void FindDanglingEdges(std::vector<edge_t>& dangling_edges, const std::vector<tri_t>& triangles)
{
    dangling_edges.clear();

    for (int i = 0; i < triangles.size(); i++)
    {
        const tri_t& tri = triangles[i];

        edge_t edges[3];
        edges[0].a = tri.a;
        edges[0].b = tri.b;

        edges[1].a = tri.b;
        edges[1].b = tri.c;

        edges[2].a = tri.c;
        edges[2].b = tri.a;

        // counts中存储每条边与其他三角形公用的次数
        int counts[3];
        counts[0] = 0;
        counts[1] = 0;
        counts[2] = 0;

        for (int j = 0; j < triangles.size(); j++)
        {
            if (j == i)
            {
                continue;
            }

            const tri_t& tri2 = triangles[j];

            edge_t edges2[3];
            edges2[0].a = tri2.a;
            edges2[0].b = tri2.b;

            edges2[1].a = tri2.b;
            edges2[1].b = tri2.c;

            edges2[2].a = tri2.c;
            edges2[2].b = tri2.a;

            // 判断三角形1和三角形2的三条边是否有相同的
            for (int k = 0; k < 3; k++)
            {
                if (edges[k] == edges2[0])
                {
                    counts[k]++;
                }
                if (edges[k] == edges2[1])
                {
                    counts[k]++;
                }
                if (edges[k] == edges2[2])
                {
                    counts[k]++;
                }
            }
        }

        // 如果一条边与其他三角形公用次数为0，则这条边悬空
        for (int k = 0; k < 3; k++)
        {
            if (0 == counts[k])
            {
                dangling_edges.push_back(edges[k]);
            }
        }
    }
}

/**
 * \brief 
 * \param body_a 
 * \param body_b 
 * \param bias 
 * \param simplex_points 
 * \param pt_on_a 
 * \param pt_on_b 
 * \return 
 */
float EPA_Expand(const Body* body_a, const Body* body_b, const float bias, const point_t simplex_points[4], Vec3& pt_on_a,
                 Vec3& pt_on_b)
{
    std::vector<point_t> points;
    std::vector<tri_t> triangles;
    std::vector<edge_t> danglingEdges;

    Vec3 center(0.0f);
    for (int i = 0; i < 4; i++)
    {
        points.push_back(simplex_points[i]);
        center += simplex_points[i].xyz;
    }
    center *= 0.25f;

    // Build the triangles
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        int k = (i + 2) % 4;
        tri_t tri;
        tri.a = i;
        tri.b = j;
        tri.c = k;

        int unusedPt = (i + 3) % 4;
        float dist = SignedDistanceToTriangle(tri, points[unusedPt].xyz, points);

        // The unused point is always on the negative/inside of the triangle.. make sure the normal points away
        if (dist > 0.0f)
        {
            std::swap(tri.a, tri.b);
        }

        triangles.push_back(tri);
    }

    //
    //	Expand the simplex to find the closest face of the CSO to the origin
    //
    while (true)
    {
        const int idx = ClosestTriangle(triangles, points);
        Vec3 normal = NormalDirection(triangles[idx], points);

        const point_t newPt = Support(body_a, body_b, normal, bias);

        // if w already exists, then just stop
        // because it means we can't expand any further
        if (HasPoint(newPt.xyz, triangles, points))
        {
            break;
        }

        float dist = SignedDistanceToTriangle(triangles[idx], newPt.xyz, points);
        if (dist <= 0.0f)
        {
            break; // can't expand
        }

        const int newIdx = static_cast<int>(points.size());
        points.push_back(newPt);

        // Remove Triangles that face this point
        int numRemoved = RemoveTrianglesFacingPoint(newPt.xyz, triangles, points);
        if (0 == numRemoved)
        {
            break;
        }

        // Find Dangling Edges
        danglingEdges.clear();
        FindDanglingEdges(danglingEdges, triangles);
        if (0 == danglingEdges.size())
        {
            break;
        }

        // In theory the edges should be a proper CCW order
        // So we only need to add the new point as 'a' in order
        // to create new triangles that face away from origin
        for (int i = 0; i < danglingEdges.size(); i++)
        {
            const edge_t& edge = danglingEdges[i];

            tri_t triangle;
            triangle.a = newIdx;
            triangle.b = edge.b;
            triangle.c = edge.a;

            // Make sure it's oriented properly
            float dist = SignedDistanceToTriangle(triangle, center, points);
            if (dist > 0.0f)
            {
                std::swap(triangle.b, triangle.c);
            }

            triangles.push_back(triangle);
        }
    }

    // Get the projection of the origin on the closest triangle
    const int idx = ClosestTriangle(triangles, points);
    const tri_t& tri = triangles[idx];
    Vec3 ptA_w = points[tri.a].xyz;
    Vec3 ptB_w = points[tri.b].xyz;
    Vec3 ptC_w = points[tri.c].xyz;
    Vec3 lambdas = BarycentricCoordinates(ptA_w, ptB_w, ptC_w, Vec3(0.0f));

    // Get the point on shape A
    Vec3 ptA_a = points[tri.a].pt_a;
    Vec3 ptB_a = points[tri.b].pt_a;
    Vec3 ptC_a = points[tri.c].pt_a;
    pt_on_a = ptA_a * lambdas[0] + ptB_a * lambdas[1] + ptC_a * lambdas[2];

    // Get the point on shape B
    Vec3 ptA_b = points[tri.a].pt_b;
    Vec3 ptB_b = points[tri.b].pt_b;
    Vec3 ptC_b = points[tri.c].pt_b;
    pt_on_b = ptA_b * lambdas[0] + ptB_b * lambdas[1] + ptC_b * lambdas[2];

    // Return the penetration distance
    Vec3 delta = pt_on_b - pt_on_a;
    return delta.GetMagnitude();
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
