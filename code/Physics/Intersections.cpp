//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, contact_t& contact)
{
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    contact.timeOfImpact = 0.0f;

    // 如果是两个球
    if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE)
    {
        const ShapeSphere* sphere_a = dynamic_cast<ShapeSphere*>(bodyA->m_shape);
        const ShapeSphere* sphere_b = dynamic_cast<ShapeSphere*>(bodyB->m_shape);

        Vec3 pos_a = bodyA->m_position;
        Vec3 pos_b = bodyB->m_position;

        if (SphereSphereStatic(sphere_a, sphere_b, pos_a, pos_b, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace))
        {
            contact.normal = pos_a - pos_b;
            contact.normal.Normalize();

            contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
            contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

            const Vec3 ab = pos_a - pos_b;
            const float separation_distance = ab.GetMagnitude() - (sphere_a->m_radius + sphere_b->m_radius);
            contact.separationDistance = separation_distance;
            return true;
        }
        Vec3 pt_on_a;
        Vec3 pt_on_b;
        const float bias = 0.001f;

        if (GJK_DoesIntersect(bodyA, bodyB, bias, pt_on_a, pt_on_b))
        {
            Vec3 normal = pt_on_b - pt_on_b;
            normal.Normalize();

            pt_on_a -= normal * bias;
            pt_on_b += normal * bias;

            contact.normal = normal;

            contact.ptOnA_WorldSpace = pt_on_a;
            contact.ptOnB_WorldSpace = pt_on_b;

            contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
            contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

            Vec3 ab = bodyB->m_position - bodyA->m_position;
            float separation_distance = (pt_on_a - pt_on_b).GetMagnitude();
            contact.separationDistance = -separation_distance;
            return true;
        }

        // 未发生接触的情况
        GJK_ClosestPoints(bodyA, bodyB, pt_on_a, pt_on_b);
        contact.ptOnA_WorldSpace = pt_on_a;
        contact.ptOnB_WorldSpace = pt_on_b;

        contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
        contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

        Vec3 ab = bodyB->m_position - bodyA->m_position;
        float r = (pt_on_a - pt_on_b).GetMagnitude();
        contact.separationDistance = r;

        return false;
    }

    const Vec3 ab = bodyB->m_position - bodyA->m_position;
    contact.normal = ab;
    contact.normal.Normalize();

    const ShapeSphere* sphere_a = dynamic_cast<ShapeSphere*>(bodyA->m_shape);
    const ShapeSphere* sphere_b = dynamic_cast<ShapeSphere*>(bodyB->m_shape);

    contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphere_a->m_radius;
    contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphere_b->m_radius;

    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = ab.GetLengthSqr();
    if (length_square <= (radius_ab * radius_ab))
        return true;

    return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact)
{
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;

    // 如果两个都是球体，则执行球体的碰撞
    if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE)
    {
        const ShapeSphere* sphere_a = dynamic_cast<ShapeSphere*>(bodyA->m_shape);
        const ShapeSphere* sphere_b = dynamic_cast<ShapeSphere*>(bodyB->m_shape);

        const Vec3 pos_a = bodyA->m_position;
        const Vec3 pos_b = bodyB->m_position;

        const Vec3 vel_a = bodyA->m_linearVelocity;
        const Vec3 vel_b = bodyB->m_linearVelocity;

        if (SphereSphereDynamic(sphere_a, sphere_b, pos_a, pos_b, vel_a, vel_b, dt, contact.ptOnA_WorldSpace,
                                contact.ptOnB_WorldSpace, contact.timeOfImpact))
        {
            // 前向模拟，CCD预测
            bodyA->Update(contact.timeOfImpact);
            bodyB->Update(contact.timeOfImpact);

            // 转换
            contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
            contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

            contact.normal = bodyA->m_position - bodyB->m_position;
            contact.normal.Normalize();

            // 复位
            bodyA->Update(-contact.timeOfImpact);
            bodyB->Update(-contact.timeOfImpact);

            const Vec3 ab = bodyB->m_position - bodyA->m_position;
            float r = ab.GetMagnitude() - (sphere_a->m_radius + sphere_b->m_radius);
            contact.separationDistance = r;
            return true;
        }
    }
    return false;
}


/**
 * \brief 检测碰撞
 * \param body_a BodyA
 * \param body_b BodyB
 * \return 返回是否碰撞
 */
bool Intersect(const Body* body_a, const Body* body_b)
{
    const Vec3 distance_ab = body_b->m_position - body_a->m_position;
    const auto sphere_a = dynamic_cast<const ShapeSphere*>(body_a->m_shape);
    const auto sphere_b = dynamic_cast<const ShapeSphere*>(body_b->m_shape);

    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = distance_ab.GetLengthSqr();

    if (length_square <= (radius_ab * radius_ab))
        return true;
    return false;
}

/**
 * \brief 射线与球体碰撞检测，可以参考GAMES103 Lecture02 Example 4: Particale-Sphere Collision
 * \param ray_start 射线起始点
 * \param ray_dir 射线方向
 * \param sphere_center 球体中心坐标
 * \param sphere_radius 球体半径
 * \param t1 射线发生碰撞的参数t1
 * \param t2 射线发生碰撞的参数t2
 * \return 射线是否与球体发生碰撞
 */
bool RaySphere(const Vec3& ray_start, const Vec3& ray_dir, const Vec3& sphere_center, float sphere_radius, float& t1,
               float& t2)
{
    // d = b - a
    // 射线的形式：r(t) = a + d * t
    // r(t)可以看作射线头部的坐标
    // 射线头部与球体中心坐标形成的向量：s(t) = c - r(t)，c为球体中心坐标
    // 当射线与球体相撞，有接触点，即可得：s · s = R^2，或者写成：s.Dot(s) = R * R, R为球体半径大小
    // 求解一元二次方程
    // d^2 * t^2 + a·d * t + a^2 + c^2 - R^2 = 0
    const Vec3 m = sphere_center - ray_start;
    const float a = ray_dir.Dot(ray_dir); // d^2
    const float b = m.Dot(ray_dir); // a·d
    const float c = m.Dot(m) - sphere_radius * sphere_radius; // a^2 + c^2 - R^2

    const float delta = b * b - a * c;
    const float inv_a = 1.0f / a;

    if (delta < 0)
        return false;

    const float delta_root = sqrtf(delta);
    t1 = inv_a * (b - delta_root);
    t2 = inv_a * (b + delta_root);

    return true;
}

bool SphereSphereDynamic(const ShapeSphere* shape_a, const ShapeSphere* shape_b, const Vec3& pos_a, const Vec3& pos_b,
                         const Vec3& vel_a, const Vec3& vel_b, float dt, Vec3& pt_a, Vec3& pt_b, float& toi)
{
    const Vec3 relative_velocity = vel_a - vel_b;

    const Vec3 start_a = pos_a;
    const Vec3 end_a = pos_a + relative_velocity * dt;
    const Vec3 ray_dir = end_a - start_a;

    float t0 = 0;
    float t1 = 0;

    if (ray_dir.GetLengthSqr() < 0.001f * 0.001f)
    {
        // Ray is too short, just check is already intersecting
        const Vec3 ab = pos_b - pos_a;
        const float radius = shape_a->m_radius + shape_b->m_radius + 0.001f;
        if (ab.GetLengthSqr() > radius * radius)
            return false;
    }
    else if (!RaySphere(pos_a, ray_dir, pos_b, shape_a->m_radius + shape_b->m_radius, t0, t1))
    {
        return false;
    }

    // 将t0,t1从[0, 1]缩放到[0, dt]范围
    t0 *= dt;
    t1 *= dt;

    // 如果计算出的时间较大的值t1任然处于过去，则说明这一帧未来没有碰撞
    if (t1 < 0.0f)
        return false;

    // 得到距离现在最近的碰撞时间
    toi = (t0 < 0.0f) ? 0.0f : t0;

    // 如果距离现在最近的碰撞时间也要大于dt，则说明这一帧没有碰撞
    if (toi > dt)
        return false;

    //
    Vec3 new_pos_a = pos_a + vel_a * toi; // a的新坐标
    Vec3 new_pos_b = pos_b + vel_b * toi; // b的新坐标
    Vec3 ab = new_pos_b - new_pos_a;
    ab.Normalize();

    pt_a = new_pos_a + ab * shape_a->m_radius;
    pt_b = new_pos_b - ab * shape_b->m_radius;
    return true;
}

bool SphereSphereStatic(const ShapeSphere* shape_a, const ShapeSphere* shape_b, const Vec3& pos_a,
                        const Vec3& pos_b, Vec3& pt_a, Vec3& pt_b)
{
    const Vec3 ab = pos_b - pos_a;
    Vec3 norm = ab;
    norm.Normalize();

    pt_a = pos_a + norm * shape_a->m_radius;
    pt_b = pos_b - norm * shape_b->m_radius;

    const float radius_ab = shape_a->m_radius + shape_b->m_radius;
    const float length_sqr = ab.GetLengthSqr();

    if (length_sqr <= radius_ab * radius_ab)
    {
        return true;
    }

    return false;
}
