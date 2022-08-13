//
//  Broadphase.cpp
//
#include "Broadphase.h"


int CompareSAP(const void* a, const void* b)
{
    auto ea = static_cast<const psuedoBody_t*>(a);
    auto eb = static_cast<const psuedoBody_t*>(b);

    if (ea->value < eb->value)
    {
        return -1;
    }
    return 1;
}

void SortBodiesBounds(const Body* bodies, const int num, psuedoBody_t* sortedArray, const float dt_sec)
{
    auto axis = Vec3(1, 1, 1);
    axis.Normalize();

    for (int i = 0; i < num; i++)
    {
        const Body& body = bodies[i];
        Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

        // 在broadphase过程中，需要考虑物体的速度
        // 我们把物体在一个时间间隔内移动的距离添加到bounds里
        bounds.Expand(bounds.mins + body.m_linearVelocity * dt_sec);
        bounds.Expand(bounds.maxs + body.m_linearVelocity * dt_sec);

        // 给bounds加一个阈值边界
        const float epsilon = 0.01f;
        bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
        bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

        sortedArray[i * 2 + 0].id = i;
        sortedArray[i * 2 + 0].value = axis.Dot(bounds.mins);
        sortedArray[i * 2 + 0].ismin = true;

        sortedArray[i * 2 + 1].id = i;
        sortedArray[i * 2 + 1].value = axis.Dot(bounds.maxs);
        sortedArray[i * 2 + 1].ismin = false;
    }

    qsort(sortedArray, num * 2, sizeof(psuedoBody_t), CompareSAP);
}

void BuildPairs(std::vector<collisionPair_t>& collision_pairs, const psuedoBody_t* sorted_bodies, const int num)
{
    // 清空collision_pairs数组
    collision_pairs.clear();

    //建立collision pairs
    for (int i = 0; i < num * 2; i++)
    {
        const psuedoBody_t& a = sorted_bodies[i];
        if (!a.ismin)
        {
            continue;
        }

        collisionPair_t pair;
        pair.a = a.id;

        for (int j = i + 1; j < num * 2; j++)
        {
            const psuedoBody_t& b = sorted_bodies[j];
            // 如果b是最后一个元素，则终止
            if (b.id == a.id)
            {
                break;
            }

            if (!b.ismin)
            {
                continue;
            }

            pair.b = b.id;
            collision_pairs.push_back(pair);
        }
    }
}

void SweepAndPrune1D(const Body* bodies, int num, std::vector<collisionPair_t>& final_pairs, float dt_sec)
{
    auto sorted_bodies = static_cast<psuedoBody_t*>(alloca(sizeof(psuedoBody_t) * num * 2));

    SortBodiesBounds(bodies, num, sorted_bodies, dt_sec);
    BuildPairs(final_pairs, sorted_bodies, num);
}

/*
====================================================
BroadPhase
====================================================
*/
void BroadPhase(const Body* bodies, const int num, std::vector<collisionPair_t>& finalPairs, const float dt_sec)
{
    finalPairs.clear();

    SweepAndPrune1D(bodies, num, finalPairs, dt_sec);
}
