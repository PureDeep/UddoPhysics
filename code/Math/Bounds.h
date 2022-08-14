//
//	Bounds.h
//
#pragma once
#include <math.h>
#include <assert.h>
#include "Vector.h"
#include <vector>

/*
====================================================
Bounds
====================================================
*/
class Bounds
{
public:
    Bounds() { Clear(); }

    Bounds(const Bounds& rhs) : mins(rhs.mins), maxs(rhs.maxs)
    {
    }

    const Bounds& operator =(const Bounds& rhs);

    ~Bounds()
    {
    }

    void Clear()
    {
        mins = Vec3(1e6);
        maxs = Vec3(-1e6);
    }

    bool DoesIntersect(const Bounds& rhs) const;
    void Expand(const Vec3* pts, int num);
    void Expand(const Vec3& rhs);
    void Expand(const Bounds& rhs);

    // 返回X边长度
    float WidthX() const { return maxs.x - mins.x; }
    // 返回Y边长度
    float WidthY() const { return maxs.y - mins.y; }
    // 返回Z边长度
    float WidthZ() const { return maxs.z - mins.z; }

    // 只需要保存盒体的两个对角，就可以很方便地进行重叠比较
public:
    Vec3 mins;
    Vec3 maxs;
};
