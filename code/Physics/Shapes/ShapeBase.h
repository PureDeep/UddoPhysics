//
//	ShapeBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"
#include <vector>

/*
====================================================
Shape
====================================================
*/
class Shape
{
public:
    virtual Mat3 InertiaTensor() const = 0;

    virtual Bounds GetBounds(const Vec3& pos, const Quat& orient) const = 0;
    virtual Bounds GetBounds() const = 0;

    virtual Vec3 GetCenterOfMass() const { return m_centerOfMass; }

    enum shapeType_t
    {
        SHAPE_SPHERE,
        SHAPE_BOX,
        SHAPE_CONVEX,
    };

    virtual shapeType_t GetType() const = 0;

    // 用于给出凸包上在某一特定方向上最远的点
    // 支撑点就是，找出形状中所有顶点在该方向的投影点，如果该方向作为坐标轴，具有最大值的投影点所对应的顶点就是支撑点。
    // 在GJK算法中，通过分别计算顶点坐标和方向的点乘，然后找到最大值得到支撑点。
    // 当顶点比较多时，可以使用爬山法来优化遍历过程，但却会引入顶点间的连接信息等数据，增加内存。
    // Chaos没有使用该方法可能是考虑到了这方面因素。
    // https://zhuanlan.zhihu.com/p/396719279
    virtual Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, float bias) const = 0;

    // 用于持续性检测，只对非球面形状有效。
    virtual float FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const { return 0.0f; }

protected:
    Vec3 m_centerOfMass;
};
