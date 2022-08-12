//
//	Contact.h
//
#pragma once
#include "Body.h"


struct contact_t
{
    Vec3 ptOnA_WorldSpace; // 世界坐标系下A上的接触点
    Vec3 ptOnB_WorldSpace; // 世界坐标系下B上的接触点
    Vec3 ptOnA_LocalSpace; // A上的接触点在A本地坐标系下的坐标
    Vec3 ptOnB_LocalSpace; // B上的接触点在B本地坐标系下的坐标

    Vec3 normal; // In World Space coordinates
    float separationDistance; // 分离距离，未穿透为正，穿透为负。positive when non-penetrating, negative when penetrating
    float timeOfImpact; // 发生碰撞的时间

    Body* bodyA;
    Body* bodyB;
};

void ResolveContact(contact_t& contact);
