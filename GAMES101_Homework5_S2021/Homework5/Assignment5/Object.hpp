#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Object
{
public:
    Object()
        : materialType(DIFFUSE_AND_GLOSSY), ior(1.3), Kd(0.8), Ks(0.2), diffuseColor(0.2), specularExponent(25)
    {
    }

    virtual ~Object() = default;
    // intersect函数用于检测光线是否与网格模型相交
    virtual bool intersect(const Vector3f &, const Vector3f &, float &, uint32_t &, Vector2f &) const = 0;

    // getSurfaceProperties函数用于获取相交点的表面属性，包括法向量和纹理坐标
    virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &,
                                      Vector2f &) const = 0;
    // evalDiffuseColor函数用于计算漫反射颜色
    virtual Vector3f evalDiffuseColor(const Vector2f &) const
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType; // 材质类型，用于描述物体的表面材质，包括漫反射、镜面反射、透明等。
    float ior;                 // 折射率，用于确定光线进入或离开球体时的弯曲程度
    float Kd, Ks;              // 漫反射系数和镜面反射系数
    Vector3f diffuseColor;     // 漫反射颜色
    float specularExponent;    // 镜面反射指数
};
