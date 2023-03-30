//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel
{

public:
    // BVHAccel Public Types
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode *root;

    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);

    // BVHAccel Private Data
    const int maxPrimsInNode;         // 分割到最后，每个节点内最大物体数量
    const SplitMethod splitMethod;    // 切分方法
    std::vector<Object *> primitives; // 每个节点内物体集合
};

// 一个二叉树结构，每个BVH节点都有左儿子和右儿子，节点内储存的数据是本节点的包围盒边界和在这个包围盒里的物体集合。
struct BVHBuildNode
{
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object *object;

public:
    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
    // BVHBuildNode Public Methods
    BVHBuildNode()
    {
        bounds = Bounds3();
        left = nullptr;
        right = nullptr;
        object = nullptr;
    }
};

#endif // RAYTRACING_BVH_H
