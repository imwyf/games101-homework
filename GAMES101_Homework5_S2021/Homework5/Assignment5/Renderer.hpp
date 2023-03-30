#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv; // 可以通过这个uv表示射线与三角形交点的重心坐标
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};