//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir;
    Vector3f L_indir;

    // 先使用BVH结构判断相交，得到交点p
    Intersection inter = intersect(ray);
    // 不相交
    if (!inter.happened)
        return L_dir;

    // 与光源相交
    if (inter.m->hasEmission())
        return inter.m->getEmission(); // 直接得到自发光项，就是渲染方程的第一项（渲染方程=自发光项+反射光项）

    // 与不发光物体相交
    Vector3f p = inter.coords; // p_normal
    Material *m = inter.m;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction; // wo是着色点实际光线的出射方向，即摄像机投射的光线的反方向（光路可逆）

    // 对于这个交点p，先在光源上采样，采样后得到一个光源上的采样点
    float pdf_Light;
    Intersection sample_point;
    // 在sampleLight中，pdf_Light会被设置为1/A，并返回
    // 在sampleLight中，参数sample_point的坐标会被改变成光源上采样点x的坐标
    sampleLight(sample_point, pdf_Light);

    Vector3f x = sample_point.coords;
    Vector3f ws = (x - p).normalized();             // ws是从p指向光源采样点x的方向
    Vector3f NN = sample_point.normal.normalized(); // x_normal
    Vector3f emit = sample_point.emit;
    float d = (x - p).norm();

    // 沿着ws发出一条光线，判断中间是否有阻挡
    Ray Obj2Light(p, ws);
    float d2 = intersect(Obj2Light).distance;
    // 深度是否相等，浮点数相等用阈值判断
    if (d - d2 < 0.001)
        L_dir = emit * m->eval(wo, ws, N) * dotProduct(N, ws) * dotProduct(NN, -ws) / std::pow(d, 2) / pdf_Light;

    // 测试俄罗斯轮盘赌
    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = m->sample(wo, N).normalized();
        Ray r(p, wi); // 第二条光线，间接反射
        Intersection inter = intersect(r);

        // 判断是否打到的物体是不发光的
        if (inter.happened && !inter.m->hasEmission())
        {
            // 投射光线，将返回的颜色结果作为间接光照
            L_indir = castRay(r, depth + 1) * m->eval(wo, wi, N) * dotProduct(wi, N) / m->pdf(wo, wi, N) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}