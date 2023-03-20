// clang-format off
//
// Created by goksu on 4/6/19.
//

#include<tuple>
#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

// 生成采样点
std::vector<Eigen::Vector3f> genSSList(int x,int y)
{
    Eigen::Vector3f a(x,y,1.0f);
    Eigen::Vector3f b(x,y+0.5,1.0f);
    Eigen::Vector3f c(x+0.5,y,1.0f);
    Eigen::Vector3f d(x+0.5,y+0.5,1.0f);
    return {a,b,c,d};
}

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // 由三角形的三个顶点求三个边的向量
    Vector3f p0p1,p1p2,p2p0;
    p0p1 = _v[1]-_v[0];
    p1p2 = _v[2]-_v[1];
    p2p0 = _v[0]-_v[2];

    // 扩展点p，取中心坐标
    Vector3f p(x+0.5,y+0.5,1.0f);
    // 求三个叉积的z分量
    float z1 = p0p1.cross(p-_v[0]).z();
    float z2 = p1p2.cross(p-_v[1]).z();
    float z3 = p2p0.cross(p-_v[2]).z();
    // 判断是否同号,signbit()：<0-> return 1
    // 这里注意不要用三个连等，if(a==b==c)这种语句，c++不支持
    if(signbit(z1) == signbit(z2) && signbit(z2) == signbit(z3)) return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) 
{
    auto v = t.toVector4();
    // get the bounding box of current triangle
    int minX,minY,maxX,maxY;
    minX = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    maxX = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    minY = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    maxY = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    // 遍历
    for(int y = minY; y <= maxY; y++)
        for (int x = minX; x <=maxX ; x++)
        {
            // 扩展作业：MSAA
            // 每个像素内部维护一个数组sample_list，对里面的每个采样点都判断一次在不在三角形内，按照百分比来给予灰度值
            auto ssList = genSSList(x,y);
            int judge = 0;
            // 对每个采样点用z_buffer
            for (int i=0; i<ssList.size(); i++)
            {
                if (insideTriangle(ssList[i].x(), ssList[i].y(), t.v))
                {
                    auto tup = computeBarycentric2D(ssList[i].x(), ssList[i].y(), t.v);
                    float alpha;
                    float beta;
                    float gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (super_depth_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] > z_interpolated)
                    {
                        judge = 1;
                        //深度存入缓存
                        super_depth_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] = z_interpolated;
                        //颜色存入缓存
                        super_frame_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] = t.getColor();
                    }
                }
            }
            if (judge)
            //若像素的四个样本中有一个通过了深度测试，就需要对该像素进行着色，因为有一个通过就说明有颜色，就需要着色。
            {
                Vector3f point = {x,y,0 };
                Vector3f color = (super_frame_buf[get_super_index(x*2 , y*2)]+ super_frame_buf[get_super_index(x*2+1, y*2)]+ super_frame_buf[get_super_index(x*2, y*2+1)]+ super_frame_buf[get_super_index(x*2+1, y*2+1)])/4;
                //着色
                set_pixel(point, color);
            }
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y)
{
    return (height*2 - 1 - y) * width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}
// clang-format on