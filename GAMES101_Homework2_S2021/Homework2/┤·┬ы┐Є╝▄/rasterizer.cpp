// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

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

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // 由三角形的三个顶点求三个边的向量
    Vector3f p0p1,p1p2,p2p0;
    p0p1 = _v[1]-_v[0];
    p1p2 = _v[2]-_v[1];
    p2p0 = _v[0]-_v[2];

    // 扩展点Q，取中心坐标
    Vector3f Q(x,y,1.0f);
    // 求三个叉积的z分量
    float z1 = p0p1.cross(Q-_v[0]).z();
    float z2 = p1p2.cross(Q-_v[1]).z();
    float z3 = p2p0.cross(Q-_v[2]).z();
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

// MSAA，遍历一个像素内部的超采样点，判断其是否满足1.深度更小 2.在三角形内，返回满足的超采样像素个数。本质上这是将1个像素点变成4个像素点，做4次深度判断、位置判断，因此深度缓存和颜色缓存4倍大小，例子：depth_buf[0]、depth_buf[1]、depth_buf[2]、depth_buf[3]对应的是在(0,0)的像素的4个采样点的深度。n、m表示采样点矩阵，即1个像素一共有n*m个采样点，n是行数，m是列数
int rst::rasterizer::MSAA(int x, int y, const Triangle& t, int n, int m, float z)
{
    float size_x = 1.0/n; // the size_x of every super sample pixel
    float size_y = 1.0/m;

    int blocksinTriangle = 0;
    // 遍历n*m个采样点
    for(int i=0; i<n; ++i) 
        for(int j=0; j<m; ++j) 
        {
            float _x = x+i*size_x; // _x is the coordinate of the sample point
            float _y = y+j*size_y;
            if (z<MSAA_depth_buf[get_index(x,y)*4 + i*n + j] && insideTriangle(_x, _y, t.v)) 
            {
                // 写入深度缓存
                MSAA_depth_buf[get_index(x,y)*4 + i*n +j] = z;
                // 写入颜色缓存
                MSAA_frame_buf[get_index(x,y)*4 + i*n +j] = t.getColor();
                blocksinTriangle ++;
            }
        }
    return blocksinTriangle;
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

    // 设置采样点个数
    int n = 2;
    int m = 2;
    // 遍历每个像素
    for(int y = minY; y <= maxY; y++)
        for (int x = minX; x <=maxX ; x++)
        {
            // 扩展作业：MSAA
            int blockinTriangle = 0;
            // 深度插值,在三角形中的一个像素只有一个深度,因此用像素坐标(x,y)插值深度,比较后采样点深度取这个深度值,代表属于这个三角形,(和颜色类似,一个像素也只能有一个颜色,因此像素在哪个三角形内就取哪个三角形的颜色)因此放在MSAA采样点遍历外
            auto alpha = std::get<0>(computeBarycentric2D(x, y, t.v));
            auto beta = std::get<1>(computeBarycentric2D(x, y, t.v));
            auto gamma = std::get<2>(computeBarycentric2D(x, y, t.v));
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            // 当满足条件的采样点>0时，需要对这个像素着色
            if ((blockinTriangle = MSAA(x, y, t, n, m, z_interpolated)) > 0) 
            {
                int idx = get_index(x, y);
                // 混合所有采样点的颜色
                Vector3f mixColor = (MSAA_frame_buf[idx*4]+MSAA_frame_buf[idx*4+1]+MSAA_frame_buf[idx*4+2]+MSAA_frame_buf[idx*4+3])/4.0;
                set_pixel(Eigen::Vector3f(x, y, z_interpolated), mixColor);
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
        std::fill(MSAA_frame_buf.begin(), MSAA_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(MSAA_depth_buf.begin(), MSAA_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    MSAA_frame_buf.resize(w * h * 4);
    MSAA_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}
// clang-format on