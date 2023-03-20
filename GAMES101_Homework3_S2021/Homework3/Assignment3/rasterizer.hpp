//
// Created by goksu on 4/6/19.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "global.hpp"
#include "Shader.hpp"
#include "Triangle.hpp"

using namespace Eigen;

namespace rst
{
    enum class Buffers
    {
        Color = 1,
        Depth = 2
    };

    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    enum class Primitive
    {
        Line,
        Triangle
    };

    /*
     * For the curious : The draw function takes two buffer id's as its arguments. These two structs
     * make sure that if you mix up with their orders, the compiler won't compile it.
     * Aka : Type safety
     * */
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

    class rasterizer
    {
    public:
        rasterizer(int w, int h);
        pos_buf_id load_positions(const std::vector<Eigen::Vector3f> &positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i> &indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f> &colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f> &normals);

        void set_model(const Eigen::Matrix4f &m);
        void set_view(const Eigen::Matrix4f &v);
        void set_projection(const Eigen::Matrix4f &p);

        void set_texture(Texture tex) { texture = tex; }

        void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
        void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

        void set_pixel(const Vector2i &point, const Eigen::Vector3f &color);

        void clear(Buffers buff);

        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
        void draw(std::vector<Triangle *> &TriangleList);

        std::vector<Eigen::Vector3f> &frame_buffer() { return frame_buf; }

    private:
        void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

        void rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &world_pos);

        // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER

    private:
        Eigen::Matrix4f model;
        Eigen::Matrix4f view;
        Eigen::Matrix4f projection;

        int normal_id = -1;

        // 位置缓存：map，int->容器，里面存的是3维向量->顶点的位置向量
        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        // 索引缓存:map，int->容器，存的是一个3维整型向量，代表一个图形的顶点索引，因此一个向量代表一个图形，容器的容量是需要渲染的图形个数
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;
        std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

        std::optional<Texture> texture;

        std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
        std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

        // 帧缓冲：一个容器，容器里面存的是3维向量->RGB值，容器的索引即屏幕坐标(w,h)展开的值
        std::vector<Eigen::Vector3f> frame_buf;
        // 深度缓存
        std::vector<float> depth_buf;
        int get_index(int x, int y);

        int width, height;

        int next_id = 0;
        int get_next_id() { return next_id++; }
    };
}
