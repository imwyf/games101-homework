#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

// 生成V矩阵
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 扩展作业：绕任意过原点的轴的旋转变换矩阵。
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    // 将轴向量扩展成齐次坐标
    axis = axis.normalized(); // 记得先标准化
    Vector4f n = Vector4f(axis.x(), axis.y(), axis.z(), 1.0f);

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    translate << 0, -n.z(), n.y(), 0,
        n.z(), 0, -n.x(), 0,
        -n.y(), n.x(), 0, 0,
        0, 0, 0, 1;
    model = cos(angle) * model + (1 - cos(angle)) * (n * n.transpose()) + sin(angle) * translate;

    return model;
}

// 生成M矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // 旋转轴
    Vector3f axis(0, 0, 1);
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    // 注意：rotation_angle是角度，要把它变化成弧度
    float angle = rotation_angle / 180.0f * MY_PI;

    /*
    // 绕z轴旋转
    translate << cos(angle), -sin(angle), 0, 0,
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    */

    translate = get_rotation(axis, angle);
    model = translate * model;

    return model;
}

// 生成P矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f per2ortho, translate_ortho, scale_ortho;
    // 角度化弧度
    eye_fov = eye_fov * MY_PI / 180.0;
    // width and height and deepth_z
    // 传进来的zNear是长度没有负数，因此取负
    auto top = -zNear * tan(eye_fov / 2);
    auto right = top * aspect_ratio;
    auto bottom = -top;
    auto left = -right;
    per2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    translate_ortho << 1, 0, 0, -0.5 * (right + left),
        0, 1, 0, -0.5 * (top + bottom),
        0, 0, 1, -0.5 * (zNear + zFar),
        0, 0, 0, 1;
    scale_ortho << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    // 疑问：这里为什么是反着乘的矩阵
    projection = per2ortho * translate_ortho * scale_ortho * projection;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    // 摄像机位置
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // 三角形顶点位置
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 索引
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
