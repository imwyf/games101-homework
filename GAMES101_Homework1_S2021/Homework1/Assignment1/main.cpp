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
    // 记得先标准化
    axis = axis.normalized();

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f translate;
    translate << 0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;
    /* ---------------------------------注意---------------------------------------
    这里是先计算出罗德里格斯旋转矩阵，再对其扩展成4x4矩阵，不要先将axis扩展成齐次坐标带入公式,因为，
    如果先扩展，I是4阶单位矩阵，w维度上的向量是(0,0,0,1)，在公式里会乘以cos(a)，同时n*nT也会改变
    w维（第4维）的值，使得该向量变为(0,0,0,w)，导致顶点坐标变成(x/w,y/w,z/w,1)，成倍的缩放坐标。
    旋转矩阵只负责旋转，因此，应该保证w维向量始终为(0,0,0,1)，因此先计算三维旋转矩阵，再进行扩展。
    ------------------------------------------------------------------------------*/
    model.block<3, 3>(0, 0) = (cos(angle) * I + (1 - cos(angle)) * (axis * axis.transpose()) + sin(angle) * translate);

    return model;
}

// 生成M矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // 旋转轴
    Vector3f axis(0, 0, 1);
    Eigen::Matrix4f translate;
    // 注意：rotation_angle是角度，要把它变化成弧度
    float angle = rotation_angle / 180.0f * MY_PI;

    /* // 绕z轴旋转
    translate << cos(angle), -sin(angle), 0, 0,
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1; */

    translate = get_rotation(axis, angle);
    return translate;
}

// 生成P矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // 角度化弧度
    eye_fov = eye_fov * MY_PI / 180.0;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // Compute l, r, b, t
    float t = tan(eye_fov / 2) * -zNear; // 传进来的zNear是长度没有负数，因此取负
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    // Orthographic projection
    // Translate to origin
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = -(r + l) / 2;
    translate(1, 3) = -(t + b) / 2;
    translate(2, 3) = -(zNear + zFar) / 2;

    // Sclae to [-1,1]^3
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = 2 / (r - l);
    scale(1, 1) = 2 / (t - b);
    scale(2, 2) = 2 / (zNear - zFar);

    // get Orthographic projection
    Eigen::Matrix4f ortho = scale * translate;

    // Perspective projection
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Zero();
    persp2ortho(0, 0) = zNear;
    persp2ortho(1, 1) = zNear;
    persp2ortho(2, 2) = zNear + zFar;
    persp2ortho(2, 3) = -zNear * zFar;
    persp2ortho(3, 2) = 1;

    // get Perspective projection
    projection = ortho * persp2ortho;

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
