#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

/*
本次作业的任务是填写一个旋转矩阵和一个透视投影矩阵。给定三维下三个点
v0(2.0, 0.0, ?2.0), v1(0.0, 2.0, ?2.0), v2(?2.0, 0.0, ?2.0),
你需要将这三个点的坐标变换为屏幕坐标并在屏幕上绘制出对应的线框三角形
(在代码框架中，我们已经提供了 draw_triangle 函数，所以你只需要去构建变换矩阵即可)。
简而言之，我们需要进行模型、视图、投影、视口等变换来将三角形显示在屏幕上。
在提供的代码框架中，我们留下了模型变换和投影变换的部分给你去完成。
*/
constexpr float MY_PI = 3.1415926;

// 视图变换---->将物体与摄像机一起移动（相机移动到原点），相对位置不变
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    // 将摄像机平移到原点
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;
    view = translate * view;
    return view;
}

// 模型变换---->model矩阵是为了初始化物体的位置物体
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // 旋转弧度a
    float a = rotation_angle / 180.0 * MY_PI;
    // 按z轴旋转的 旋转矩阵
    model << cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

/// @brief 投影矩阵
/// @param eye_fov 视场角
/// @param aspect_ratio 宽高比
/// @param zNear 近平面
/// @param zFar 远平面
/// @return
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // 投影变换矩阵，需要两步走：
    // 1.将透视投影转化为正交投影
    // 2.将正交投影转化到正则立方体内:  平移到原点 + 缩放
    float n = -zNear;
    float f = -zFar;
    // M_persp2Ortho：透视投影矩阵  将一个棱锥挤压乘一个立方体
    Eigen::Matrix4f m;
    m << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // M_ortho：正交投影矩阵    平移到原点 + 缩放
    Eigen::Matrix4f trans, scale;
    float havle = (eye_fov / 2) * MY_PI / 180.0; // 计算弧度
    float t = abs(n) * tan(havle);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;

    // 平移到原点
    trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    // 缩放
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    projection = scale * trans * m;
    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false; // 是否输入参数
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    // 构造一个光栅器
    rst::rasterizer r(700, 700);
    // 初始化观察点  默认朝向-Z轴
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // 三角形顶点
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // std::vector<Eigen::Vector3f> pos{{2, 1, -2}, {1, 2, -2}, {-2, 1, -2}};

    // 对顶点分组（这里只有一组）
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // 将三角形信息存入光栅器中
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;         // 键盘输入
    int frame_count = 0; // 帧数

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

    // 渲染循环
    while (key != 27) // Esc
    {
        // 清除帧缓存图像，准备重新绘制
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 向光栅器传入 模型变换矩阵
        r.set_model(get_model_matrix(angle));
        // 向光栅器传入 视角变换矩阵
        r.set_view(get_view_matrix(eye_pos));
        // 向光栅器传入 投影变换矩阵
        r.set_projection(get_projection_matrix(60, 1, 0.1, 50));

        // 开始渲染
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // 使用opencv显示图像
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count=  " << frame_count++ << '\n';

        // 物体 绕z运动
        if (key == 'z') // 左旋
        {
            angle += 10;
        }
        else if (key == 'c') // 右旋
        {
            angle -= 10;
        }

        // 观测坐标变化

        if (key == 'a') // 左移
        {
            eye_pos[0] += 1;
        }
        else if (key == 'd') // 右移
        {
            eye_pos[0] -= 1;
        }
        else if (key == 'w') // 上
        {
            eye_pos[1] -= 1;
        }
        else if (key == 's') // 下
        {
            eye_pos[1] += 1;
        }
        else if (key == 'i') // 大
        {
            eye_pos[2] -= 1;
        }
        else if (key == 'k') // 小
        {
            eye_pos[2] += 1;
        }
    }

    return 0;
}
