#include <iostream>
#include <opencv2/opencv.hpp>
#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

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

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m;
    m << -zNear, 0, 0, 0,
        0, -zNear, 0, 0,
        0, 0, -zNear - zFar, -zNear * zFar,
        0, 0, 1, 0;

    float halve = (eye_fov / 2) * MY_PI / 180;
    float t = tan(halve) * zNear;
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;
    Eigen::Matrix4f scale, trans; // 缩放  位移矩阵
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;

    trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(-zFar + -zNear) / 2,
        0, 0, 0, 1;

    projection = scale * trans * m;

    return projection;
}
// 获取顶点位置
Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload)
{
    return payload.position;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f &vec, const Eigen::Vector3f &axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

// 法线着色器  使用法线表示颜色
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload &payload)
{
    // 法线各个坐标的范围是---->[-1,1]  但是颜色没有负数  加上111，再除2，将其范围限制为[0,1]
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

// 纹理着色器  与phong的过程类似
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture) // 如果有纹理 ，则获取纹理
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    // ka 环境光  kd漫反射--->纹理代替  ks高光
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f; // 使用纹理代替漫反射
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};
    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = (eye_pos - point);                              // 眼睛观察向量
        auto l = (light.position - point);                       // 入射光源向量
        auto h = (v.normalized() + l.normalized()).normalized(); // v和l的半程向量
        auto r_2 = l.dot(l);                                     // 光源到point距离的平方，用于计算光能量的损失

        // 环境光   La = ka(light)
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        // 漫反射  Ld = kd(light/r_2) * max(0, n*l)
        auto diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, normal.dot(l.normalized()));
        // 高光  Ls = ks(light/r_2) * max(0, n*h)^p
        auto specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, normal.dot(h)), p);

        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}

// phong模型着色器
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload)
{
    // ka环境光系数    kd漫反射（即物体本身的颜色）    ks高光系数(镜面反射)
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    // 初始化两个光源 ---->点光源的坐标、亮度
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};            // 点光源数组
    Eigen::Vector3f amb_light_intensity{10, 10, 10}; // 环境光强度
    Eigen::Vector3f eye_pos{0, 0, 0};                // 观测位置

    float p = 150; // 控制高光范围--->幂

    Eigen::Vector3f color = payload.color;                // 颜色
    Eigen::Vector3f point = payload.view_pos;             // 像素对应的三维位置
    Eigen::Vector3f normal = payload.normal.normalized(); // 法线

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = (eye_pos - point);                              // 眼睛观察向量
        auto l = (light.position - point);                       // 入射光源向量
        auto h = (v.normalized() + l.normalized()).normalized(); // v和l的半程向量
        auto r_2 = l.dot(l);                                     // 光源到point距离的平方，用于计算光能量的损失

        // 环境光   La = ka(light)
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        // 漫反射  Ld = kd(light/r_2) * max(0, n*l)
        auto diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, normal.dot(l.normalized()));
        // 高光  Ls = ks(light/r_2) * max(0, n*h)^p
        auto specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, normal.dot(h)), p);

        // L = La + Ld + Ls
        result_color += (ambient + diffuse + specular);
    }
    return result_color * 255.f;
}

// 可视化凹凸向量
Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload)
{
    // 描述法线如何移动
    //  TODO: Implement bump mapping here
    //  Let n = normal = (x, y, z)
    //  Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    //  Vector b = n cross product t
    //  Matrix TBN = [t b n]
    //  dU = kh * kn * (h(u+1/w,v)-h(u,v))
    //  dV = kh * kn * (h(u,v+1/h)-h(u,v))
    //  Vector ln = (-dU, -dV, 1)
    //  Normal n = normalize(TBN * ln)

    // 法向量
    Eigen::Vector3f normal = payload.normal;
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    // 纹理记录相对高度移动
    float u = payload.tex_coords.x(); // 纹理坐标x
    float v = payload.tex_coords.y(); // 纹理坐标y
    float w = payload.texture->width;
    float h = payload.texture->height;

    float kh = 0.2, kn = 0.1;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    normal = TBN * ln; // 改变法线来改变着色效果

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;
    return result_color * 255.f;
}

// 可视化置换信息
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload)
{

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150; // 控制高光范围--->幂

    Eigen::Vector3f color = payload.color;    // 颜色
    Eigen::Vector3f point = payload.view_pos; // 像素对应的三维位置
    Eigen::Vector3f normal = payload.normal;  // 法线

    float kh = 0.2, kn = 0.1;

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    point += (kn * normal * payload.texture->getColor(u, v).norm());
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = eye_pos - point;                                // 眼睛观察向量
        auto l = light.position - point;                         // 入射光源向量
        auto h = (v.normalized() + l.normalized()).normalized(); // v和l的半程向量
        auto r_2 = l.dot(l);                                     // 光源到point距离的平方，用于计算光能量的损失

        // 环境光   La = ka(light)
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        // 漫反射  Ld = kd(light/r_2) * max(0, n*l)
        auto diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, normal.normalized().dot(l.normalized()));
        // 高光  Ls = ks(light/r_2) * max(0, n*h)^p
        auto specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, normal.normalized().dot(h)), p);

        result_color += (ambient + diffuse + specular);
    }
    return result_color * 255.f;
}

/*
1. 修改函数 rasterize_triangle(const Triangle& t) in rasterizer.cpp:
在此处实现与作业2 类似的插值算法，实现法向量、颜色、纹理颜色的插值。
2. 修改函数 get_projection_matrix() in main.cpp: 将你自己在之前的实验中实现的投影矩阵填到此处，此时你可以运行 ./Rasterizer output.png normal来观察法向量实现结果。
3. 修改函数 phong_fragment_shader() in main.cpp: 实现 Blinn-Phong 模型计算 Fragment Color.
4. 修改函数 texture_fragment_shader() in main.cpp: 在实现 Blinn-Phong的基础上，将纹理颜色视为公式中的 kd，实现 Texture Shading FragmentShader.
5. 修改函数 bump_fragment_shader() in main.cpp: 在实现 Blinn-Phong 的基础上，仔细阅读该函数中的注释，实现 Bump mapping.
6. 修改函数 displacement_fragment_shader() in main.cpp: 在实现 Bumpmapping 的基础上，实现 displacement mapping.
*/

int main(int argc, const char **argv)
{
    std::vector<Triangle *> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for (auto mesh : Loader.LoadedMeshes) // 加载模型中的全部三角形  放入TriangleList中
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            auto *t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                // 顶点信息
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                // 法线信息
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                // 纹理信息
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);    // 光栅器初始化
    auto texture_path = "hmap.jpg"; // 纹理图片路径
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture") // 纹理模型
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal") // 法线模型
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong") // phong模型
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump") // 凹凸模型
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0, 0, 10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 依次是mvp变换矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);
        return 0;
    }

    return 0;
}
