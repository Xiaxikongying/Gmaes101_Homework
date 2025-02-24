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

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

/// @brief 像素点是否在三角形内部
/// @param xy 像素点的位置
/// @param _v 三角形的指针
/// @return 是否在三角形内部
static bool insideTriangle2(int x, int y, const Vector3f *_v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f P(x, y); // 检测点
    // 三角形的三个点
    Eigen::Vector2f A = _v[0].head(2); // head是取前两个坐标
    Eigen::Vector2f B = _v[1].head(2);
    Eigen::Vector2f C = _v[2].head(2);
    // 三角形的三条边向量
    Eigen::Vector2f AB = B - A;
    Eigen::Vector2f BC = C - B;
    Eigen::Vector2f CA = A - C;
    // 点与边的向量
    Eigen::Vector2f AP = P - A;
    Eigen::Vector2f BP = P - B;
    Eigen::Vector2f CP = P - C;
    // 计算叉乘
    float eq1 = AB[0] * AP[1] - AB[1] * AP[0];
    float eq2 = BC[0] * BP[1] - BC[1] * BP[0];
    float eq3 = CA[0] * CP[1] - CA[1] * CP[0];

    if (eq1 > 0 && eq2 > 0 && eq3 > 0)
        return true;
    else if (eq1 < 0 && eq2 < 0 && eq3 < 0)
        return true;
    else
        return false;
}

static bool insideTriangle(float x, float y, const Vector3f *_v) // float
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f A = _v[0], B = _v[1], C = _v[2];
    Vector3f P(x, y, 0);
    Vector3f AB = B - A, BC = C - B, CA = A - C;
    Vector3f AP = P - A, BP = P - B, CP = P - C;
    return AB.cross(AP).z() > 0 && BC.cross(BP).z() > 0 && CA.cross(CP).z() > 0;
    // cross表示的是叉乘(三维)，z()表示的是z坐标（是前两维的计算结果）
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    // 取出三角形的顶点  下标  颜色数组
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    // 这里反一下，是因为本来是将-1映射到-50  1映射到-0.1
    // 但是深度计算处，是数字越小，深度越小。但由于-0.1大于-50，导致-0.1会被-50覆盖，所以这里反一下
    //  float f1 = (50 - 0.1) / 2.0;
    //  float f2 = (-50 + -0.1) / 2.0;
    float f1 = (-50 + -0.1) / 2.0;
    float f2 = (50 - 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    // 对每个三角形进行mvp变换
    for (auto &i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};

        // Homogeneous division   齐次化
        for (auto &vec : v)
        {
            vec /= vec.w();
        }
        // Viewport transformation  视口变换，将坐标映射到屏幕上
        for (auto &vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>()); // 设置视口变换后的顶点
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        // 设置各个顶点的颜色
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// 三角形光栅化
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();

    // 1. 创建三角形的 2 维 bounding box。（三角形接触到的最大最小坐标）
    float xmin = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    float ymin = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    float xmax = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    float ymax = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    // 化为整数
    xmin = (int)std::floor(xmin);
    xmax = (int)std::ceil(xmax);
    ymin = (int)std::floor(ymin);
    ymax = (int)std::ceil(ymax);

    // 2. 遍历此 bounding box 内的所有像素（使用其整数索引）。然后，使用像素中心的屏幕空间坐标来检查中心点是否在三角形内。
    // 每个像素记录4个点
    float dx[4] = {0.25, 0.25, 0.75, 0.75};
    float dy[4] = {0.25, 0.75, 0.25, 0.75};
    for (int x = xmin; x <= xmax; ++x)
    {
        for (int y = ymin; y <= ymax; ++y)
        {
            // 根据已有代码来得到 当前像素点的深度值，也就是z_interpolated即距离相机的远近  深度越小，距离相机越近
            // 计算深度  alpha, beta, gamma表示当前像素点在t.v中的重心坐标
            // z_interpolated = 三个顶点的z坐标 * 对应的重心坐标 ====>三个顶点z坐标的插值
            auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v); // xy像素坐标   t.v所在的三角形
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            for (int i = 0; i < 4; ++i)
            {
                float nx = x + dx[i];
                float ny = y + dy[i];
                // 3. 如果在内部，则将其位置处的插值深度值 (interpolated depth value) 与深度缓冲区 (depth buffer) 中的相应值进行比较。
                if (insideTriangle(nx, ny, t.v)) // 如果有点在内部
                {
                    // 4. 如果当前点更靠近相机，请设置像素颜色并更新深度缓冲区 (depth buffer)
                    // 如果当前位置深度比depth_buf（类rasterizer的一个成员）更小，则更新颜色值。（处理两个三角形重叠部分）
                    int ind = get_index(x, y);
                    if (z_interpolated < pixels[ind].depth[i])
                    {
                        pixels[ind].depth[i] = z_interpolated; // 更新当前点的深度
                        pixels[ind].color[i] = t.getColor();   // 记录当前点的颜色
                    }
                }
            }
        }
    }

    // 将每个像素点内的4个记录点的颜色混合
    for (int x = xmin; x <= xmax; x++)
    {
        for (int y = ymin; y <= ymax; y++) // 遍历每个像素点
        {
            int index = get_index(x, y);
            Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);
            for (int i = 0; i < 4; i++) // 遍历4个记录点
            {
                color += pixels[index].color[i]; // 将4个记录点的颜色相加
            }
            color /= 4.0;                               // 求平均值
            set_pixel(Eigen::Vector3f(x, y, 0), color); // 设置像素点的颜色
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for (int i = 0; i < pixels.size(); ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                pixels[i].color[j] = Eigen::Vector3f{0, 0, 0};
                pixels[i].depth[j] = std::numeric_limits<float>::infinity();
            }
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    pixels.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on