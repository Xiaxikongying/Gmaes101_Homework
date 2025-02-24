// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // ͶӰ�任������Ҫ�����ߣ�
    // 1.��͸��ͶӰת��Ϊ����ͶӰ
    // 2.������ͶӰת����������������:  ƽ�Ƶ�ԭ�� + ����
    float n = -zNear;
    float f = -zFar;
    // m��͸��ͶӰ����  ��һ����׶��ѹ��һ��������
    Eigen::Matrix4f m;
    m << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // M_ortho������ͶӰ����    ƽ�Ƶ�ԭ�� + ����
    Eigen::Matrix4f trans, scale;
    float halve = (eye_fov / 2) * MY_PI / 180.0;
    float t = abs(n) * tan(halve);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;

    // ����
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    // ƽ�Ƶ�ԭ��
    trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2, 
        0, 0, 0, 1;

    projection = scale * trans * m;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);//��դ��ʼ��
    Eigen::Vector3f eye_pos = {0,0,5}; //�۲�����

    //������  ����������
    std::vector<Eigen::Vector3f> pos
    {
        {2, 0, -2}, {0, 2, -2}, {-2, 0, -2},
        {3.5, -1, -5}, {2.5, 1.5, -5}, {-1, 0.5, -5}
    };
    
    //012 345����ʾһ��������
    std::vector<Eigen::Vector3i> ind  { {0, 1, 2}, {3, 4, 5}};

    //���������εĶ�����ɫ
    std::vector<Eigen::Vector3f> cols
    {
        {217.0, 238.0, 185.0}, {217.0, 238.0, 185.0}, {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}
    };

    //�洢�����εĶ��� �±� ��ɫ��Ϣ
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;
    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);
        return 0;
    }

    while(key != 27)
    {
        //������ɫ����Ȼ���
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //����mvp�任����
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //����ͼ��
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image.png", image);
        key = cv::waitKey(10);

        std::cout << "frame count :  " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on