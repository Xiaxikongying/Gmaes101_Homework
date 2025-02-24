#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

/*
������ҵ����������дһ����ת�����һ��͸��ͶӰ���󡣸�����ά��������
v0(2.0, 0.0, ?2.0), v1(0.0, 2.0, ?2.0), v2(?2.0, 0.0, ?2.0),
����Ҫ���������������任Ϊ��Ļ���겢����Ļ�ϻ��Ƴ���Ӧ���߿�������
(�ڴ������У������Ѿ��ṩ�� draw_triangle ������������ֻ��Ҫȥ�����任���󼴿�)��
�����֮��������Ҫ����ģ�͡���ͼ��ͶӰ���ӿڵȱ任������������ʾ����Ļ�ϡ�
���ṩ�Ĵ������У�����������ģ�ͱ任��ͶӰ�任�Ĳ��ָ���ȥ��ɡ�
*/
constexpr float MY_PI = 3.1415926;

// ��ͼ�任---->�������������һ���ƶ�������ƶ���ԭ�㣩�����λ�ò���
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    // �������ƽ�Ƶ�ԭ��
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;
    view = translate * view;
    return view;
}

// ģ�ͱ任---->model������Ϊ�˳�ʼ�������λ������
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // ��ת����a
    float a = rotation_angle / 180.0 * MY_PI;
    // ��z����ת�� ��ת����
    model << cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

/// @brief ͶӰ����
/// @param eye_fov �ӳ���
/// @param aspect_ratio ��߱�
/// @param zNear ��ƽ��
/// @param zFar Զƽ��
/// @return
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
    // M_persp2Ortho��͸��ͶӰ����  ��һ����׶��ѹ��һ��������
    Eigen::Matrix4f m;
    m << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // M_ortho������ͶӰ����    ƽ�Ƶ�ԭ�� + ����
    Eigen::Matrix4f trans, scale;
    float havle = (eye_fov / 2) * MY_PI / 180.0; // ���㻡��
    float t = abs(n) * tan(havle);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;

    // ƽ�Ƶ�ԭ��
    trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    // ����
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
    bool command_line = false; // �Ƿ��������
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

    // ����һ����դ��
    rst::rasterizer r(700, 700);
    // ��ʼ���۲��  Ĭ�ϳ���-Z��
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // �����ζ���
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // std::vector<Eigen::Vector3f> pos{{2, 1, -2}, {1, 2, -2}, {-2, 1, -2}};

    // �Զ�����飨����ֻ��һ�飩
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // ����������Ϣ�����դ����
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;         // ��������
    int frame_count = 0; // ֡��

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

    // ��Ⱦѭ��
    while (key != 27) // Esc
    {
        // ���֡����ͼ��׼�����»���
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // ���դ������ ģ�ͱ任����
        r.set_model(get_model_matrix(angle));
        // ���դ������ �ӽǱ任����
        r.set_view(get_view_matrix(eye_pos));
        // ���դ������ ͶӰ�任����
        r.set_projection(get_projection_matrix(60, 1, 0.1, 50));

        // ��ʼ��Ⱦ
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // ʹ��opencv��ʾͼ��
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count=  " << frame_count++ << '\n';

        // ���� ��z�˶�
        if (key == 'z') // ����
        {
            angle += 10;
        }
        else if (key == 'c') // ����
        {
            angle -= 10;
        }

        // �۲�����仯

        if (key == 'a') // ����
        {
            eye_pos[0] += 1;
        }
        else if (key == 'd') // ����
        {
            eye_pos[0] -= 1;
        }
        else if (key == 'w') // ��
        {
            eye_pos[1] -= 1;
        }
        else if (key == 's') // ��
        {
            eye_pos[1] += 1;
        }
        else if (key == 'i') // ��
        {
            eye_pos[2] -= 1;
        }
        else if (key == 'k') // С
        {
            eye_pos[2] += 1;
        }
    }

    return 0;
}
