#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    const auto radian = rotation_angle * MY_PI / 180;
    const auto sin_v = sin(radian);
    const auto cos_v = cos(radian);
    model(0, 0) = cos_v;
    model(0, 1) = -sin_v;

    model(1, 0) = sin_v;
    model(1, 1) = cos_v;

    model(2, 2) = 0;
    return model;
}

// 绕任意轴旋转
Eigen::Matrix4f get_rotation_matrix(float rotation_angle, Eigen::Vector3f axis, Eigen::Vector3f start)
{
    // 移动到原点
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = -start.x();
    translate(1, 3) = -start.y();
    translate(2, 3) = -start.z();
    // 旋转
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f N = Eigen::Matrix4f::Identity();
    N << 0, -axis.z(), axis.y(),0,
        axis.z(), 0, -axis.x(),0,
        -axis.y(), axis.x(), 0,0,
        0, 0, 0, 0;
    const auto alpha = rotation_angle * MY_PI / 180.0;
    Eigen::Vector4f axis_4 = Eigen::Vector4f({axis.x(), axis.y(), axis.z(), 0});
    rotation = cos(alpha) * I + (1 - cos(alpha)) * (axis_4 * axis_4.transpose())  + sin(alpha) * N;
    // 撤回
    Eigen::Matrix4f rollback = Eigen::Matrix4f::Identity();
    rollback(0, 3) = start.x();
    rollback(1, 3) = start.y();
    rollback(2, 3) = start.z();
    Eigen::Matrix4f res =  rollback * rotation * translate;
    res(3,3) = 1;
    std::cout << rollback << std::endl;
    std::cout << translate << std::endl;
    return res;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    return get_rotation_matrix(rotation_angle,
                               Eigen::Vector3f{0, 1, 0},
                               Eigen::Vector3f{0, 0, -2});
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    const auto tan_v = tan(eye_fov * MY_PI / 180);
    const auto top = tan_v * zNear;
    const auto bottom = -top;

    const auto left = top * aspect_ratio;
    const auto right = -left;

    // 统一近平面和远平面
    const auto planeDistance = fabs(zFar - zNear);
    scale(0, 0) = planeDistance;
    scale(1, 1) = planeDistance;
    scale(2, 2) = zNear + zFar;
    scale(2, 3) = -zNear * zFar;
    scale(3, 2) = 1;
    scale(3, 3) = 0;

    // 正交投影
    Eigen::Matrix4f orthographicProjection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    /**
     * 1. 长方体中心移动到原点
     * 2. 长宽高设置为[-1, 1]
     */
    translate(3, 0) = -(left + right) / 2;
    translate(3, 1) = -(bottom + top) / 2;
    translate(3, 2) = -(zNear + zFar) / 2;

    orthographicProjection(0, 0) = 2 / (-(top - bottom));
    orthographicProjection(1, 1) = 2 / (-(right - left));
    orthographicProjection(2, 2) = 2 / (-(zNear - zFar));
    return orthographicProjection * translate * scale;
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

    Eigen::Vector3f eye_pos = {0, 0, 5};
    // 三个顶点
    // 三个顶点的索引值
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // 赋值给光栅化器
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