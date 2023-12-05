#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Vector4f model_rot = {0, 0, 0, 0};

float rot_sensitivity = 0.3f;
float mov_sensitivity = 0.3f;

Eigen::Vector4f eye_pos = {0, 0, 8, 0};
Eigen::Vector4f eye_dir = {0, 0, -1, 0}; // -z
Eigen::Vector4f up_dir = {0, 1, 0, 0}; // y
Eigen::Vector4f right_dir = {1, 0, 0, 0}; // x

Eigen::Matrix4f x_rotate_matrix(float x_angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float cos_angle = cos(x_angle / 180 * MY_PI);
    float sin_angle = sin(x_angle / 180 * MY_PI);
    model << 1, 0, 0, 0, 
             0, cos_angle, -sin_angle, 0,
             0, sin_angle, cos_angle, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f y_rotate_matrix(float y_angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float cos_angle = cos(y_angle / 180 * MY_PI);
    float sin_angle = sin(y_angle / 180 * MY_PI);
    model << cos_angle, 0, sin_angle, 0, 
             0, 1, 0, 0,
             -sin_angle, 0, cos_angle, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f z_rotate_matrix(float z_angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float cos_angle = cos(z_angle / 180 * MY_PI);
    float sin_angle = sin(z_angle / 180 * MY_PI);
    model << cos_angle, -sin_angle, 0, 0, 
             sin_angle, cos_angle, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f rotate_matrix(Vector4f &angle){
    return x_rotate_matrix(angle[0])*y_rotate_matrix(angle[1])*z_rotate_matrix(angle[2]);
}

Eigen::Matrix4f get_rotation(Vector4f _axis, float angle)
{
    Eigen::Vector3f axis = _axis.head<3>();
    Eigen::Matrix3f rodrigues_rotation = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    float cos_angle = cos(angle / 180 * MY_PI);
    float sin_angle = sin(angle / 180 * MY_PI);
    Eigen::Matrix3f cross_vertex = Eigen::Matrix3f::Identity();
    cross_vertex << 0, -axis.z(), axis.y(),
                    axis.z(), 0, -axis.x(),
                    -axis.y(), axis.x(), 0;

    rodrigues_rotation = cos_angle * I + (1 - cos_angle) * axis * axis.transpose() + sin_angle * cross_vertex;
    
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation << rodrigues_rotation(0, 0), rodrigues_rotation(0, 1), rodrigues_rotation(0, 2), 0,
                rodrigues_rotation(1, 0), rodrigues_rotation(1, 1), rodrigues_rotation(1, 2), 0,
                rodrigues_rotation(2, 0), rodrigues_rotation(2, 1), rodrigues_rotation(2, 2), 0,
                0, 0, 0, 1;
    return rotation;
}



Eigen::Matrix4f get_model_matrix(Vector4f &angle)
{
    /*
    about the three axis:
    the two axis parallel to screen will be:
        ^y
        |
    ----|--->x
        |
    and axis z vertical to your screen.
    the three angle represents the rotation angle around each axis.
    */
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    return rotate_matrix(angle);
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector4f eye_pos)
{
    Eigen::Vector4f a = right_dir;
    Eigen::Vector4f b = up_dir;
    Eigen::Vector4f c = eye_dir;
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view << a[0], a[1], a[2], 0,
            b[0], b[1], b[2], 0,
            -c[0], -c[1], -c[2], 0,
            0, 0, 0, 1;
    
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;
    
    view = view * translate;
    return view;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = zNear;
    float f = zFar;
    float t = -n * tan(eye_fov / 180 * MY_PI / 2);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
    persp_to_ortho << n, 0, 0, 0,
                      0, n, 0, 0,
                      0, 0, n + f, -n * f,
                      0, 0, 1, 0;
    
    Eigen::Matrix4f move_to_original = Eigen::Matrix4f::Identity();
    move_to_original << 1, 0, 0, -(r + l) / 2,
                        0, 1, 0, -(t + b) / 2,
                        0, 0, 1, -(n + f) / 2,
                        0, 0, 0, 1;

    Eigen::Matrix4f change_size = Eigen::Matrix4f::Identity();
    change_size << 2 / (r - l), 0, 0, 0,
                   0, 2 / (t - b), 0, 0,
                   0, 0, 2 / (n - f), 0,
                   0, 0, 0, 1;
    
    projection = change_size * move_to_original * persp_to_ortho;
    return projection;
}


void on_Mouse(int event, int x, int y, int flags, void* param) {
    static bool mouse_down=false;
    static int last_x = 0;
    static int last_y = 0;
	if (event == CV_EVENT_MOUSEMOVE){
        if(mouse_down){
            float delta_x = (last_x-x)*rot_sensitivity;
            float delta_y = (last_y-y)*rot_sensitivity;
            Eigen::Vector4f old_right_dir = right_dir;
            right_dir = (get_rotation(up_dir, delta_x)*right_dir).normalized();
            up_dir = (get_rotation(old_right_dir, delta_y)*up_dir).normalized();
            Vector3f eye_dir3 = eye_dir.head<3>();
            Vector3f right_dir3 = right_dir.head<3>();
            Vector3f up_dir3 = up_dir.head<3>();
            eye_dir3 = up_dir3.cross(right_dir3);
            eye_dir << eye_dir3[0], eye_dir3[1], eye_dir3[2], 0;
            last_x=x;
            last_y=y;
        }
    }
    else if (event == CV_EVENT_LBUTTONDOWN){
        mouse_down=true;
        last_x=x;
        last_y=y;
    }
    else if (event == CV_EVENT_LBUTTONUP){
        mouse_down=false;
    }
}

int main(int argc, const char** argv)
{
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        model_rot[0] = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(model_rot));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    std::cout<<get_rotation(Vector4f(1, 0, 1, 0), 45)<<std::endl;

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(model_rot));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::namedWindow("image");
        cv::setMouseCallback("image", on_Mouse);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        if (key == 'w') {
            eye_pos += eye_dir*mov_sensitivity;
        }
        else if (key == 's') {
            eye_pos -= eye_dir*mov_sensitivity;
        }
        else if (key == 'a') {
            eye_pos -= right_dir*mov_sensitivity;
        }
        else if (key == 'd') {
            eye_pos += right_dir*mov_sensitivity;
        }
    }
    return 0;
}
