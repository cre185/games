#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Vector4f model_rot = {0, 140, 0, 0};

float rot_sensitivity = 0.3f;
float mov_sensitivity = 0.3f;

Eigen::Vector4f eye_pos = {0, 0, 10, 1};
Eigen::Vector4f eye_dir = {0, 0, -1, 0}; // -z
Eigen::Vector4f up_dir = {0, 1, 0, 0}; // y
Eigen::Vector4f right_dir = {1, 0, 0, 0}; // x

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

auto l1 = light{{20, 20, 20}, {500, 500, 500}};
auto l2 = light{{-20, 20, 0}, {500, 500, 500}};
std::vector<light> lights = {l1, l2};

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
    Eigen::Matrix4f rotation = rotate_matrix(angle);

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

Eigen::Matrix4f get_view_matrix(Vector4f &eye_pos)
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

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection;
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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;
    
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        auto v = (eye_pos-point).normalized();
        auto l = light.position-point;
        auto l2 = l.dot(l);
        auto ln = l.normalized();
        auto n = normal.normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity/l2)*std::max(0.0f,(n.dot(ln)));
        auto h = (v+ln).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity/l2)*std::max(0.0f,std::pow((n.dot(h)),p));
        result_color += (ambient+diffuse+specular);
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f _eye_pos = eye_pos.head<3>();

    float p = 150;

    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        auto v = (_eye_pos-point).normalized();
        auto l = light.position-point;
        auto l2 = l.dot(l);
        auto ln = l.normalized();
        auto n = normal.normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity/l2)*std::max(0.0f,(n.dot(ln)));
        auto h = (v+ln).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity/l2)*std::max(0.0f,std::pow((n.dot(h)),p));
        result_color += (ambient+diffuse+specular);
    }

    return result_color * 255.f;
}


Eigen::Vector3f cool_to_warm_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f warm = Eigen::Vector3f(0.9, 0.6, 0.4);
    Eigen::Vector3f cold = Eigen::Vector3f(0.3, 0.3, 0.8);
    Eigen::Vector3f warm_direction = Eigen::Vector3f(1,1,1).normalized();

    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f _eye_pos = eye_pos.head<3>();

    float p = 150;

    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        auto v = (_eye_pos-point).normalized();
        auto l = light.position-point;
        auto l2 = l.dot(l);
        auto ln = l.normalized();
        auto n = normal.normalized();
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        float x = normal.x();
        float y = normal.y();
        float z = normal.z();
        Eigen::Vector3f t(x*y/std::sqrt(x*x+z*z),std::sqrt(x*x+z*z),z*y/std::sqrt(x*x+z*z));
        Eigen::Vector3f b=normal.cross(t);
        Eigen::Matrix3f TBN;
        TBN << t,b,normal;
        auto kw = (1.0+(TBN*n).normalized().dot(warm_direction))/2;
        kd = kw*warm+(1-kw)*cold;
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity/l2)*std::max(0.0f,(n.dot(ln)));

        auto h = (v+ln).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity/l2)*std::max(0.0f,std::pow((n.dot(h)),p));
        result_color += (ambient+diffuse+specular);
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
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
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t(x*y/std::sqrt(x*x+z*z),std::sqrt(x*x+z*z),z*y/std::sqrt(x*x+z*z));
    Eigen::Vector3f b=normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t,b,normal;

    auto u = payload.tex_coords.x();
    auto v = payload.tex_coords.y();
    auto w = payload.texture->width;
    auto h = payload.texture->height;
    auto dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    auto dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    Eigen::Vector3f ln{-dU, -dV, 1.0f};
    point += kn*normal*payload.texture->getColor(u, v).norm();
    Eigen::Vector3f n = (TBN*ln).normalized();
    Eigen::Vector3f result_color = {0, 0, 0};
    // todo here

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        auto v = (eye_pos-point).normalized();
        auto l = light.position-point;
        auto l2 = l.dot(l);
        auto ln = l.normalized();
        auto n = normal.normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity/l2)*std::max(0.0f,(n.dot(ln)));
        auto h = (v+ln).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity/l2)*std::max(0.0f,std::pow((n.dot(h)),p));
        result_color += (ambient+diffuse+specular);
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    Eigen::Vector3f normal = payload.normal;
    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t(x*y/std::sqrt(x*x+z*z),std::sqrt(x*x+z*z),z*y/std::sqrt(x*x+z*z));
    Eigen::Vector3f b=normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t,b,normal;

    auto u = payload.tex_coords.x();
    auto v = payload.tex_coords.y();
    auto w = payload.texture->width;
    auto h = payload.texture->height;
    auto dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    auto dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    Eigen::Vector3f ln{-dU, -dV, 1.0f};
    Eigen::Vector3f result_color = (TBN*ln).normalized();

    return result_color * 255.f;
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
            eye_dir3 = up_dir3.cross(right_dir3).normalized();
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
    std::vector<Triangle*> TriangleList;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = cool_to_warm_shader;

    if (argc >= 2)
    {
        if (std::string(argv[1]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (std::string(argv[1]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (std::string(argv[1]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (std::string(argv[1]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (std::string(argv[1]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(model_rot));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(model_rot));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
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
