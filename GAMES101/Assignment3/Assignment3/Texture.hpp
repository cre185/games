//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::max(std::min(u,1.0f),0.0f);
        v = std::max(std::min(v,1.0f),0.0f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        // use bilinear interpolation
        int cx = std::floor(u_img+0.5f);
        int cy = std::floor(v_img+0.5f);
        auto c00 = image_data.at<cv::Vec3b>(cy+0.5f, cx-0.5f);
        auto c01 = image_data.at<cv::Vec3b>(cy+0.5f, cx+0.5f);
        auto c10 = image_data.at<cv::Vec3b>(cy-0.5f, cx-0.5f);
        auto c11 = image_data.at<cv::Vec3b>(cy-0.5f, cx+0.5f);
        float s = u_img-cx+0.5f;
        float t = v_img-cy+0.5f;
        auto c0 = (1-s)*c00+s*c01;
        auto c1 = (1-s)*c10+s*c11;
        auto color = (1-t)*c1+t*c0;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
