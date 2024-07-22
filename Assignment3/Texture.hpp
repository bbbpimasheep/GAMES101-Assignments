//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include <math.h>
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data, image_data_down;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;

        cv::pyrDown(image_data, image_data_down);
        cv::pyrDown(image_data_down, image_data_down);
        width_down = width / 4;
        height_down = height / 4;
    }

    int width, height, width_down, height_down;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width_down, v_img = (1 - v) * height_down,
              u_img_round = std::floor(u_img), v_img_round = std::floor(v_img);

        // auto color = image_data_down.at<cv::Vec3b>(v_img, u_img);
        
        float u_0, v_0, s, t;
        float u_error = u_img - u_img_round, v_error = v_img - v_img_round;
        if (u_error <= 0.5 && v_error <= 0.5)
        {
            u_0 = u_img_round - 1.0f;
            v_0 = v_img_round - 1.0f;
        }
        else if (u_error > 0.5 && v_error <= 0.5)
        {
            u_0 = u_img_round;
            v_0 = v_img_round - 1.0f;
        }
        else if (u_error <= 0.5 && v_error > 0.5)
        {
            u_0 = u_img_round - 1.0f;
            v_0 = v_img_round;
        }
        else
        {
            u_0 = u_img_round;
            v_0 = v_img_round;
        }

        s = u_img - u_0 - 0.5f; t = v_img - v_0 - 0.5f;

        auto color_00 = image_data_down.at<cv::Vec3b>(std::max(0.0f, v_0), std::max(0.0f, u_0));
        auto color_10 = image_data_down.at<cv::Vec3b>(std::max(0.0f, v_0), std::max(0.0f, u_0+1));
        auto color_01 = image_data_down.at<cv::Vec3b>(std::max(0.0f, v_0+1), std::max(0.0f, u_0));
        auto color_11 = image_data_down.at<cv::Vec3b>(std::max(0.0f, v_0+1), std::max(0.0f, u_0+1));

        auto color_0 = lerp(s, color_00, color_10); // color_00 + (color_10 - color_00) * s;
        auto color_1 = lerp(s, color_01, color_11); // color_01 + (color_11 - color_01) * s;

        auto color = lerp(t, color_0, color_1); // color_0 + (color_1 - color_0) * t;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    cv::Vec3b lerp(float ratio, cv::Vec3b color1, cv::Vec3b color2)
    {
        return cv::Vec3b((1.0f-ratio) * color1[0] + ratio * color2[0], 
                         (1.0f-ratio) * color1[1] + ratio * color2[1], 
                         (1.0f-ratio) * color1[2] + ratio * color2[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
