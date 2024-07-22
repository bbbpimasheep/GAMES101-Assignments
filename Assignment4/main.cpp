#include <chrono>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void antialiasing(cv::Point2f &point, cv::Mat &window, int c) 
{
    float distance = cv::norm(point - cv::Point2f(floor(point.x)+0.5, floor(point.y)+0.5));

    const float u_bias[4] = {-0.5, 0.5, 0.0, 0.0},
                v_bias[4] = {0.0, 0.0, -0.5, 0.5};
    for (int i = 0; i < 4; i += 1)
    {
        float current_distance, origin_color, weight;
        int u = floor(point.x + u_bias[i]),
            v = floor(point.y + v_bias[i]);

        current_distance = cv::norm(point - cv::Point2f(u+0.5, v+0.5));
        origin_color = window.at<cv::Vec3b>(v, u)[c],
        weight = std::min(1.0f, distance / current_distance);
        
        window.at<cv::Vec3b>(v, u)[c] = std::max(origin_color, 255 * weight);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
        antialiasing(point, window, 2);
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto &p_0 = control_points[0];
    auto &p_1 = control_points[1];
    auto &p_2 = control_points[2];
    auto &p_3 = control_points[3];

    auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

    return cv::Point2f(point.x, point.y);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        antialiasing(point, window, 1);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
