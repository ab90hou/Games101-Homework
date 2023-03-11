#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

const static float MY_PI = 3.141592654;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
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
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    //终止条件
    if (control_points.size() == 1)
    {
        return cv::Point2f(control_points[0].x, control_points[0].y);
    }

    std::vector<cv::Point2f> new_control_points;
    for (int index = 0; index < control_points.size()-1; index++)
    {
        auto p1 = control_points[index];
        auto p2 = control_points[index + 1];
        auto new_p = (1 - t) * p1 + t * p2;
        new_control_points.emplace_back(new_p.x,new_p.y);
    }
        
    return recursive_bezier(new_control_points,t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm
    float max_distance = 2*std::sqrt(2);
    for (double t = 0.0; t < 1.0; t+=0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        for (float i = -1; i <= 1; i++)
        {
            for (float j = -1; j <= 1; j++)
            {
                float new_x, new_y;
                new_x = (int)point.x + i+0.5;
                new_y = (int)point.y + j+0.5;
                float ratio = 1-(std::sqrt(std::pow(new_x-point.x,2)+std::pow(new_y-point.y,2)))/max_distance;
                window.at<cv::Vec3b>(point.y + i, point.x + j)[1] = std::max((float)window.at<cv::Vec3b>(point.y + i, point.x + j)[1], 255*ratio);
            }
        }
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
            //naive_bezier(control_points, window);
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
