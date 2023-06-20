#include "sim.h"
#include "ransac.h"
#include "gradient.h"

int main(int argc, char** argv)
{
    srand(time(0));

    cv::Mat view = cv::Mat::zeros(cv::Size(512, 512), CV_32FC1);

    //在中心放一个电荷
    double cx = 256, cy = 256;
    for (int x = 0; x < view.cols; ++x) {
        for (int y = 0; y < view.rows; ++y) {
            double phi = 40000 / (std::pow(cx - x, 2) + std::pow(cy - y, 2) + 1);
            view.at<float>(y, x) = (float)phi;
        }
    }

    cv::Mat fieldView = GetVectorFieldView(view, {0, 255, 0});

    cv::imshow("view", view);
    cv::imshow("fieldView", fieldView);
    cv::waitKey(0);
    return 0;
}