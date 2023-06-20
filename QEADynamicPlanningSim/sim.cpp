#include "sim.h"
#include "ransac.h"

int main(int argc, char** argv)
{
    srand(time(0));

    cv::Mat view = GenerateCircleView();
    view = MakeNoise(view);
    auto ret = FindCircleRansac(ConvertViewToPoints(view));
    cv::Mat bgrView;
    cv::cvtColor(view, bgrView, cv::COLOR_GRAY2BGR);
    if (ret.has_value()) {
        DrawCircle(bgrView, ret->first.x, ret->first.y, ret->second, cv::Scalar(0, 0, 255));
    }
    cv::imshow("view", view);
    cv::imshow("bgrView", bgrView);
    cv::waitKey(0);
    return 0;
}