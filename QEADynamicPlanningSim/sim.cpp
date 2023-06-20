#include "sim.h"
#include "ransac.h"
#include "gradient.h"
#include "qobj.h"

int main(int argc, char** argv)
{
    srand(time(0));

  
    QOBJ qobj1(100, 100, 100);
    QOBJ qobj2(-400, 200, 400);

    cv::Mat fieldView = GetGradientViewFromQOBJ({qobj1, qobj2}, cv::Size(512, 512), cv::Scalar(255, 255, 0));

    cv::imshow("fieldView", fieldView);
    cv::waitKey(0);
    return 0;
}