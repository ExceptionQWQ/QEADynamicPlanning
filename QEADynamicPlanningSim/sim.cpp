#include "sim.h"
#include "ransac.h"
#include "gradient.h"
#include "qobj.h"

int main(int argc, char** argv)
{
    srand(time(0));

  
    QOBJ qobj1(100, 100, 100);
    QOBJ qobj2(-100, 300, 300);
    QOBJ qobj3(-400, 200, 400);
    
    cv::Mat field = GenerateScalarField({qobj1, qobj2, qobj3}, cv::Size(512, 512));
    cv::Mat fieldView = GetGradientViewFromScalarField(field, {0, 255, 0});

    cv::imshow("field", field);
    cv::imshow("fieldView", fieldView);
    cv::waitKey(0);
    return 0;
}