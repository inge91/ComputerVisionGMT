#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "camera_calibration.h"


int main(int argc, char* argv[])
{
	std::cout<<"Performing Calibration\n";
	startCalibration();


}