#include <opencv/cv.h>      // include it to used Main OpenCV functions.
#include <opencv/highgui.h> //include it to use GUI functions.

using namespace cv;

int main(int argc, char** argv)
{
	
	if(argc < 1)
	{
		std::cout<<"Error: No image given"<<std::endl;
		exit(1);
	}

	Mat img = imread(argv[1]);

	// Get image size
	Size s = img.size();
	int centerX = s.width/2;
	int centerY = s.height/2;

	rectangle(img, Point(centerX - 50,centerY-50), Point(centerX + 50, centerY+50), Scalar(255, 255, 255));
	
	imshow("Assignment1", img);
	cvWaitKey(0);
	


	
	cvDestroyWindow("Assignment1");

    return 0;
}

