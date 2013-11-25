#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "camera_calibration.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

// Retrieves matrix data from the xml file
Mat retrieve_data(String filename, String data_unit)
{

	FileStorage fs;
	fs.open(filename, FileStorage::READ);

	if(!fs.isOpened())
	{
		cout<<"Could not open camera data file\n";
		exit(1);
	}

	// Retrieve specific data from the xml file
	Mat data;
	fs[data_unit] >> data;

	fs.release();

	return data;

}

// Creates the extrinsic camera matrix
// for the ith image.
Mat construct_extrinsic(Mat data, int i)
{

	// Extract the angles and translations for x y and z
	// of the ith image
	Mat ImgParams = data.row(i);
	double x_r = ImgParams.at<double>(0);
	double y_r = ImgParams.at<double>(1);
	double z_r = ImgParams.at<double>(2);
	double x_t = ImgParams.at<double>(3);
	double y_t = ImgParams.at<double>(4);
	double z_t = ImgParams.at<double>(5);

	Mat rvec = Mat(3, 1, CV_64F);
	rvec.row(0).col(0)  = x_r;
	rvec.row(1).col(0)  = y_r;
	rvec.row(2).col(0)  = z_r;

	Mat rmat = Mat();
	Rodrigues(rvec, rmat);
	Matx33f rotation_matrix = rmat;

	// Construct the final extrensic parameter matrix
	Mat eParam = Mat(4,4, CV_64F);

	eParam.row(0).col(0) =  rmat.at<double>(0,0);
	eParam.row(0).col(1) =  rmat.at<double>(0,1); 
	eParam.row(0).col(2) =  rmat.at<double>(0,2);
	eParam.row(0).col(3) =  x_t;
	eParam.row(1).col(0) =  rmat.at<double>(1,0);
	eParam.row(1).col(1) =  rmat.at<double>(1,1);
	eParam.row(1).col(2) =  rmat.at<double>(1,2);
	eParam.row(1).col(3) =  y_t;
	eParam.row(2).col(0) =  rmat.at<double>(2,0);
	eParam.row(2).col(1) =  rmat.at<double>(2,1);
	eParam.row(2).col(2) =  rmat.at<double>(2,2);
	eParam.row(2).col(3) =  z_t;
	eParam.row(3).col(0) =  0;
	eParam.row(3).col(1) =  0;
	eParam.row(3).col(2) =  0;
	eParam.row(3).col(3) =  1;

	return eParam;

}

// Create a vector from 3 input numbers)
Mat createVector(double x, double y, double z, bool homo=1)
{
	Mat vector;
	if (homo)
	{
		vector = Mat(4, 1, CV_64F);
		vector.row(0).col(0) = x;
		vector.row(1).col(0) = y;
		vector.row(2).col(0) = z;
		vector.row(3).col(0) = 1;
	}
	else{

		vector = Mat(3, 1, CV_64F);
		vector.row(0).col(0) = x;
		vector.row(1).col(0) = y;
		vector.row(2).col(0) = z;
	}
	return vector;
}

// Creates point data type from vector
Point vector2Point(Mat twodVec)
{
	Point p;
	p.x = twodVec.at<double>(0);
	p.y = twodVec.at<double>(1);
	return p;
}

// transfoms world coordinates to image coordinate
Point worldToImage(Mat intr, Mat extr, Mat coordW)
{
	if(!(coordW.rows == 4 && coordW.cols == 1))
	{
		cout<<"Vector is not of the right size";
		exit(-1);
	}

	// Create camera coordinates
	Mat coordC4 =  extr * coordW;

	// Drop the extra dimension
	Mat coordC3 = createVector(coordC4.at<double>(0,0), coordC4.at<double>(1,0), coordC4.at<double>(2,0), 0);

	// Get image coordinates
	Mat coordI3 =  intr * coordC3;
	
	// Still need to devide by the Z cooridnate
	Mat coordI2 = Mat(2,1, CV_64F);
 
	coordI2.row(0).col(0) =  coordI3.at<double>(0,0) / coordI3.at<double>(2,0);
	coordI2.row(1).col(0) =  coordI3.at<double>(1,0) / coordI3.at<double>(2,0);

	return vector2Point(coordI2);
}



// Visualise the world axis on the image
Mat visualiseAxis(Mat cameraMatrix, Mat extrinsicParams, Mat image)
{
	Point originW = worldToImage(cameraMatrix, extrinsicParams, createVector(0,0,0));
	Point xW = worldToImage(cameraMatrix, extrinsicParams, createVector(200,0,0));
	Point yW = worldToImage(cameraMatrix, extrinsicParams, createVector(0,200,0));
	Point zW = worldToImage(cameraMatrix, extrinsicParams, createVector(0,0,-200));

	line(image, originW, xW, Scalar(0, 0, 255), 3);
	line(image, originW, yW, Scalar(0, 255, 0), 3);
	line(image, originW, zW, Scalar(255, 0, 0), 3);

	return image;
}

// Draws the unit cube in an image
Mat drawUnitCube(Mat cameraMatrix, Mat extrinsicParams, Mat image)
{

	Point p1 = worldToImage(cameraMatrix, extrinsicParams, createVector(0,0,0));
	Point p2 = worldToImage(cameraMatrix, extrinsicParams, createVector(100,0,0));
	Point p3 = worldToImage(cameraMatrix, extrinsicParams, createVector(0,100,0));
	Point p4 = worldToImage(cameraMatrix, extrinsicParams, createVector(100, 100,0));

	Point p5 = worldToImage(cameraMatrix, extrinsicParams, createVector(0,0,-100));
	Point p6 = worldToImage(cameraMatrix, extrinsicParams, createVector(100,0,-100));
	Point p7 = worldToImage(cameraMatrix, extrinsicParams, createVector(0,100,-100));
	Point p8 = worldToImage(cameraMatrix, extrinsicParams, createVector(100, 100,-100));

	line(image, p1, p2, Scalar(255, 255, 255), 2);
	line(image, p1, p3, Scalar(255, 255, 255), 2);
	line(image, p2, p4, Scalar(255, 255, 255), 2);
	line(image, p3, p4, Scalar(255, 255, 255), 2);

	line(image, p5, p6, Scalar(255, 255, 255), 2);
	line(image, p5, p7, Scalar(255, 255, 255), 2);
	line(image, p6, p8, Scalar(255, 255, 255), 2);
	line(image, p7, p8, Scalar(255, 255, 255), 2);

	line(image, p1, p5, Scalar(255, 255, 255), 2);
	line(image, p2, p6, Scalar(255, 255, 255), 2);
	line(image, p3, p7, Scalar(255, 255, 255), 2);
	line(image, p4, p8, Scalar(255, 255, 255), 2);

	return image;
}


bool calibration = false;

int main(int argc, char* argv[])
{
	if(calibration)
	{
		cout<< "Performing camera calibration";
		startCalibration();
	}
	
	String cameraData = "out_camera_data.xml";
	String calibrationData = "test.xml";

	// Retieve important data from the xml files used by the calibration
	Mat cameraMatrix = retrieve_data(cameraData, "Camera_Matrix");
	Mat extrinsicComplete = retrieve_data(cameraData, "Extrinsic_Parameters");

	Mat image;
	
	for(int i = 1; i < 6; i++)
	{

		Mat eParam = construct_extrinsic(extrinsicComplete, i-1);
	
	
		image = imread("Images\\board"+ to_string(i) + ".jpg", CV_LOAD_IMAGE_COLOR);
		image = visualiseAxis(cameraMatrix, eParam, image);
		image = drawUnitCube(cameraMatrix, eParam, image);

		imshow("Assignment1", image);
		imwrite("\Images\\board"+ to_string(i) + "_calibr.jpg", image);
		waitKey(0);
	}

	// Wait for user input
	
	return(0);
}