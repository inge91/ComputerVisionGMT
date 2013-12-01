/*
 * Camera.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#include "Camera.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

vector<Point>* Camera::_BoardCorners;  // marked checkerboard corners

Camera::Camera(const string &dp, const string &cp, const int id) :
		_data_path(dp), _cam_prop(cp), _id(id)
{
	_initialized = false;

	_fx = 0;
	_fy = 0;
	_px = 0;
	_py = 0;
	_frames = 0;
}

Camera::~Camera()
{
}

/**
 * Initialize this camera
 */
bool Camera::initialize()
{
	_initialized = true;

	Mat bg_image;
	if (General::fexists(_data_path + General::BackgroundImageFile))
	{
		bg_image = imread(_data_path + General::BackgroundImageFile);
		if (bg_image.empty())
		{
			cout << "Unable to read: " << _data_path + General::BackgroundImageFile;
			return false;
		}
	}
	else if (General::fexists(_data_path + General::BackgroundVideoFile))
	{
		VideoCapture video = VideoCapture(_data_path + General::BackgroundVideoFile);
		assert(video.isOpened());
		video >> bg_image;
		if (bg_image.empty())
		{
			cout << "Unable to read: " << _data_path + General::BackgroundVideoFile;
			return false;
		}
	}
	else
	{
		cout << "Unable to find background video: " << _data_path + General::BackgroundVideoFile;
		return false;
	}
	assert(!bg_image.empty());

	// Disect the background image in HSV-color space
	Mat bg_hsv_im;
	cvtColor(bg_image, bg_hsv_im, CV_BGR2HSV);
	split(bg_hsv_im, _bg_hsv_channels);

	// Open the video for this camera
	_video = VideoCapture(_data_path + General::VideoFile);
	assert(_video.isOpened());

	// Assess the image size
	_plane_size.width = _video.get(CV_CAP_PROP_FRAME_WIDTH);
	_plane_size.height = _video.get(CV_CAP_PROP_FRAME_HEIGHT);
	assert(_plane_size.area() > 0);

	// Get the amount of video frames
	_video.set(CV_CAP_PROP_POS_AVI_RATIO, 1);  // Go to the end of the video; 1 = 100%
	_frames = _video.get(CV_CAP_PROP_POS_FRAMES);
	assert(_frames > 1);

	_video.set(CV_CAP_PROP_POS_AVI_RATIO, 0);  // Go back to the start

	// Read the camera properties (XML)
	FileStorage fs;
	fs.open(_data_path + _cam_prop, FileStorage::READ);
	if (fs.isOpened())
	{
		Mat cam_mat, dis_coe, rot_val, tra_val;
		fs["CameraMatrix"] >> cam_mat;
		fs["DistortionCoeffs"] >> dis_coe;
		fs["RotationValues"] >> rot_val;
		fs["TranslationValues"] >> tra_val;

		cam_mat.convertTo(_camera_matrix, CV_32F);
		dis_coe.convertTo(_distortion_coeffs, CV_32F);
		rot_val.convertTo(_rotation_values, CV_32F);
		tra_val.convertTo(_translation_values, CV_32F);

		fs.release();

		_fx = *_camera_matrix.ptr<float>(0, 0);
		_fy = *_camera_matrix.ptr<float>(1, 1);
		_px = *_camera_matrix.ptr<float>(0, 2);
		_py = *_camera_matrix.ptr<float>(1, 2);
	}
	else
	{
		cerr << "Unable to locate: " << _data_path << _cam_prop << endl;
		_initialized = false;
	}

	Mat rotation_matrix = getCamLoc(_id, _rotation_values, _translation_values, _camera_location);
	invertRt(rotation_matrix, _translation_values, _inverse_rt);
	camPtInWorld();

	return _initialized;
}

/**
 * Set and return the next frame from the video
 */
Mat& Camera::advanceVideoFrame()
{
	_video >> _frame;
	return _frame;
}

/**
 * Set the video location to the given frame number
 */
void Camera::setVideoFrame(int frame_number)
{
	_video.set(CV_CAP_PROP_POS_FRAMES, frame_number);
}

/**
 * Set and return frame of the video location at the given frame number
 */
Mat& Camera::getVideoFrame(int frame_number)
{
	setVideoFrame(frame_number);
	return advanceVideoFrame();
}

/**
 * Handle mouse events
 */
void Camera::onMouse(int event, int x, int y, int flags, void* param)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		if (flags == (EVENT_FLAG_LBUTTON + EVENT_FLAG_CTRLKEY))
		{
			if (!_BoardCorners->empty())
			{
				cout << "Removed corner " << _BoardCorners->size() << "... (use Click to add)" << endl;
				_BoardCorners->pop_back();
			}
		}
		else
		{
			_BoardCorners->push_back(Point(x, y));
			cout << "Added corner " << _BoardCorners->size() << "... (use CTRL+Click to remove)" << endl;
		}
		break;
	default:
		break;
	}
}

/**
 * - Determine the camera's extrinsics based on a checkerboard image and the camera intrinsics
 * - Allows for hand pointing the checkerboard corners
 */
bool Camera::detExtrinsics(const string &data_path, const string &checker_vid_fname, const string &intr_filename,
		const string &out_fname)
{
	cout << "Estimate camera extrinsics by hand..." << endl;

	int cb_width = 0, cb_height = 0;
	int cb_square_size = 0;

	// Read the checkerboard properties (XML)
	FileStorage fs;
	fs.open(data_path + ".." + string(PATH_SEP) + General::CBConfigFile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["CheckerBoardWidth"] >> cb_width;
		fs["CheckerBoardHeight"] >> cb_height;
		fs["CheckerBoardSquareSize"] >> cb_square_size;
	}
	fs.release();

	const Size board(cb_width, cb_height);
	const int square_size = cb_square_size;  // Actual size of the checkerboard square in millimeters

	Mat camera_matrix, distortion_coeffs;
	fs.open(data_path + intr_filename, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["CameraMatrix"] >> camera_matrix;
		fs["DistortionCoeffs"] >> distortion_coeffs;
		fs.release();
	}
	else
	{
		cerr << "Unable to read camera intrinsics from: " << data_path << intr_filename << endl;
		return false;
	}

	VideoCapture cap(data_path + checker_vid_fname);
	if (!cap.isOpened())
	{
		cerr << "Unable to open: " << data_path + checker_vid_fname << endl;
		return false;
	}

	// read first frame
	Mat frame;
	cap >> frame;

	_BoardCorners = new vector<Point>();

	namedWindow(MAIN_WINDOW, CV_WINDOW_KEEPRATIO);
	setMouseCallback(MAIN_WINDOW, onMouse);

	cout << "Now mark the " << board.area() << " interior corners of the checkerboard" << endl;
	Mat canvas;
	while ((int) _BoardCorners->size() < board.area())
	{
		canvas = frame.clone();

		if (!_BoardCorners->empty())
		{
			for (size_t c = 0; c < _BoardCorners->size(); c++)
			{
				circle(canvas, _BoardCorners->at(c), 4, Color_MAGENTA, 1, 8);
				if (c > 0) line(canvas, _BoardCorners->at(c), _BoardCorners->at(c - 1), Color_MAGENTA, 1, 8);
			}
		}

		int key = waitKey(10);
		if (key == 'q' || key == 'Q')
		{
			return false;
		}
		else if (key == 'c' || key == 'C')
		{
			_BoardCorners->pop_back();
		}

		imshow(MAIN_WINDOW, canvas);
	}
	assert((int ) _BoardCorners->size() == board.area());
	cout << "Marking finished!" << endl;

	Mat object_points = Mat::zeros(board.area(), 3, CV_32F);
	Mat image_points(board.area(), 2, CV_32F);

	// save the object points and image points
	for (int s = 0; s < board.area(); ++s)
	{
		*object_points.ptr<float>(s, 0) = s / board.width * square_size;
		*object_points.ptr<float>(s, 1) = s % board.height * square_size;
		*image_points.ptr<float>(s, 0) = _BoardCorners->at(s).x;
		*image_points.ptr<float>(s, 1) = _BoardCorners->at(s).y;
	}

	delete _BoardCorners;

	Mat rotation_values_d, translation_values_d;
	solvePnP(object_points, image_points, camera_matrix, distortion_coeffs, rotation_values_d, translation_values_d);
	Mat rotation_values, translation_values;
	rotation_values_d.convertTo(rotation_values, CV_32F);
	translation_values_d.convertTo(translation_values, CV_32F);

	fs.open(data_path + out_fname, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "CameraMatrix" << camera_matrix;
		fs << "DistortionCoeffs" << distortion_coeffs;
		fs << "RotationValues" << rotation_values;
		fs << "TranslationValues" << translation_values;
		fs.release();
	}
	else
	{
		cerr << "Unable to write camera intrinsics+extrinsics to: " << data_path << out_fname << endl;
		return false;
	}

	return true;
}

/**
 * Calculate the inverse of R/t
 */
void Camera::invertRt(const Mat &rot_mat, const Mat &trans_vals, Mat &inv_mat)
{
	// Xi = K[R/t]Xw
	// t = -RC
	_rt = Mat(4, 4, CV_32F);

	// [R|t]
	*_rt.ptr<float>(0, 0) = *rot_mat.ptr<float>(0, 0);
	*_rt.ptr<float>(0, 1) = *rot_mat.ptr<float>(0, 1);
	*_rt.ptr<float>(0, 2) = *rot_mat.ptr<float>(0, 2);
	*_rt.ptr<float>(0, 3) = *trans_vals.ptr<float>(0, 0);

	*_rt.ptr<float>(1, 0) = *rot_mat.ptr<float>(1, 0);
	*_rt.ptr<float>(1, 1) = *rot_mat.ptr<float>(1, 1);
	*_rt.ptr<float>(1, 2) = *rot_mat.ptr<float>(1, 2);
	*_rt.ptr<float>(1, 3) = *trans_vals.ptr<float>(0, 1);

	*_rt.ptr<float>(2, 0) = *rot_mat.ptr<float>(2, 0);
	*_rt.ptr<float>(2, 1) = *rot_mat.ptr<float>(2, 1);
	*_rt.ptr<float>(2, 2) = *rot_mat.ptr<float>(2, 2);
	*_rt.ptr<float>(2, 3) = *trans_vals.ptr<float>(0, 2);

	*_rt.ptr<float>(3, 0) = 0;
	*_rt.ptr<float>(3, 1) = 0;
	*_rt.ptr<float>(3, 2) = 0;
	*_rt.ptr<float>(3, 3) = 1;

	invert(_rt, inv_mat);
}

/**
 * Calculate the camera's location in the world
 */
Mat Camera::getCamLoc(int id, const Mat &rot_vals, const Mat &trans_vals, Point3f &cam_loc)
{
	Mat rotation;
	Rodrigues(rot_vals, rotation);

	Mat zeros_col = Mat::zeros(rotation.rows, 1, rotation.type());
	Mat zeros_row = Mat::zeros(1, rotation.cols, rotation.type());
	Mat one(1, 1, rotation.type(), Scalar::all(1));
	hconcat(rotation, zeros_col, rotation);
	hconcat(zeros_row, one, zeros_row);
	vconcat(rotation, zeros_row, rotation);
	assert(rotation.rows == 4 && rotation.cols == 4);

	float t_array[4][4] = { { 1.f, 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f, 0.f },
	                        { *trans_vals.ptr<float>(0, 0) * -1,
	                          *trans_vals.ptr<float>(0, 1) * -1,
	                          *trans_vals.ptr<float>(0, 2) * -1, 1.f } };
	Mat translation(4, 4, CV_32F, t_array);

	Mat camera_mat = translation * rotation;

	cam_loc = Point3d(*camera_mat.ptr<float>(0, 0) + *camera_mat.ptr<float>(3, 0),
			*camera_mat.ptr<float>(1, 1) + *camera_mat.ptr<float>(3, 1),
			*camera_mat.ptr<float>(2, 2) + *camera_mat.ptr<float>(3, 2));

	cout << "Camera " << id + 1 << " " << cam_loc.x << " " << cam_loc.y << " " << cam_loc.z << endl;

	return rotation;
}

/**
 * Calculate the camera's plane and fov in the 3D scene
 */
void Camera::camPtInWorld()
{
	_camera_plane.clear();
	_camera_plane.push_back(_camera_location);

	// clockwise four image plane corners
	// 1 image plane's left upper corner
	Point3f p1 = cam3DtoW3D(Point3f(-_px, -_py, (_fx + _fy) / 2));
	_camera_plane.push_back(p1);
	// 2 image plane's right upper conner
	Point3f p2 = cam3DtoW3D(Point3f(_plane_size.width - _px, -_py, (_fx + _fy) / 2));
	_camera_plane.push_back(p2);
	// 3 image plane's right bottom conner
	Point3f p3 = cam3DtoW3D(Point3f(_plane_size.width - _px, _plane_size.height - _py, (_fx + _fy) / 2));
	_camera_plane.push_back(p3);
	// 4 image plane's left bottom conner
	Point3f p4 = cam3DtoW3D(Point3f(-_px, _plane_size.height - _py, (_fx + _fy) / 2));
	_camera_plane.push_back(p4);

	// principal point on the image plane
	Point3f p5 = cam3DtoW3D(Point3f(_px, _py, (_fx + _fy) / 2));
	_camera_plane.push_back(p5);
}

/**
 * Convert a point on the camera image to a point in the world
 */
Point3f Camera::ptToW3D(const Point &point)
{
	return cam3DtoW3D(Point3f(float(point.x - _px), float(point.y - _py), (_fx + _fy) / 2));
}

/**
 * Convert a point on the camera to a point in the world
 */
Point3f Camera::cam3DtoW3D(const Point3f &cam_point)
{
	Mat Xc(4, 1, CV_32F);
	*Xc.ptr<float>(0, 0) = cam_point.x;
	*Xc.ptr<float>(1, 0) = cam_point.y;
	*Xc.ptr<float>(2, 0) = cam_point.z;
	*Xc.ptr<float>(3, 0) = 1;

	Mat Xw = _inverse_rt * Xc;

	return Point3f(*Xw.ptr<float>(0, 0), *Xw.ptr<float>(1, 0), *Xw.ptr<float>(2, 0));
}

/**
 * Projects points from the scene space to the image coordinates
 */
Point Camera::projectOnView(const Point3f &coords)
{
	Mat object_points = Mat::zeros(3, 3, CV_32F);
	*object_points.ptr<float>(0, 0) = coords.x;
	*object_points.ptr<float>(0, 1) = coords.y;
	*object_points.ptr<float>(0, 2) = coords.z;

	Mat image_points;
	projectPoints(object_points, _rotation_values, _translation_values, _camera_matrix, _distortion_coeffs, image_points);

	return Point((*image_points.ptr<Vec3f>(0, 0))[0], (*image_points.ptr<Vec3f>(0, 0))[1]);
}

} /* namespace nl_uu_science_gmt */
