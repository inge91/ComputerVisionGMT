/*
 * Reconstructor.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Reconstructor.h"
#include "FindingValues.h"
#include "Histogram.h"




// Histogram function
vector<Histogram> _histograms;
// returns the histograms
vector<Histogram>& getHistograms() 
{
	return _histograms;
}

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

/**
 * Voxel reconstruction class
 */
Reconstructor::Reconstructor(const vector<Camera*> &cs) :
		_cameras(cs)
{
	for (size_t c = 0; c < _cameras.size(); ++c)
	{
		if (_plane_size.area() > 0)
			assert(_plane_size.width == _cameras[c]->getSize().width && _plane_size.height == _cameras[c]->getSize().height);
		else
			_plane_size = _cameras[c]->getSize();
	}

	_step = 32;
	_size = 512;
	const size_t h_edge = _size * 4;
	const size_t edge = 2 * h_edge;
	_voxels_amount = (edge / _step) * (edge / _step) * (h_edge / _step);

	initialize();
}

/**
 * Free the memory of the pointer vectors
 */
Reconstructor::~Reconstructor()
{
	for (size_t c = 0; c < _corners.size(); ++c)
		delete _corners.at(c);
	for (size_t v = 0; v < _voxels.size(); ++v)
		delete _voxels.at(v);
}

/**
 * Create some Look Up Tables
 * 	- LUT for the scene's box corners
 * 	- LUT with a map of the entire voxelspace: point-on-cam to voxels
 * 	- LUT with a map of the entire voxelspace: voxel to cam points-on-cam
 */
void Reconstructor::initialize()
{
	const int h_edge = _size * 4;
	const int xL = -h_edge;
	const int xR = h_edge;
	const int yL = -h_edge;
	const int yR = h_edge;
	const int zL = 0;
	const int zR = h_edge;

	// Save the volume corners
	// bottom
	_corners.push_back(new Point3f((float) xL, (float) yL, (float) zL));
	_corners.push_back(new Point3f((float) xL, (float) yR, (float) zL));
	_corners.push_back(new Point3f((float) xR, (float) yR, (float) zL));
	_corners.push_back(new Point3f((float) xR, (float) yL, (float) zL));

	// top
	_corners.push_back(new Point3f((float) xL, (float) yL, (float) zR));
	_corners.push_back(new Point3f((float) xL, (float) yR, (float) zR));
	_corners.push_back(new Point3f((float) xR, (float) yR, (float) zR));
	_corners.push_back(new Point3f((float) xR, (float) yL, (float) zR));

	cout << "Initializing voxels";

	// Acquire some memory for efficiency
	_voxels.resize(_voxels_amount);

#ifdef PARALLEL_PROCESS
#pragma omp parallel for //schedule(static, 1)
#endif
	for (int z = zL; z < zR; z += _step)
	{
		cout << "." << flush;

		for (int y = yL; y < yR; y += _step)
		{
			for (int x = xL; x < xR; x += _step)
			{
				Voxel* voxel = new Voxel;
				voxel->x = x;
				voxel->y = y;
				voxel->z = z;
				voxel->camera_projection = vector<Point>(_cameras.size());
				voxel->valid_camera_projection = vector<int>(_cameras.size(), 0);

				const int zp = ((z - zL) / _step);
				const int yp = ((y - yL) / _step);
				const int xp = ((x - xL) / _step);
				const int plane_y = (yR - yL) / _step;
				const int plane_x = (xR - xL) / _step;
				const int plane = plane_y * plane_x;
				const int p = zp * plane + yp * plane_x + xp;  // The voxel's index

				for (size_t c = 0; c < _cameras.size(); ++c)
				{
					Point point = _cameras[c]->projectOnView(Point3f((float) x, (float) y, (float) z));

					// Save the pixel coordinates 'point' of the voxel projections on camera 'c'
					voxel->camera_projection[(int) c] = point;

					if (point.x >= 0 && point.x < _plane_size.width && point.y >= 0 && point.y < _plane_size.height)
						voxel->valid_camera_projection[(int) c] = 1;
				}

				//'p' is not critical as it's unique
				_voxels[p] = voxel;
			}
		}
	}
	
	cout << "done!" << endl;	
}

/**
 * Count the amount of camera's each voxel in the space appears on,
 * if that amount equals the amount of cameras, add that voxel to the
 * visible_voxels vector
 *
 * Optimized by inverting the process (iterate over voxels instead of camera pixels for each camera)
 */
int frame_no  = 0;
void Reconstructor::update()
{
	_visible_voxels.clear();

#ifdef PARALLEL_PROCESS
#pragma omp parallel for //schedule(static, 1)
#endif
	for (size_t v = 0; v < _voxels_amount; ++v)
	{
		int camera_counter = 0;
		Voxel* voxel = _voxels[v];

		for (size_t c = 0; c < _cameras.size(); ++c)
		{
			if (voxel->valid_camera_projection[c])
			{
				const Point point = voxel->camera_projection[c];

				//If there's a white pixel on the foreground image at the projection point, add the camera
				if (_cameras[c]->getForegroundImage().at<uchar>(point) == 255) ++camera_counter;
			}
		}

		// If the voxel is present on all cameras
		if (camera_counter == _cameras.size())
		{
#ifdef PARALLEL_PROCESS
#pragma omp critical //push_back is critical
#endif
			_visible_voxels.push_back(voxel);
		}
	}


	// The very first clustering step
	if (frame_no == 0)
	{
		kMeans(_visible_voxels, 4,  _centroids, _clusters);
		//Check the centroid distances to eachother
		while(!check_centroids(_centroids))
		{
			kMeans(_visible_voxels, 4,  _centroids, _clusters);
		}
		cout<<_clusters[0].size()<<endl; 
		cout<<_clusters[1].size()<<endl; 
		cout<<_clusters[2].size()<<endl; 
		cout<<_clusters[3].size()<<endl; 

		// Fill all histograms givewn current clustering
		Histogram h1(_clusters[0], _cameras, frame_no);
		Histogram h2(_clusters[1], _cameras, frame_no);
		Histogram h3(_clusters[2], _cameras, frame_no);
		Histogram h4(_clusters[3], _cameras, frame_no);

		// Store in the histogram vector
		_histograms.push_back(h1);
		_histograms.push_back(h2);
		_histograms.push_back(h3);
		_histograms.push_back(h4);
		// Go to following frame
		frame_no +=1;
		for(int i = 0; i < 4; i ++)
		{
			_histograms[i].print_histogram();
		}
	}

	// All other frame steps we use the histograms to determine where
	// each voxel belongs to
	else
	{
	
		// make current frame hsv format for each camera
		cv::Mat frame_rgb0 = _cameras[0]->getVideoFrame(frame_no);
		cv::Mat frame_rgb1 = _cameras[1]->getVideoFrame(frame_no);
		cv::Mat frame_rgb2 = _cameras[2]->getVideoFrame(frame_no);
		cv::Mat frame_rgb3 = _cameras[3]->getVideoFrame(frame_no);
		cv::Mat frame_hsv0;
		cv::Mat frame_hsv1;
		cv::Mat frame_hsv2;
		cv::Mat frame_hsv3;
		cv::cvtColor(frame_rgb0,frame_hsv0, CV_RGB2HSV);
		cv::cvtColor(frame_rgb1,frame_hsv1, CV_RGB2HSV);
		cv::cvtColor(frame_rgb2,frame_hsv2, CV_RGB2HSV);
		cv::cvtColor(frame_rgb3,frame_hsv3, CV_RGB2HSV);
		std::vector<cv::Mat> hsv;
		hsv.push_back(frame_hsv0);
		hsv.push_back(frame_hsv1);
		hsv.push_back(frame_hsv2);
		hsv.push_back(frame_hsv3);
		// remove all voxels added to histograms
		for (int i = 0; i < _histograms.size(); i++)
		{
			_histograms[i].remove_voxels();
		}

		// We now can label voxels using the color histograms.
		for(int i = 0; i < _visible_voxels.size(); i ++)
		{
			// get colour from voxel 
			int h = Histogram::get_colour(_visible_voxels[i], _cameras, hsv);
			
			double match = 0;
			int bin = 0;
			// Calculate the distance to each histogram, then apply to the closest match
			for(int j = 0; j < _histograms.size(); j++)
			{
				double current_match = _histograms[j].calculate_distance(h);

				//cout<<"Distance value for cluster "<< j<< " : "<< current_match<<endl;
				if(current_match > match)
				{
					//cout<<"Distance is highter than before"<<endl;
					bin = j;
					match = current_match;
				}
				
			}

			//cout<<"Winner is cluster : "<< bin<<endl;
			// Add voxel to this histogram
			_histograms[bin].add_voxel(_visible_voxels[i]);
		}

		for(int i = 0; i < _histograms.size(); i++)//_histograms.size(); i ++)
		{
		
			// Calculate centroid
			_histograms[i].calculate_centroid();
			
			// remove voxels
			_histograms[i].remove_voxels();
		}
		


		// Redistribute voxels over centroids()
		for(int i = 0; i < _visible_voxels.size(); i ++)
		{
			int cluster = 9;
			double min = 999999;
			
			// calculate the distance to each centroid.
			for(int j = 0; j < _histograms.size(); j++)
			{
				double distance = _histograms[j].calculate_centroid_distance(_visible_voxels[i]);
				if (distance < min)
				{
					min = distance;
					cluster = j;
				}
			}
			// Add the voxel to the right cluster
			_histograms[cluster].add_voxel(_visible_voxels[i]);	
		}

		for(int i = 0 ; i < _histograms.size(); i++)
		{
			cout<<_histograms[i].voxel_list.size()<<endl;
		}
		cout<<"\n"<<endl;
		// update frame
		frame_no += 1;
	}
}
// Draws the voxels contained in histograms
void Reconstructor::draw_voxels()
{
	for (size_t v = 0; v < _histograms.size(); v++)
	{
		// Determine the color to draw in
		switch(v)
		{
		case 0:
			glColor4f(0.5,0,0,0.5);
			break;

		case 1:
			glColor4f(0, 0.5, 0, 0.5f);
			break;

		case 2:
			glColor4f(0, 0, 0.5, 0.5f);
			break;

		case 3:
			glColor4f(0, 0.5, 0.5, 0.5f);
			break;
		}
		for(int i = 0; i <	_histograms[v].voxel_list.size(); i++)
		{
			glVertex3f((GLfloat) _histograms[v].voxel_list[i]->x, (GLfloat) _histograms[v].voxel_list[i]->y, (GLfloat) _histograms[v].voxel_list[i]->z);
		}
	}
}


} /* namespace nl_uu_science_gmt */
