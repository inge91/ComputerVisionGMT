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


// Check if centroids have changed
bool Reconstructor::different_centroids(vector<Voxel> prev_centroids)
{

	for (int i = 0; i < prev_centroids.size(); i++)
	{
		Voxel v = prev_centroids[i];
		Voxel v2 = _histograms[i].centroid;
		if( v.x != v2.x || v.y != v2.y || v.z != v2.z)
		{
			return true;
		}
	}

	return false;

}

/**
 * Count the amount of camera's each voxel in the space appears on,
 * if that amount equals the amount of cameras, add that voxel to the
 * visible_voxels vector
 *
 * Optimized by inverting the process (iterate over voxels instead of camera pixels for each camera)
 */

bool first = true;
void Reconstructor::update(int frame_no)
{

	cout<<"frame no: "<< frame_no<<endl;
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
	if (first)
	{
		first = false;
		kMeans(_visible_voxels, 4,  _centroids, _clusters);
		//Check the centroid distances to eachother
		// If not right distance just execute Kmeans again
		while(!check_centroids(_centroids))
		{
			kMeans(_visible_voxels, 4,  _centroids, _clusters);
		}

		// For each camera determine the closes voxel in this specific frame
		//Voxel* camera_1 [640][480];
	
		_closest_voxel.resize(4);
		for(int i = 0; i < _closest_voxel.size(); i++)
		{
			_closest_voxel[i].resize(648);
			for(int j = 0; j < 648; j ++)
			{
				_closest_voxel[i][j].resize(488);

				for(int k = 0; k < 488; k++)
				{
					_closest_voxel[i][j][k] = NULL;
				}
			}
		}

		Histogram::find_closest_voxels(_closest_voxel, _cameras, _visible_voxels);
		
		// Fix occlusion given 
		//vector<Voxel*> closest_voxels;

		// Fill all histograms given current clustering
		Histogram h1(_clusters[0], _cameras, frame_no, _closest_voxel);
		Histogram h2(_clusters[1], _cameras, frame_no, _closest_voxel);
		Histogram h3(_clusters[2], _cameras, frame_no, _closest_voxel);
		Histogram h4(_clusters[3], _cameras, frame_no, _closest_voxel);

		// Store in the histogram vector
		_histograms.push_back(h1);
		_histograms.push_back(h2);
		_histograms.push_back(h3);
		_histograms.push_back(h4);
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

		cv::Mat frame_rgb0 = _cameras[0]->getFrame();
		cv::Mat frame_rgb1 = _cameras[1]->getFrame();
		cv::Mat frame_rgb2 = _cameras[2]->getFrame();
		cv::Mat frame_rgb3 = _cameras[3]->getFrame();
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

		// Reset all closest voxels back to 0
		for(int i = 0; i < _closest_voxel.size(); i++)
		{
			for(int j = 0; j < 648; j ++)
			{
				for(int k = 0; k < 488; k++)
				{
					_closest_voxel[i][j][k] = NULL;
				}
			}
		}
		// Refind closest foxels for this frame
		Histogram::find_closest_voxels(_closest_voxel, _cameras, _visible_voxels);


		// We now can label voxels using the color histograms.
		for(int i = 0; i < _visible_voxels.size(); i ++)
		{
			// get colour from voxel 
			int h = Histogram::get_colour(_visible_voxels[i], _cameras, hsv, _closest_voxel);
			// Skip distance calculation in case there are no projections to camera
			if(h == -1|| _visible_voxels[i]->z < 350)
			{
				continue;
			}
			double match = 0;
			int bin = 0;
			// Calculate the distance to each histogram, then apply to the closest match
			for(int j = 0; j < _histograms.size(); j++)
			{
				double current_match = _histograms[j].calculate_distance(h);

				if(current_match > match)
				{
					bin = j;
					match = current_match;
				}
				
			}
			// Add voxel to this histogram
			_histograms[bin].add_voxel(_visible_voxels[i]);
		}
		
		vector<Voxel> prev_centroids(4);

		bool initial = true;
		while(initial || different_centroids(prev_centroids))
		{
			initial = false;
			for(int i = 0; i < _histograms.size(); i++)//_histograms.size(); i ++)
			{
			
				prev_centroids[i] =  _histograms[i].centroid;
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
		}

	}
		cout<<"frame no: "<< frame_no<<endl;
		cout<<"\n"<<endl;
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
