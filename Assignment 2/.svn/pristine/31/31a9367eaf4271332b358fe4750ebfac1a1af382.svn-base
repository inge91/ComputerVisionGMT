/*
 * Reconstructor.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Reconstructor.h"

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
	_step = 40;
	_size = 480;

	const size_t h_edge = _size * _cameras.size();
	const size_t edge = 2 * h_edge;
	_voxels_amount = (edge / _step) * (edge / _step) * (h_edge / _step);

	for (size_t c = 0; c < _cameras.size(); ++c)
	{
		if (_plane_size.area() > 0)
			assert(_plane_size.width == _cameras[c]->getSize().width && _plane_size.height == _cameras[c]->getSize().height);
		else
			_plane_size = _cameras[c]->getSize();

		LookupTable* lut = new LookupTable;
		lut->camera_id = c;
		_luts.push_back(lut);
	}

	initialize();
}

/**
 * Free the memory of the pointer vectors
 */
Reconstructor::~Reconstructor()
{
	for (size_t c = 0; c < _corners.size(); ++c)
		delete _corners.at(c);
	for (size_t l = 0; l < _luts.size(); ++l)
		delete _luts.at(l);
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
	const int h_edge = (int) _size * (int) _cameras.size();
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
				voxel->visible_on = 0;

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
					if (point.x >= 0 && point.x < _plane_size.width && point.y >= 0 && point.y < _plane_size.height)
					{
						std::map<cv::Point, std::vector<int>, PointComparator>::iterator voxel_it = _luts[c]->voxel_space.find(point);
						if (voxel_it == _luts[c]->voxel_space.end())
						{
#ifdef PARALLEL_PROCESS
#pragma omp critical //push_back is critical
#endif
							// Save the voxel index 'p' to the LUT for camera 'c' at pixel coordinates 'point'
							_luts[c]->voxel_space[point].push_back(p);
						}
						else
						{
#ifdef PARALLEL_PROCESS
#pragma omp critical //push_back is critical
#endif
							voxel_it->second.push_back(p);
						}

						// Save the pixel coordinates 'point' of the voxel projections on camera 'c'
						voxel->pixel_on_views[(int) c] = point;
					}
				}

				//'p' is not critical as it's unique
				_voxels[p] = voxel;
//#pragma omp critical //push_back is critical
//				_voxels.push_back(voxel);
			}
		}

		for (size_t c = 0; c < _cameras.size(); ++c)
			assert(_luts[c]->voxel_space.size() > 1);
	}

	cout << "done!" << endl;
}

/**
 * Count the amount of camera's each voxel in the space appears on,
 * if that amount equals the amount of cameras, add that voxel to the
 * visible_voxels vector
 */
void Reconstructor::update()
{
#ifdef PARALLEL_PROCESS
#pragma omp parallel for //schedule(static, 1)
#endif
	for (size_t v = 0; v < _voxels_amount; ++v)
		_voxels[v]->visible_on = 0;
	_visible_voxels.clear();

	// update the voxel list
#ifdef PARALLEL_PROCESS
#pragma omp parallel for //schedule(static, 1)
#endif
	for (size_t c = 0; c < _cameras.size(); ++c)
	{
		Camera* camera = _cameras[c];
		LookupTable* lut = _luts[c];

		Mat foreground = camera->getForegroundImage();
		if (!foreground.empty())
		{
			assert(foreground.type() == CV_8U);

			// Iterate over every pixel in the foreground image for camera 'c'
			for (MatConstIterator it = foreground.begin<uchar>(); it < foreground.end<uchar>(); ++it)
			{
				// If the foreground pixel is 'on' (=white)
				if (**it == 255)
				{
					// Find the corresponding voxels at the current pixel
					std::map<cv::Point, std::vector<int>, PointComparator>::iterator voxel_it = lut->voxel_space.find(it.pos());

					if (voxel_it != lut->voxel_space.end() && !voxel_it->second.empty())
					{
						for (size_t v_i = 0; v_i < voxel_it->second.size(); ++v_i)
						{
							// Get the current voxel's index
							const int p = voxel_it->second[v_i];
							++_voxels[p]->visible_on;

							// If the voxel is visible on all views, add it to _visible_voxels vector
							if (_voxels[p]->visible_on == (int) _cameras.size())
							{
#ifdef PARALLEL_PROCESS
#pragma omp critical //push_back is critical
#endif
								_visible_voxels.push_back(_voxels[p]);
							}
						}
					}
				}
			}
		}
	}
}

} /* namespace nl_uu_science_gmt */
