/*
 * Reconstructor.h
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#ifndef RECONSTRUCTOR_H_
#define RECONSTRUCTOR_H_

#include <vector>
#include <iostream> 
#include "opencv2/opencv.hpp"
#include "Camera.h"
#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#ifdef __linux__
#include <GL/glut.h>
#include <GL/glu.h>
#endif


namespace nl_uu_science_gmt
{

class Reconstructor
{
public:
	struct Voxel
	{
		int x, y, z;
		cv::Scalar color;
		std::vector<cv::Point> camera_projection;
		std::vector<int> valid_camera_projection;
	};

private:
	const std::vector<Camera*> &_cameras;

	int _step;
	int _size;

	std::vector<cv::Point3f*> _corners;

	size_t _voxels_amount;
	cv::Size _plane_size;

	std::vector<Voxel*> _voxels;
	std::vector<Voxel*> _visible_voxels;
	//The centroid vector
	std::vector<Voxel*> _centroids;
	std::vector<Voxel*> _clusters[4];
	std::vector<std::vector<std::vector<Voxel*>>> _closest_voxel; 

	std::vector<Voxel> track_centroid1;
	std::vector<Voxel> track_centroid2;
	std::vector<Voxel> track_centroid3;
	std::vector<Voxel> track_centroid4;

	void initialize();

public:
	bool different_centroids(std::vector<Voxel> prev_centroids);
	static void draw_voxels();
	Reconstructor(const std::vector<Camera*> &);
	virtual ~Reconstructor();

	void update(int frame_no);

	const std::vector<Voxel*>& getVisibleVoxels() const
	{
		return _visible_voxels;
	}
	// Return centroids and cluster vectors and array
	const std::vector<Voxel*>& getCentroids() const
	{
		return _centroids;
	}

	const std::vector<Voxel*>* getClusters() const
	{
		return _clusters;
	}

	const std::vector<Voxel*>& getVoxels() const
	{
		return _voxels;
	}
	

	void setVisibleVoxels(const std::vector<Voxel*>& visibleVoxels)
	{
		_visible_voxels = visibleVoxels;
	}

	void setVoxels(const std::vector<Voxel*>& voxels)
	{
		_voxels = voxels;
	}

	const std::vector<cv::Point3f*>& getCorners() const
	{
		return _corners;
	}

	int getSize() const
	{
		return _size;
	}

	const cv::Size& getPlaneSize() const
	{
		return _plane_size;
	}
};


} /* namespace nl_uu_science_gmt */

#endif /* RECONSTRUCTOR_H_ */
