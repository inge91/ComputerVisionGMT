/*
 * Reconstructor.h
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#ifndef RECONSTRUCTOR_H_
#define RECONSTRUCTOR_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "Camera.h"

namespace nl_uu_science_gmt
{

class Reconstructor
{
public:
	struct PointComparator
	{
		bool operator()(const cv::Point& lhs, const cv::Point& rhs) const
		{
			if (lhs.y == rhs.y) return lhs.x < rhs.x;
			return lhs.y < rhs.y;
		}
	};

	struct LookupTable
	{
		size_t camera_id;
		std::map<cv::Point, std::vector<int>, PointComparator> voxel_space;
	};

	struct Voxel
	{
		int x, y, z;
		cv::Scalar color;
		int visible_on;
		std::map<int, cv::Point> pixel_on_views;
	};

private:
	const std::vector<Camera*> &_cameras;

	int _step;
	int _size;

	std::vector<cv::Point3f*> _corners;

	size_t _voxels_amount;
	cv::Size _plane_size;

	std::vector<LookupTable*> _luts;
	std::vector<Voxel*> _voxels;
	std::vector<Voxel*> _visible_voxels;

	void initialize();

public:
	Reconstructor(const std::vector<Camera*> &);
	virtual ~Reconstructor();

	void update();

	const std::vector<Voxel*>& getVisibleVoxels() const
	{
		return _visible_voxels;
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
};

} /* namespace nl_uu_science_gmt */

#endif /* RECONSTRUCTOR_H_ */
