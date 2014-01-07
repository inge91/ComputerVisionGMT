#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include <iostream> 
#include "Reconstructor.h"
using namespace nl_uu_science_gmt;
// THis is the class that creates color histograms
class Histogram
{
	// The histogram uses the Hue channel with a range of 10 for each bin
	double histogram[72];
	Reconstructor::Voxel centroid;


	public:

	std::vector<Reconstructor::Voxel*> voxel_list;
	// The input consists of the voxels belonging to the cluster which we are creating a histogram for.
	Histogram(std::vector<Reconstructor::Voxel*> cluster_members, std::vector<Camera*> c,int k);
	
	static double get_colour(Reconstructor::Voxel *v, std::vector<Camera*> c, std::vector<cv::Mat> hsv);
	void add_voxel(Reconstructor::Voxel* v);
	double calculate_centroid_distance(Reconstructor::Voxel* v);
	void calculate_centroid();
	double calculate_distance(cv::Vec3b f);
	void normalize();
	void remove_voxels();
	double* get_histogram();
	void print_histogram();
};



#endif