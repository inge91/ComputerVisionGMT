#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include <iostream> 
#include "Reconstructor.h"
using namespace nl_uu_science_gmt;
using namespace std;
// THis is the class that creates color histograms
class Histogram
{
	// The histogram uses the Hue channel with a range of 10 for each bin
	double histogram[72];
	Reconstructor::Voxel centroid;


	public:

	vector<Reconstructor::Voxel*> voxel_list;
	// The input consists of the voxels belonging to the cluster which we are creating a histogram for.
	Histogram(vector<Reconstructor::Voxel*> cluster_members, vector<Camera*> c, int k, vector<vector<vector<Reconstructor::Voxel*>>> &camera);
	
	static double get_colour(Reconstructor::Voxel *v, vector<Camera*> c, vector<cv::Mat> hsv, vector<vector<vector<Reconstructor::Voxel*>>> &camer);
	static void find_closest_voxels(vector<vector<vector<Reconstructor::Voxel*>>> &camera_1, vector<Camera*> c, vector<Reconstructor::Voxel*> voxels);
	static bool is_valid(Reconstructor::Voxel* v1, Reconstructor::Voxel* v2);

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