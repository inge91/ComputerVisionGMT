#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include <iostream> 
#include "Reconstructor.h"
using namespace nl_uu_science_gmt;
// THis is the class that creates color histograms
class Histogram
{
	// The histogram uses the Hue channel with a range of 10 for each bin
	int histogram[36];


	public:
	// The input consists of the voxels belonging to the cluster which we are creating a histogram for.
	Histogram(std::vector<Reconstructor::Voxel*> cluster_members, std::vector<Camera*> c,int k);
	
	int* get_histogram();
	void print_histogram();
};



#endif