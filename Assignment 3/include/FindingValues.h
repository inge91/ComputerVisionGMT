#ifndef FINDINGVALUES_H_
#define FINDINGVALUES_H_
#include "opencv2/opencv.hpp"
#include "Reconstructor.h"
using namespace std; 
using namespace cv;
using namespace nl_uu_science_gmt;
// Some nice protoypes for your enjoyment 
void calculateHSV(int hsv[]);
float get_difference(Mat m1, Mat m2);
float calculate_score();
Mat process( int h, int s, int v, int erode_nr, int dilate_nr);
void calculateImprovement(int h, int s, int v, int improvement[]);
void kMeans(std::vector<Reconstructor::Voxel*> v, int nr_centroids, vector<Reconstructor::Voxel*>&centroid, vector<Reconstructor::Voxel*> f[]);
bool check_centroids(vector<Reconstructor::Voxel*> centroid);

#endif