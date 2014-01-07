#include "Histogram.h"

Histogram::Histogram(std::vector<Reconstructor::Voxel*> cluster_members, std::vector<Camera*> c, int k)
{
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;

	cv::Mat frame_rgb0 = c[0]->getVideoFrame(k);
	cv::Mat frame_rgb1 = c[1]->getVideoFrame(k);
	cv::Mat frame_rgb2 = c[2]->getVideoFrame(k);
	cv::Mat frame_rgb3 = c[3]->getVideoFrame(k);
	cv::Mat frame_hsv0;
	cv::Mat frame_hsv1;
	cv::Mat frame_hsv2;
	cv::Mat frame_hsv3;
	cv::cvtColor(frame_rgb0,frame_hsv0,CV_RGB2HSV);
	cv::cvtColor(frame_rgb1,frame_hsv1,CV_RGB2HSV);
	cv::cvtColor(frame_rgb2,frame_hsv2,CV_RGB2HSV);
	cv::cvtColor(frame_rgb3,frame_hsv3,CV_RGB2HSV);
	std::vector<cv::Mat> hsv;
	hsv.push_back(frame_hsv0);
	hsv.push_back(frame_hsv1);
	hsv.push_back(frame_hsv2);
	hsv.push_back(frame_hsv3);

	// make array empty
	for(int i = 0; i < 72; i++)
	{
		histogram[i] = 0;
	}
	for(int i = 0; i < cluster_members.size(); i++)
	{

		int h = get_colour(cluster_members[i], c, hsv);
		histogram[h/5] ++;
	}
	// normalize the histogram 
	normalize();

}

// normalizes the histogram
void Histogram::normalize()
{
	int total = 0;
	for(int i = 0; i < 72; i++)
	{
		total += histogram[i];
		//std::cout<< total<<std::endl;
	}

		//std::cout<<"Total"<<std::endl;
		//std::cout<< total<<std::endl;

	for(int i = 0; i < 72; i++)
	{
		//std::cout<<histogram[i]/total<<std::endl;
		histogram[i] = histogram[i] / total;
	}
}

// How to deal with occlusion: 
// loop through pixels of each camera view. Look for each voxel belonging to 
// pixel combination and only use voxel closest to camera


// returns the color from a single voxel
double Histogram::get_colour(Reconstructor::Voxel* v, std::vector<Camera*> c, std::vector<cv::Mat> hsv)
{
	// Get all camera projections
	std::vector<cv::Point> cameras = v->camera_projection;
	// Get the validity of all camera projections
	std::vector<int> validity = v->valid_camera_projection;

	// Loop through all camera projections and only use the valid ones
	// to find out mean color
	double h = 0;
	int cameras_used = 0;
	for(int j = 0; j < 4; j ++)//cameras.size(); j++)
	{

		// Still need to fix occlusion (validity is not it)
		if(validity[j] == 1)
		{
			cv::Point pos = cameras[j];
			// Convert frame to hsv
			h += hsv[j].at<cv::Vec3b>(pos)[0];
			cameras_used += 1;
		}
	}
	// Final hue value
	h /= cameras_used;
	return h;
}

// Calculate the distance between a voxel color and histogram
double Histogram::calculate_distance(cv::Vec3b f)
{
	double total = 0;
	double hue = f[0];
	int bin = hue/5;
	for(int i = 0; i < 72; i++)
	{
		total += histogram[i] / (i - bin + 1);
	}
	//Increae value a lot so it wont turn out zero
	return total;
}

double* Histogram::get_histogram()
{
	return histogram;
}

void Histogram::print_histogram()
{
	for(int i = 0; i < 25; i++)
	{
		std::cout<< histogram[i] << " ";
	}

		std::cout<<std::endl;
}


// Add voxel to the vector of voxels
// labeld as belonging to this histogram
void Histogram::add_voxel(Reconstructor::Voxel* v)
{
	voxel_list.push_back(v);
}

// Set centroid
void Histogram::calculate_centroid()
{
	double total_x = 0;
	double total_z = 0;
	
	//std::cout<<"voxel list size " << voxel_list.size()<<std::endl;
	for(int i = 0; i < voxel_list.size(); i++)
	{
		
		//std::cout<< " x y "<< voxel_list[i]->x << " "<< voxel_list[i]->y<<std::endl;
		total_x += voxel_list[i]->x;
		total_z += voxel_list[i]->z;
		
	}

	centroid.x = total_x / voxel_list.size();
	centroid.y = 0;
	centroid.z = total_z / voxel_list.size();
		
//	std::cout<<"centroid x y z: "<< centroid.x << " "<< centroid.y<< " "<<centroid.z <<std::endl;
}

double Histogram::calculate_centroid_distance(Reconstructor::Voxel* v)
{
	double dx = v->x - centroid.x;
	double dz = v->z - centroid.z;
	return 	sqrt(dx * dx + dz * dz);
}


// Remove the voxels beloning to the histogram
void Histogram::remove_voxels()
{
	voxel_list.clear();
}


