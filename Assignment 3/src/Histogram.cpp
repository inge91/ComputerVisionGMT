#include "Histogram.h"

Histogram::Histogram(std::vector<Reconstructor::Voxel*> cluster_members, std::vector<Camera*> c, int k)
{
	// make array empty
	for(int i = 0; i < 25; i++)
	{
		histogram[i] = 0;
	}
	for(int i = 0; i < cluster_members.size(); i++)
	{
		// Get all camera projections
		std::vector<cv::Point> cameras = cluster_members[i]->camera_projection;
		// Get the validity of all camera projections
		std::vector<int> validity = cluster_members[i]->valid_camera_projection;

		// Loop through all camera projections and only use the valid ones
		// to find out mean color
		int h = 0;
		int cameras_used = 0;
		for(int j = 0; j < 1; j ++)//cameras.size(); j++)
		{
			if(validity[j] == 1)
			{
				cv::Point pos = cameras[j];
				cv::Mat frame_rgb = c[j]->getVideoFrame(k);
				cv::Mat frame_hsv;
				// Convert frame to hsv
				cv::cvtColor(frame_rgb,frame_hsv,CV_RGB2HSV);
				h += frame_hsv.at<cv::Vec3b>(pos)[0];
				cameras_used += 1;
			}
		}
		// Final hue value
		h /= cameras_used;
		histogram[h/10] ++;
	}

}


int* Histogram::get_histogram()
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