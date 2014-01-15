#include "Histogram.h"

Histogram::Histogram(vector<Reconstructor::Voxel*> cluster_members, vector<Camera*> c, int k, vector<vector<vector<Reconstructor::Voxel*>>> &camera)
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
	vector<cv::Mat> hsv;
	hsv.push_back(frame_hsv0);
	hsv.push_back(frame_hsv1);
	hsv.push_back(frame_hsv2);
	hsv.push_back(frame_hsv3);

	// make array empty
	for(int i = 0; i < HISTOGRAM_SIZE; i++)
	{
		histogram[i] = 0;
	}
	for(int i = 0; i < cluster_members.size(); i++)
	{
		if(cluster_members[i]->z < 800)
		{
			continue;
		}
		int h = get_colour(cluster_members[i], c, hsv, camera);
		if(h >= 0)
		{
			histogram[h/(360/HISTOGRAM_SIZE)] ++;
		}
	}

	// normalize the histogram 
	normalize();

}

// normalizes the histogram
void Histogram::normalize()
{
	int total = 0;
	for(int i = 0; i < HISTOGRAM_SIZE; i++)
	{
		total += histogram[i];
	}


	for(int i = 0; i < HISTOGRAM_SIZE; i++)
	{
		//cout<<histogram[i]/total<<endl;
		histogram[i] = histogram[i] / total;
	}
}

// How to deal with occlusion: 
// loop through pixels of each camera view. Look for each voxel belonging to 
// pixel combination and only use voxel closest to camera


// Check if the voxels are the same
bool Histogram::is_valid(Reconstructor::Voxel* v1, Reconstructor::Voxel* v2)
{
	if( v1->x == v2->x && v1->y == v2->y && v1->z == v2->z)
	{
		return true;
	}
	return false;
}

// returns the color from a single voxel
double Histogram::get_colour(Reconstructor::Voxel* v, vector<Camera*> c, vector<cv::Mat> hsv,  vector<vector<vector<Reconstructor::Voxel*>>> &camera)
{
	// Get all camera projections
	vector<cv::Point> cameras = v->camera_projection;
	// Get the validity of all camera projections
	vector<int> validity = v->valid_camera_projection;

	// Loop through all camera projections and only use the valid ones
	// to find out mean color
	double h = 0;
	double s = 0;
	double value = 0;
	int cameras_used = 0;
	for(int j = 0; j < camera.size(); j ++)
	{
		cv::Point pos = cameras[j];
		if(Histogram::is_valid(v, camera[j][pos.x][pos.y]))
		{
			// Convert frame to hsv
			h += hsv[j].at<cv::Vec3b>(pos)[0];
			s += hsv[j].at<cv::Vec3b>(pos)[1];
			value += hsv[j].at<cv::Vec3b>(pos)[2];
			cameras_used += 1;
		}
	}
	s /= cameras_used;
	value /= cameras_used;

	if (cameras_used == 0 || (s < 25 || value < 25))
	{
		return -1;
	}
	else return h /= cameras_used;
}

// Calculate the distance between a voxel color and histogram
double Histogram::calculate_distance(cv::Vec3b f)
{
	double total = 0;
	double hue = f[0];
	int bin = hue/(360/HISTOGRAM_SIZE);
	for(int i = 0; i < HISTOGRAM_SIZE; i++)
	{
		total += histogram[i] / (i - bin + 1);
	}
	//Increae value a lot so it wont turn out zero
	//return total;
	return histogram[bin];
}

double* Histogram::get_histogram()
{
	return histogram;
}

void Histogram::print_histogram()
{
	for(int i = 0; i < HISTOGRAM_SIZE; i++)
	{
		cout<< histogram[i] << " ";
	}

		cout<<endl;
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
	double total_y = 0;
	double total_z = 0;
	
	for(int i = 0; i < voxel_list.size(); i++)
	{
		
		total_x += voxel_list[i]->x;
		total_y += voxel_list[i]->y;
		total_z += voxel_list[i]->z;
		
	}

	centroid.x = total_x / voxel_list.size();
	centroid.y = total_y / voxel_list.size();
	//centroid.z = total_z / voxel_list.size();
	centroid.z = 0;
}

double Histogram::calculate_centroid_distance(Reconstructor::Voxel* v)
{
	double dx = v->x - centroid.x;
	double dy = v->y - centroid.y;
	double dz = v->z - centroid.z;
	return 	sqrt(dx * dx + dy * dy);
}


// Remove the voxels beloning to the histogram
void Histogram::remove_voxels()
{
	voxel_list.clear();
}


// Put only the closes voxels at eat projeciton point in camera view array 
void Histogram::find_closest_voxels(vector<vector<vector<Reconstructor::Voxel*>>> &camera, 
									vector<Camera*> c, vector<Reconstructor::Voxel*> voxels)

{
	// Loop through all voxels and update positions in camera in case this new position is closer
	for (Reconstructor::Voxel *v : voxels)
	{
		for(int i = 0 ; i < c.size(); i ++)
		{
			// Get the pixel the voxel is projected to for this camera view
			cv::Point p = v->camera_projection[i];
			// In case the current pointer is a null pointer replace it by pointing to this voxel
			if(camera[i][p.x][p.y] == NULL)
			{
				camera[i][p.x][p.y] = v;
			}
			else 
			{
				
				cv::Point3f f = c[i]->getCameraLocation();
				double dx = f.x - v->x;
				double dy = f.y - v->y;
				double dz = f.z - v->z;
				double newlength = sqrt(dx * dx + dy * dy + dz * dz);
				dx = f.x - camera[i][p.x][p.y]->x;
				dy = f.y - camera[i][p.x][p.y]->y;
				dz = f.z - camera[i][p.x][p.y]->z;
				double oldlength = sqrt(dx * dx + dy * dy + dz * dz);

				// If the new voxel is closer than the voxel before, change it to the new one
				if(newlength < oldlength)
				{
					camera[i][p.x][p.y] = v;
				}
			}
		}
	}

}