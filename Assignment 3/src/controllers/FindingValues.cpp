// Written by Inge Becht 4157281
//			  Merijn van Tooren 3689557

#include "FindingValues.h";

	// sums the difference between two images
	// Every pixel that is different adds a value of
	// 1 to the total sum. The final value returned
	// is the percentage that was similar between
	// the two images.
	float get_difference(Mat m1, Mat m2)
	{
		float sum = 0;
		for(int i=0; i<m1.rows; i++)
		{
			for(int j=0; j<m1.cols; j++)
			{
				int first =m1.at<cv::Vec3b>(i,j)[0] > 0? 1:0;
				int second = m2.at<cv::Vec3b>(i,j)[0] > 0? 1:0;
				int diff = abs(first-second);
				sum += diff;
			}
		}
		sum = 100 - (((float)sum  / (m1.rows * m2.cols)) * (float)100);
		return sum;
	}

	// The score is calculate dbetween a ground truth with dilation 6 and 
	// the very first frame of cam 1 contoured.
	float calculate_score()
	{
		const Mat m = imread("data\\cam1\\ground_truth.png");
		const Mat img = imread("data\\cam1\\first_frame_threshed.png");
		float score = get_difference(img, m);
		return score;
	}

	
	// Calculates Hue, Saturation or Value depending on ground_truth
	// To no have to calculate a triple for loop we:
	// -First H is fixed with S and V set to default
	// -Then S with the optimal H and default V
	// -Last V with optimal H and optimal V
	void calculateHSV(int hsv[])
	{
		// calculate H
		float best_value = 0;
		for(int i = 0; i < 256; i ++)
		{
			Mat img = process(i, 15, 25, 0, 0);
			float temp_value = calculate_score();
			// Determine if we have a better value for H
			if(temp_value > best_value)
			{
				best_value = temp_value;
				hsv[0] = i;
			}
		}

		// calculate S
		best_value = 0;
		for(int i = 0; i < 256; i ++)
			{
				Mat img = process(hsv[0], i, 25, 0, 0);

				float temp_value = calculate_score();
				if(temp_value > best_value)
				{
					best_value = temp_value;
					hsv[1] = i;
				}
			}

		// Calculate v
		best_value = 0;
		for(int i = 0; i < 256; i ++)
			{
				Mat img = process(hsv[0], hsv[1], i, 0, 0);


				float temp_value = calculate_score();
				if(temp_value > best_value)
				{
					best_value = temp_value;
					hsv[2] = i;
				}
			}
	}
	
	// Determine the amount of erosion and dilation steps that is optimal
	void calculateImprovement(int h, int s, int v, int improvement[])
	{
		double best_value= 0;
		for(int i = 0; i < 25; i ++)
		{

			for(int j = 0; j < 25; j ++)
			{
			Mat img = process(h, s, v, i, j);
			float temp_value = calculate_score();
			if(temp_value > best_value)
				{
					best_value = temp_value;
					improvement[0] = i;
					improvement[1] = j;
				}
			}
		}
	}

// Reusing code by Coert, now for the purpose of applying the hsv values 
// We always use the first frame and save this to file.
Mat process( int h, int s, int v, int noise_remove, int silhouette_fill)
{
	Mat hsv_image;

	const Mat m = imread("data\\cam1\\first_frame.png");
	const Mat bg = imread("data\\cam1\\background.png");
	cvtColor(m, hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

	vector<Mat> channels;
	split(hsv_image, channels);  // Split the HSV-channels for further analysis


	vector<Mat> bgchannels;
	cvtColor(bg, hsv_image, CV_BGR2HSV);  // from BGR to HSV color space
	split(hsv_image, bgchannels);  // Split the HSV-channels for further analysis

	// Background subtraction H
	Mat tmp, foreground, background;
	
	absdiff(channels[0], bgchannels[0], tmp);
	threshold(tmp, foreground, h, 255, CV_THRESH_BINARY);

	// Background subtraction S
	absdiff(channels[1], bgchannels[1], tmp);
	threshold(tmp, background, s, 255, CV_THRESH_BINARY);
	bitwise_and(foreground, background, foreground);

	// Background subtraction V
	absdiff(channels[2], bgchannels[2], tmp);
	threshold(tmp, background, v, 255, CV_THRESH_BINARY);
	bitwise_or(foreground, background, foreground);
	// Remove noise
	#ifndef USE_GRAPHCUTS
		
		// erosion and dilation just for small noise removal
		for(int i = 0; i < noise_remove; i++)
		{
			erode(foreground, foreground, Mat());
		}

		for(int i = 0; i < noise_remove; i++)
		{
			dilate(foreground, foreground, Mat());
		}
		// dilation and erosion for filling of the silhouette
		for(int i = 0; i < silhouette_fill; i ++)
		{
			dilate(foreground, foreground, Mat());
		}
		for(int j = 0; j < silhouette_fill; j ++)
		{
			erode(foreground, foreground, Mat());

		}
		// Using Erosion and/or Dilation of the foreground image
	#else
		// Using Graph cuts on the foreground image
	#endif

	imwrite("data\\cam1\\first_frame_threshed.png", foreground);
	return foreground;
}
