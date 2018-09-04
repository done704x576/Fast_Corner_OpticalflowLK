// Fast_Corner_OpticalflowLK.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include "opticalflow.h"

using namespace cv;
using namespace std;

//决定哪些跟踪点被接受
bool acceptTrackedPoint(Point pt0, Point pt1)
{
	return ((abs(pt0.x - pt1.x) + abs(pt0.y - pt1.y)) > 2);
}

int _tmain(int argc, _TCHAR* argv[])
{
	//capture first frame for image subtraction
	Mat im;
	Mat gray;

	string video_name = "bike.avi";
	//string video_name = "E:\\BaiduNetdiskDownload\\Caltech Pedestrian Videos\\set00\\V005.avi";
	VideoCapture cap(video_name);

	if (!cap.isOpened())
	{
		cout << "Failed to open cam" << endl;
		exit(EXIT_FAILURE);
	}

	cap >> im;
	if (im.empty())
	{
		return -1;
	}

	cvtColor(im, gray, CV_BGR2GRAY);

	//set detection region
	const int IMW = gray.cols;
	const int IMH = gray.rows;
	const int ROIW = gray.cols >> 1;
	const int ROIH = gray.rows >> 1;
	//xyrect roi = { { gray.cols >> 2, gray.rows >> 2 }, { roi.tl.x + ROIW, roi.tl.y + ROIH } };
	xyrect roi = { { 20, 20 }, { gray.cols - 20, gray.rows - 20 } };
	const int ROISTART_OFFSET = IMW * roi.tl.y + roi.tl.x;

	//initialize variable
	init_opticalflow_estimation(gray.data, IMW, IMH, roi);

	while (true)
	{
		// Start a timer
		double duration;
		duration = static_cast<double>(cv::getTickCount());

		cap >> im;
		if (im.empty())
		{
			break;
		}
		cvtColor(im, gray, CV_BGR2GRAY);
		int numcorners = 0;
		int i = 0;
		const int CORNER_THRESHOLD = 15;
		//xy* fast_corners_start = fast12_detect_nonmax(gray.data + ROISTART_OFFSET, ROIW, ROIH, IMW, CORNER_THRESHOLD, &numcorners);
		xy* fast_corners_start = fast12_detect_nonmax(gray.data + ROISTART_OFFSET, roi.br.x - roi.tl.x, roi.br.y - roi.tl.y, IMW, CORNER_THRESHOLD, &numcorners);

		//to global coordinates
		for (i = 0; i < numcorners; ++i)
		{
			fast_corners_start[i].x += roi.tl.x;
			fast_corners_start[i].y += roi.tl.y;
		}

		xy* fast_corners_end = (xy*)malloc(numcorners * sizeof(xy));
		memcpy(fast_corners_end, fast_corners_start, numcorners * sizeof(xy));
		opticalflow_estimation(gray.data, fast_corners_start, numcorners, fast_corners_end);

		//draw result
		rectangle(im, Rect(Point(roi.tl.x, roi.tl.y), Point(roi.br.x, roi.br.y)), Scalar(125, 125, 125), 2, 8);
		for (i = 0; i < numcorners; ++i)
		{
			if (acceptTrackedPoint(Point(fast_corners_start[i].x, fast_corners_start[i].y),Point(fast_corners_end[i].x, fast_corners_end[i].y)))
			{
				circle(im, Point(fast_corners_start[i].x, fast_corners_start[i].y), 3, Scalar(255, 0, 0), -1);
				line(im, Point(fast_corners_start[i].x, fast_corners_start[i].y),
					Point(fast_corners_end[i].x, fast_corners_end[i].y), Scalar(0, 0, 255), 1);
			}
		}

		imshow("im", im);
		waitKey(1);
	
		free(fast_corners_start);
		free(fast_corners_end);

		// Calculate the time cost and print
		duration = static_cast<double>(cv::getTickCount()) - duration;
		duration /= cv::getTickFrequency();
		std::cout << duration * 1000 << " ms" << std::endl;
	}

	//free variable
	free_opticalflow_estimation();
	return 0;
}



