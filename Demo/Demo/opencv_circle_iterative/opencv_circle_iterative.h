#pragma once
#ifndef OPENCV_CIRCLE_ITERATIVE_INCLUDED
#define OPENCV_CIRCLE_ITERATIVE_INCLUDED

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;
class opencv_circle_iterative {
private:
	Size image_size, board_size;
	Mat src[30], gray_src[30];
	vector<vector <Point2f> > image_points_seq;
	int src_count = 0;
	vector <Mat> tvecsMat, rvecsMat;
	Mat cameraMatrix, distCoeffs;
	Mat unproject_src[30];
	Mat H[30];
	double total_err = 0, err = 0;
	vector<Point2f> srcTri[30], disTri[30];
public:
	void set_mat(Size bs);
	void _FindCorner(bool flag);
	void calibration();
	void init();
	void iterative(int num);
	void unproject();
	void reproject();
	vector<Point2f> cornersort(vector<Point2f> tmp, vector<Point2f> res);
};

#endif // !OPENCV_ITERATIVE_INCLUDED
