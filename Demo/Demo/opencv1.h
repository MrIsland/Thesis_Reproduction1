#pragma once
#ifndef OPENCV_1_INCLUDED
#define OPENCV_1_INCLUDED

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
class opencv1 {
private:
	Size image_size, board_size;
	Mat src[20], gray_src[20];
	vector<vector <Point2f> > image_points_seq;
	int src_count = 0;
	vector <Mat> tvecsMat, rvecsMat;
	Mat cameraMatrix, distCoeffs;
	double total_err = 0, err = 0;
public:
	void set_mat(string str[], Size bs, int ln);
	void _FindCorner();
	void calibration();
};

#endif // !OPENCV_1_INCLUDED

