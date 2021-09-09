#include "opencv1.h"


void opencv1::set_mat(string str[], Size bs, int ln) {
	this->src_count = ln; 
	cout << "ln: " << ln << " " << src_count << endl;
	for (int i = 1; i <= ln; i++) {
		this->src[i] = imread(str[i]);
		this->src[i].copyTo(this->gray_src[i]);
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
		if (i == 1) {
			this->image_size.width = this->src[i].cols;
			this->image_size.height = this->src[i].rows;
		}
	}
	this->board_size = bs;
	this->cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	this->distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));

	//cout << "str: " << str << endl;
	//waitKey();
	//imshow("setmat_gray", this->gray_src);
	//waitKey(0);
}

void opencv1::_FindCorner() {
	int ln = this->src_count;
	cout << "ln: " << ln << endl;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << endl;
		vector <Point2f> image_points_buf;
		if (findChessboardCorners(this->src[i], this->board_size, image_points_buf) == 0) {
			cout << "Can't find chessboard corners!\n";
		}
		else {
			Mat view_gray = this->gray_src[i];
			find4QuadCornerSubpix(view_gray, image_points_buf, this->board_size);
			this->image_points_seq.push_back(image_points_buf);

			drawChessboardCorners(view_gray, this->board_size, image_points_buf, true);
			//imshow("Camera Calibration", view_gray);
			//waitKey(0);
		}
	}
}

void opencv1::calibration() {
	//cout << "?????" << endl;
 	Size square_size = Size(28, 28);
	vector <vector <Point3f> >object_points;
	vector <int> point_counts;
	cout << board_size.height << " " << board_size.width << endl;
	for (int t = 1; t <= this->src_count; t++) {
		vector <Point3f> tmpPointSet;
		for (int i = 0; i < this->board_size.height; i++) {
			for (int j = 0; j < this->board_size.width; j++) {
				Point3f realpoint;
				realpoint.x = i * square_size.width;
				realpoint.y = j * square_size.height;
				realpoint.z = 0;
				tmpPointSet.push_back(realpoint);
			}
		}
		object_points.push_back(tmpPointSet);
	}
	for (int t = 1; t <= src_count; t++) {
		int width = board_size.width, height = board_size.height;
		point_counts.push_back(width * height);
	}
	calibrateCamera(object_points, this->image_points_seq, this->image_size, this->cameraMatrix, this->distCoeffs, this->rvecsMat, this->tvecsMat, 0);
	cout << "opencv1 finished !" << endl;

	double total_res = 0, res = 0;
	vector <Point2f> image_points2; // 保存重新计算得到的投影点
	for (int i = 1; i <= this->src_count; i++) {
		vector<Point3f> tmpPointSet = object_points[i-1];

		projectPoints(tmpPointSet, this->rvecsMat[i-1], this->tvecsMat[i-1], this->cameraMatrix, this->distCoeffs, image_points2);

		vector <Point2f> tmpImagePoint = image_points_seq[i-1];
		Mat tmpImagePointMat = Mat(1, tmpImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		int ln = (int)tmpImagePoint.size();
		for (int j = 0; j < ln; j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tmpImagePointMat.at<Vec2f>(0, j) = Vec2f(tmpImagePoint[j].x, tmpImagePoint[j].y);
		}
		err = norm(image_points2Mat, tmpImagePointMat, NORM_L2);
		err /= point_counts[i-1];
		total_err += err;
		cout << "No." << i << " picture's average error: " << err << " pixel!" << endl;
		//fout << "No." << i << " picture's average error: " << err << " pixel!";
	}
	cout << "Total average error: " << total_err / this->src_count << endl;
}