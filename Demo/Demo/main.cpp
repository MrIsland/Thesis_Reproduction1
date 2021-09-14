#include "opencv1/opencv1.h"
#include "opencv_iterative/opencv_iterative.h"
#include "opencv_circle/opencv_circle.h"
#include "opencv_circle_iterative/opencv_circle_iterative.h"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cv;

const int maxn = 20;
int image_count = 0;
string str_path[maxn];

void func1_opencv1(Size board_size) {
	opencv1 res;
	res.set_mat(board_size);
	res._FindCorner();
	res.calibration();
}

void func2_opencv_Iterative(Size board_size) {
	opencv_iterative res;
	res.set_mat(board_size);
	res.iterative(5);
}

void func3_opencv_circle(Size board_size) {
	opencv_circle res;
	res.set_mat(board_size);
	res._FindCorner();
	res.calibration();
}

void func4_opencv_circle_iterative(Size board_size) {
	opencv_circle_iterative res;
	res.set_mat(board_size);
	res.iterative(5);
}

int main() {
	//ifstream fin("F:/MrIsland/Thesis_Reproduction1/Demo/calibdata.txt");
	//ofstream fout("calibration_result.txt");

	Size image_size;
	Size board_size = Size(9, 6);    /* �궨����ÿ�С��еĽǵ��� */
	//vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	//vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */

	//string filename;
	//while (getline(fin, filename)) {
	//	str_path[++image_count] = filename;
	//	//cout << "filename: " << filename << endl;
	//	Mat imageinput = imread(filename);
	//	if (image_count == 1) {
	//		image_size.width = imageinput.cols;
	//		image_size.height = imageinput.rows;
	//	}
	//}
	//cout << image_size.width << " " << image_size.height << endl;
	// 
	//func1_opencv1(board_size);
	//func2_opencv_Iterative(board_size);
	//func3_opencv_circle(board_size);
	func4_opencv_circle_iterative(board_size);

	return 0;
}
