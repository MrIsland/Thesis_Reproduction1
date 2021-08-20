#include "opencv1.h"
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

void func1_opencv1(string path[], Size board_size, int ln) {
	opencv1 res;
	res.set_mat(path, board_size, ln);
	res._FindCorner();
	res.calibration();
}

void func2_opencv_Iterative(string path[], Size board_size, int ln) {
	opencv_Iterative res;
}

int main() {
	ifstream fin("F:/MrIsland/Thesis_Reproduction1/Demo/calibdata.txt");
	//ofstream fout("calibration_result.txt");

	Size image_size;
	Size board_size = Size(7, 5);    /* 标定板上每行、列的角点数 */
	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */

	string filename;
	while (getline(fin, filename)) {
		str_path[++image_count] = filename;
		//cout << "filename: " << filename << endl;
		Mat imageinput = imread(filename);
		if (image_count == 1) {
			image_size.width = imageinput.cols;
			image_size.height = imageinput.rows;
		}
	}
	cout << image_size.width << " " << image_size.height << endl;
	func1_opencv1(str_path, board_size, image_count);
	return 0;
}
