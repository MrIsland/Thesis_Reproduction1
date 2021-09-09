#include "opencv_iterative.h"

void opencv_iterative::set_mat(string str[], Size bs, int ln) {
	this->src_count = ln;
	this->board_size = bs;
	this->cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	this->distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	for (int i = 1; i <= ln; i++) {
		this->src[i] = imread(str[i]);
		if (i == 1) {
			this->image_size.width = this->src[i].cols;
			this->image_size.height = this->src[i].rows;
		}
	}
}
 
void opencv_iterative::init() {
	int ln = this->src_count;
	for (int i = 1; i <= ln; i++) {
		this->src[i].copyTo(this->gray_src[i]);
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
	}
}

void opencv_iterative::_FindCorner() {
	this->image_points_seq.clear();
	int ln = this->src_count;
	cout << "ln: " << ln << endl;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << endl;
		cout << this->board_size << endl;
		vector <Point2f> image_points_buf;

		//imshow("Camera Calibration", this->gray_src[i]);
		//waitKey(0);
		
		if (findChessboardCorners(this->gray_src[i], this->board_size, image_points_buf) == 0) {
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

void opencv_iterative::calibration() {
	Size square_size = Size(28, 28);
	vector <vector <Point3f> >object_points;
	vector <int> point_counts;

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

	//计算单应性矩阵
	//for (int i = 1; i <= this->src_count; i++) {
	//	this->H[i] = findHomography(object_points[i-1], this->image_points_seq[i-1]);
	//	cout << format(H[i], Formatter::FMT_NUMPY) << endl;
	//}

	for (int t = 1; t <= src_count; t++) {
		int width = board_size.width, height = board_size.height;
		cout << "width: " << width << "  " << " height: " << height << endl;
		point_counts.push_back(width * height);
	}
	this->rvecsMat.clear(); this->rvecsMat.clear();
	calibrateCamera(object_points, this->image_points_seq, this->image_size, this->cameraMatrix, this->distCoeffs, this->rvecsMat, this->tvecsMat, 0);
	cout << "camera calibration finished !" << endl;

	double total_err = 0, err = 0;
	vector <Point2f> image_points2; // 保存重新计算得到的投影点
	for (int i = 1; i <= this->src_count; i++) {
		vector<Point3f> tmpPointSet = object_points[i - 1];

		projectPoints(tmpPointSet, this->rvecsMat[i - 1], this->tvecsMat[i - 1], this->cameraMatrix, this->distCoeffs, image_points2);

		vector <Point2f> tmpImagePoint = image_points_seq[i - 1];
		Mat tmpImagePointMat = Mat(1, tmpImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		int ln = (int)tmpImagePoint.size();
		for (int j = 0; j < ln; j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tmpImagePointMat.at<Vec2f>(0, j) = Vec2f(tmpImagePoint[j].x, tmpImagePoint[j].y);
		}
		err = norm(image_points2Mat, tmpImagePointMat, NORM_L2);
		err /= point_counts[i - 1];
		total_err += err;
		cout << "No." << i << " picture's average error: " << err << " pixel!" << endl;
		//fout << "No." << i << " picture's average error: " << err << " pixel!";
	}
	cout << "Total average error: " << total_err / this->src_count << endl;
}

void opencv_iterative::iterative(int num) {
	int tmp_num = num;
	init();
	_FindCorner();
	calibration();
	for (int t=1; t<=num; t++) {
		init();
		cout << "This is the " << t <<" time(s) !" << endl;
		Mat mapx = Mat(this->image_size, CV_32FC1);
		Mat mapy = Mat(this->image_size, CV_32FC1);
		Mat R = Mat::eye(3, 3, CV_32F);
		for (int i = 1; i <= this->src_count; i++) {
			initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, R, this->cameraMatrix, this->image_size, CV_32FC1, mapx, mapy);
			Mat imageSrc = this->src[i];

			//if (i == 1) {
			//	imshow("newsrc", this->src[i]);
			//	imshow("gray", this->gray_src[i]);
			//	waitKey(0);
			//}

			Mat newSrc = imageSrc.clone();
			remap(imageSrc, newSrc, mapx, mapy, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
			this->src[i] = newSrc;
			cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);

			//if (i == 1) {
			//	imshow("newsrc", this->src[i]);
			//	imshow("gray", this->gray_src[i]);
			//	waitKey(0);
			//}

			//imshow("imagestc", imageSrc);
		}
		unproject();
		_FindCorner();
		reproject();
		calibration();
	}
}

void opencv_iterative::unproject() {
	int ln = (int)this->src_count;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << "   ";

		srcTri[i - 1].clear();
		disTri[i - 1].clear();

		// 后续换底板时需要修改！！！！
		// 此处应该将srcTri四个点换成背景板中白色的四个角点
		Mat graypic, binpic, cannypic;
		this->gray_src[i].copyTo(graypic);
		this->gray_src[i].copyTo(binpic);
		this->gray_src[i].copyTo(cannypic);
		medianBlur(graypic, graypic, 7);
		threshold(graypic, binpic, 80, 255, THRESH_BINARY);
		Canny(binpic, cannypic, 200, 2.5);

		//imshow("unproject", binpic);
		//waitKey(0);

		vector <vector<Point> > contours; // 用于存储轮廓
		vector <Vec4i> hierarchy;

		findContours(cannypic, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

		Mat linepic = Mat::zeros(cannypic.rows, cannypic.cols, CV_8UC3);
		for (int j = 0; j < contours.size(); j++) 
			drawContours(linepic, contours, j, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8);
		
		//imshow("unproject", linepic);
		//waitKey(0);

		vector <vector<Point> > polyContours(contours.size());
		int pmax = 0;
		for (int j = 0; j < contours.size(); j++) {
			if (contourArea(contours[j]) > contourArea(contours[pmax])) pmax = j;
			approxPolyDP(contours[j], polyContours[j], 10, true);
		}
		Mat polypic = Mat::zeros(this->gray_src[i].size(), CV_8UC3);
		drawContours(polypic, polyContours, pmax, Scalar(0, 0, 255), 2);


		//imshow("unproject", polypic);
		//waitKey(0);

		vector <int> hull;
		convexHull(polyContours[pmax], hull, false);

		vector <Point2f> quadangle, tmp_quadangle;

		for (int j = 0; j < hull.size(); ++j) {
			circle(polypic, polyContours[pmax][j], 10, Scalar(rand() & 255, rand() & 255, rand() & 255), 3);
			//cout << polyContours[pmax][i].x << " " << polyContours[pmax][i].y << endl;
			tmp_quadangle.push_back(polyContours[pmax][j]);
		}
		Mat showpic;
		this->src[i].copyTo(showpic);
		addWeighted(polypic, 0.5, showpic, 0.5, 0, showpic);

		


		cout << "11111" << endl;
		cout << "ln: " << image_points_seq.size() << endl;
		cout << this->board_size.width << " " << this->board_size.height << endl;
		cout << "polyC ln: " << hull.size() << endl;
		quadangle.push_back(image_points_seq[i-1][0]);
		quadangle.push_back(image_points_seq[i-1][this->board_size.width-1]);
		quadangle.push_back(image_points_seq[i-1][this->board_size.width * (this->board_size.height-1)]);
		quadangle.push_back(image_points_seq[i-1][this->board_size.height*this->board_size.width - 1]);

		cout << "2222" << endl;
		srcTri[i-1] = cornersort(tmp_quadangle, quadangle);
		cout << "3333" << endl;

		circle(showpic, image_points_seq[i-1][0], 10, Scalar(255, 0, 0), 3);
		circle(showpic, image_points_seq[i-1][this->board_size.width - 1], 10, Scalar(0, 0, 255), 3);
		circle(showpic, image_points_seq[i-1][this->board_size.width * (this->board_size.height - 1)], 10, Scalar(0, 255, 0), 3);
		circle(showpic, image_points_seq[i-1][this->board_size.height * this->board_size.width - 1], 10, Scalar(255, 0, 255), 3);

		//imshow("showPic", showpic);
		//waitKey(0);

		cout << image_points_seq[i-1][0] << endl;
		cout << image_points_seq[i-1][this->board_size.width * (this->board_size.height - 1)] << endl;
		cout << image_points_seq[i-1][this->board_size.width - 1] << endl;
		cout << image_points_seq[i-1][this->board_size.height * this->board_size.width - 1] << endl;

		//cout << this->src[i].cols << " " << this->src[i].rows << endl;

		//srcTri[0] = Point2f(this->image_points_seq[i - 1][28].x, this->image_points_seq[i - 1][28].y);
		//srcTri[1] = Point2f(this->image_points_seq[i - 1][34].x, this->image_points_seq[i - 1][34].y);
		//srcTri[2] = Point2f(this->image_points_seq[i - 1][0].x, this->image_points_seq[i - 1][0].y);
		//srcTri[3] = Point2f(this->image_points_seq[i - 1][6].x, this->image_points_seq[i - 1][6].y);

		disTri[i-1].push_back(Point2f(0, 0));
		disTri[i-1].push_back(Point2f(this->src[i].cols, 0));
		disTri[i-1].push_back(Point2f(0, this->src[i].rows));
		disTri[i-1].push_back(Point2f(this->src[i].cols, this->src[i].rows));

		cout << "??????" << endl;
		Mat warpPerspective_mat = getPerspectiveTransform(srcTri[i-1], disTri[i-1]);
		warpPerspective(this->src[i], this->unproject_src[i], warpPerspective_mat, this->image_size);
		this->src[i] = this->unproject_src[i];
		cvtColor(this->src[i], this->gray_src[i], COLOR_BGR2GRAY);
		//imshow("unproject", this->unproject_src[i]);
		//waitKey(0);
	}
	cout << endl;
}

void opencv_iterative::reproject() {
	int ln = (int)this->src_count;
	for (int i = 1; i <= ln; i++) {
		cout << "i: " << i << "   ";
		// 后续换底板时需要修改！！！！

		Mat warpPerspective_mat = getPerspectiveTransform(disTri[i-1], srcTri[i-1]);
		warpPerspective(this->src[i], this->unproject_src[i], warpPerspective_mat, this->image_size);

		//imshow("unproject", this->unproject_src[i]);
		//waitKey(0);

		int l = (int)image_points_seq[i-1].size();
		for (int j = 0; j < l; j++) {
			Mat a = (Mat_<double>(3, 1) << image_points_seq[i-1][j].x, image_points_seq[i-1][j].y, 1);
			Mat y = warpPerspective_mat * a ;
			image_points_seq[i-1][j] = Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0));
		}

		//Mat view_gray = this->unproject_src[i];
		//drawChessboardCorners(view_gray, this->board_size, image_points_seq[i-1], true);

		//imshow("view_gray", view_gray);
		//waitKey(0);

	}
	cout << endl;
}

vector<Point2f> opencv_iterative::cornersort(vector<Point2f> tmp, vector<Point2f> res) {
	vector <Point2f> ans;
	int ln = (int)res.size();
	for (int i = 0; i < ln; i++) {
		int min = 0x3f3f3f3f, p = 0;
		for (int j = 0; j < ln; j++) {
			int dis = (tmp[j].x - res[i].x) * (tmp[j].x - res[i].x) + (tmp[j].y - res[i].y) * (tmp[j].y - res[i].y);
			if (dis < min) {
				min = dis; p = j;
			}
		}
		ans.push_back(tmp[p]);
	}
	return ans;
}