#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

//////////////////////////////////////////////////////////////////
//函数功能：用向量来做COSα=两向量之积/两向量模的乘积求两条线段夹角
//输入：   线段3个点坐标pt1,pt2,pt0,最后一个参数为公共点
//输出：   线段夹角，单位为角度
//////////////////////////////////////////////////////////////////
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	double angle_line = (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);//余弦值
	return acos(angle_line) * 180 / 3.141592653;
}

/*************************************************************
函数功能：利用给定的范围对图像进行阈值化
输入：   Mat& image 原始图像
		Mat& result 输出结果
		int *range 色彩范围
输出：   void
**************************************************************/
void doubleThreshold(Mat& image, Mat& result, int *range)
{
	Mat hsv_image;
	cvtColor(image, hsv_image, CV_RGB2HSV);
	image.copyTo(result);
	int channels = image.channels();
	int rows = image.rows;
	int cols = image.cols*channels;
	

	for (int j = 0; j<rows; ++j)
	{
		uchar *data = hsv_image.ptr<uchar>(j);
		uchar *res = result.ptr<uchar>(j);
		for (int i = 0; i<cols; i = i + channels)
		{
			//if (((data[i] > 90) & (data[i] < 140))&((data[i + 1] > 75) & (data[i + 1] < 105))&((data[i + 2] > 85) & (data[i + 2] < 105)))
			if (((data[i] > range[0])&(data[i] < range[1]))&((data[i + 1] > range[2])&(data[i + 1] < range[3]))&((data[i + 2] > range[4])&(data[i + 2] < range[5])))
			{
				res[i] = 255;
				res[i + 1] = 255;
				res[i + 2] = 255;
			}
			else
			{
				res[i] = 0;
				res[i + 1] = 0;
				res[i + 2] = 0;
			}

		}
	}
}

/*************************************************************
函数功能：对图像做一下前处理，高斯模糊，用HSV空间做颜色分离
输入：   Mat& image 原始图像
		Mat& blue_result 蓝色方块为白色，其他区域为黑色
		Mat& red_result  红色三角形为白色，其他区域为黑色
输出：   void
**************************************************************/
void preProcess(Mat& scr, Mat& blue_result, Mat& red_result, Mat& green_result)
{
	Mat Blue_image, Red_image, Green_image;
	scr.copyTo(Blue_image);
	//                  H_min,H_max,S_min,S_max,V_min,V_max
	int BLUE_range[6] = { 0,   20,   40,   240,  40,   240 };
	int RED_range[6] =  { 110, 130,  40,   200,  40,   220 };
	int GREEN_range[6] ={ 20, 40,    40,   200,  40,   220 };
	Mat result;
	int ksize1 = 11;
	int ksize2 = 11;
	double sigma1 = 10;
	double sigma2 = 20;
	GaussianBlur(Blue_image, Blue_image, cv::Size(ksize1, ksize2), sigma1, sigma2);
	doubleThreshold(Blue_image, result, BLUE_range);
	cvtColor(result, blue_result, CV_BGR2GRAY);
	doubleThreshold(Blue_image, result, RED_range);
	cvtColor(result, red_result, CV_BGR2GRAY);
	doubleThreshold(Blue_image, result, GREEN_range);
	cvtColor(result, green_result, CV_BGR2GRAY);
	

}

/*************************************************************
函数功能：查找四边形
输入：   Mat& src, 原始图像
		vector<vector<Point>> &squares 用于存放查找到的矩形结果
		vector<float> &direction 用于存放查找到的矩形的方向
		vector<Point2f> &center 用于存放查找到的矩形的中心点
		int minarea, int maxarea  筛选矩形的面积，面积不在此范围的不作为结果
输出：   void
**************************************************************/
void findSquares4(Mat& src, vector<vector<Point>> &squares, int minarea, int maxarea)
{
	Mat image = src.clone();
	Mat gray;
	vector<vector<Point> > contours;

	int size = 3;
	int BLUE_range[6] = { 0, 20, 120, 240, 100, 240 };
	Mat result;
	int ksize1 = 11;
	int ksize2 = 11;
	double sigma1 = 10;
	double sigma2 = 20;
	GaussianBlur(image, image, cv::Size(ksize1, ksize2), sigma1, sigma2);
	doubleThreshold(image, gray, BLUE_range);
	cvtColor(gray, gray, CV_BGR2GRAY);


	//开运算
	//Mat element = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), Point(size, size));
	//morphologyEx(gray, gray, MORPH_OPEN, element); //去除较小的明亮区域
	
	Canny(gray, gray, 100, 100 * 2, 3);
	imshow("Canny", gray);
	//二值化
	//threshold(gray, gray, 160, 255, THRESH_BINARY);
	
	vector<Vec4i> hierarchy;
	findContours(gray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	Mat contoursImage(gray.rows, gray.cols, CV_8U, Scalar(255));
	vector<vector<Point>> contours_poly(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{

		approxPolyDP(Mat(contours[i]), contours_poly[i], 12, true);
		if (contours_poly[i].size() == 4 && fabs(contourArea(contours_poly[i], false)) > minarea && fabs(contourArea(contours_poly[i], false)) < maxarea && isContourConvex(contours_poly[i]))
		{
			//??再增加一个检测角度的，
			//cout << "Area = " << fabs(contourArea(contours_poly[i], false)) << endl;
			//cout << "squares = " << fabs(contourArea(contours_poly[i], false)) << endl;
			squares.push_back(contours_poly[i]);
			//drawContours(image, contours_poly, i, Scalar(0, 0, 0), 2, 8);  //绘制
		}

	}
}

/*************************************************************
函数功能：查找三角形
输入：   Mat& src, 原始图像
vector<vector<Point>> &triangle 用于存放查找到的三角形结果
vector<float> &direction 用于存放查找到的三角形的方向
vector<Point2f> &center 用于存放查找到的三角形的中心点
int minarea, int maxarea  筛选三角形的面积，面积不在此范围的不作为结果
输出：   void
**************************************************************/
void findTriangle(Mat& src, vector<vector<Point>> &triangle, int minarea, int maxarea)
{
	Mat image = src.clone();
	Mat gray;
	vector<vector<Point> > contours; //暂时存放未经筛选的多边形
	int size = 3;

	int RED_range[6] = { 110, 150, 120, 200, 100, 220 };
	Mat result;
	int ksize1 = 5;
	int ksize2 = 5;
	double sigma1 = 10;
	double sigma2 = 20;
	GaussianBlur(image, image, cv::Size(ksize1, ksize2), sigma1, sigma2);
	doubleThreshold(image, gray, RED_range);
	cvtColor(gray, gray, CV_BGR2GRAY);

	
	ksize1 = 3;
	ksize2 = 3;
	sigma1 = 10;
	sigma2 = 20;
	GaussianBlur(gray, gray, cv::Size(ksize1, ksize2), sigma1, sigma2);
	//Canny(gray, gray, 100, 100 * 2, 3);
	

	vector<Vec4i> hierarchy;
	findContours(gray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	Mat contoursImage(gray.rows, gray.cols, CV_8U, Scalar(255));
	vector<vector<Point>> contours_poly(contours.size()); //用于存放多边形逼近之后的多边形
	for (int i = 0; i < contours.size(); i++)
	{
		
		approxPolyDP(Mat(contours[i]), contours_poly[i], 12, true);
		if (contours_poly[i].size() == 3 && fabs(contourArea(contours_poly[i], false)) > minarea && fabs(contourArea(contours_poly[i], false)) < maxarea && isContourConvex(contours_poly[i]))
		{
			//??再增加一个检测角度的，
			//cout << "Area = " << fabs(contourArea(contours_poly[i], false)) << endl;
			//cout << "squares = " << fabs(contourArea(contours_poly[i], false)) << endl;
			triangle.push_back(contours_poly[i]);
			//drawContours(image, contours_poly, i, Scalar(0, 0, 0), 2, 8);  //绘制
		}

	}
}


void findCircles(Mat& src, vector<Vec3f> &circles,  int minarea, int maxarea)
{
	Mat image = src.clone();

	//前处理
	int GREEN_range[6] = { 20, 60, 120, 200, 90, 220 };
	Mat gray;
	int ksize1 = 3;
	int ksize2 = 3;
	double sigma1 = 10;
	double sigma2 = 20;
	GaussianBlur(image, image, cv::Size(ksize1, ksize2), sigma1, sigma2);
	doubleThreshold(image, gray, GREEN_range);
	cvtColor(gray, gray, CV_BGR2GRAY);

	//边缘模糊化处理
	ksize1 = 11;
	ksize2 = 11;
	sigma1 = 10;
	sigma2 = 20;
	GaussianBlur(gray, gray, cv::Size(ksize1, ksize2), sigma1, sigma2);
	
	//二值化
	//threshold(gray, gray, 160, 255, THRESH_BINARY);
	//Canny(gray, gray, 100, 100 * 2, 3);
	//imshow("Canny", gray);

	//霍夫圆变换
	/*
	src_gray: 输入图像 (灰度图)
	circles: 存储下面三个参数: x_{c}, y_{c}, r 集合的容器来表示每个检测到的圆.
	CV_HOUGH_GRADIENT: 指定检测方法. 现在OpenCV中只有霍夫梯度法
	dp = 1: 累加器图像的反比分辨率
	min_dist = src_gray.rows/8: 检测到圆心之间的最小距离
	param_1 = 200: Canny边缘函数的高阈值
	param_2 = 100: 圆心检测阈值.
	min_radius = 0: 能检测到的最小圆半径, 默认为0.
	max_radius = 0: 能检测到的最大圆半径, 默认为0
	*/
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 5, 10, 200, 100, 0, 0);
}

//////////////////////////////////////////////////////////////////
//函数功能：画出所有矩形
//输入：   img 原图像
//          squares 矩形序列
//          wndname 窗口名称
//输出：   图像中标记矩形
//////////////////////////////////////////////////////////////////
void drawPolys(Mat& image, vector<vector<Point>> squares, vector<float> direction, vector<Point2f> center)
{
	int num_polys = squares.size();
	for (int i = 0; i < num_polys; i++)
	{
		vector<Point> ploy = squares[i];
		const int num_point = ploy.size();
		Point points[1][20];
		for (int j = 0; j < num_point; j++)
		{
			points[0][j] = ploy[j];
		}
		const Point* ppt[1] = { points[0] };
		int npt[] = { num_point };
		polylines(image, ppt, npt, 1, 1, Scalar(0*i * 50, i * 50, i * 50), 5);
	}
}

void drawCircles(Mat& image, vector<Vec3f> circles)
{
	
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}
}

Mat image;
void on_mouse(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		Mat hsv_image;
		cvtColor(image, hsv_image, CV_RGB2HSV);
		cout << "FUCK" << endl;
		cout << "H = " << int(hsv_image.at<cv::Vec3b>(y, x)[0]) ;
		cout << " S = " << int(hsv_image.at<cv::Vec3b>(y, x)[1]);
		cout << " V = " << int(hsv_image.at<cv::Vec3b>(y, x)[2]) << endl;
		getchar();
	}
}
int main()
{
	VideoCapture cap(1); //打开摄像头
	if (!cap.isOpened()) //
	{
		cout << "Open cam failed!!" << endl;
		getchar();
		return -1;
	}
	Mat frame;
	cap >> image; //捕捉一帧
	Mat blue_result, red_result, green_result;
	
	//创建窗口
	string windowName = "original";
	string windowName1 = "Canny";
	namedWindow(windowName);
	namedWindow(windowName1);
	//设置鼠标动作，便于调试
	setMouseCallback(windowName, on_mouse, 0);
	bool stop = false;
	while (!stop)
	{
		// Canny();
		cap >> image;
		//preProcess(image, blue_result, red_result, green_result);
		
		//??增加畸变校正
		
		vector<vector<Point>> squares;
		vector<vector<Point>> triangle;
		vector<float> direction;
		vector<Point2f> center;
		vector<Vec3f> circles;

		findSquares4(image, squares, 4000, 120000);
		findTriangle(image, squares, 1000, 120000);
		findCircles(image, circles, 100, 100);

		drawPolys(image, squares, direction, center);
		drawCircles(image, circles );
		imshow(windowName, image);
		if (waitKey(30) >= 0)
			stop = true;
	}
	return 0;
}