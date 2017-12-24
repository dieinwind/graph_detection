#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

//////////////////////////////////////////////////////////////////
//�������ܣ�����������COS��=������֮��/������ģ�ĳ˻��������߶μн�
//���룺   �߶�3��������pt1,pt2,pt0,���һ������Ϊ������
//�����   �߶μнǣ���λΪ�Ƕ�
//////////////////////////////////////////////////////////////////
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	double angle_line = (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);//����ֵ
	return acos(angle_line) * 180 / 3.141592653;
}

/*************************************************************
�������ܣ����ø����ķ�Χ��ͼ�������ֵ��
���룺   Mat& image ԭʼͼ��
		Mat& result ������
		int *range ɫ�ʷ�Χ
�����   void
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
�������ܣ���ͼ����һ��ǰ������˹ģ������HSV�ռ�����ɫ����
���룺   Mat& image ԭʼͼ��
		Mat& blue_result ��ɫ����Ϊ��ɫ����������Ϊ��ɫ
		Mat& red_result  ��ɫ������Ϊ��ɫ����������Ϊ��ɫ
�����   void
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
�������ܣ������ı���
���룺   Mat& src, ԭʼͼ��
		vector<vector<Point>> &squares ���ڴ�Ų��ҵ��ľ��ν��
		vector<float> &direction ���ڴ�Ų��ҵ��ľ��εķ���
		vector<Point2f> &center ���ڴ�Ų��ҵ��ľ��ε����ĵ�
		int minarea, int maxarea  ɸѡ���ε������������ڴ˷�Χ�Ĳ���Ϊ���
�����   void
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


	//������
	//Mat element = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), Point(size, size));
	//morphologyEx(gray, gray, MORPH_OPEN, element); //ȥ����С����������
	
	Canny(gray, gray, 100, 100 * 2, 3);
	imshow("Canny", gray);
	//��ֵ��
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
			//??������һ�����Ƕȵģ�
			//cout << "Area = " << fabs(contourArea(contours_poly[i], false)) << endl;
			//cout << "squares = " << fabs(contourArea(contours_poly[i], false)) << endl;
			squares.push_back(contours_poly[i]);
			//drawContours(image, contours_poly, i, Scalar(0, 0, 0), 2, 8);  //����
		}

	}
}

/*************************************************************
�������ܣ�����������
���룺   Mat& src, ԭʼͼ��
vector<vector<Point>> &triangle ���ڴ�Ų��ҵ��������ν��
vector<float> &direction ���ڴ�Ų��ҵ��������εķ���
vector<Point2f> &center ���ڴ�Ų��ҵ��������ε����ĵ�
int minarea, int maxarea  ɸѡ�����ε������������ڴ˷�Χ�Ĳ���Ϊ���
�����   void
**************************************************************/
void findTriangle(Mat& src, vector<vector<Point>> &triangle, int minarea, int maxarea)
{
	Mat image = src.clone();
	Mat gray;
	vector<vector<Point> > contours; //��ʱ���δ��ɸѡ�Ķ����
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
	vector<vector<Point>> contours_poly(contours.size()); //���ڴ�Ŷ���αƽ�֮��Ķ����
	for (int i = 0; i < contours.size(); i++)
	{
		
		approxPolyDP(Mat(contours[i]), contours_poly[i], 12, true);
		if (contours_poly[i].size() == 3 && fabs(contourArea(contours_poly[i], false)) > minarea && fabs(contourArea(contours_poly[i], false)) < maxarea && isContourConvex(contours_poly[i]))
		{
			//??������һ�����Ƕȵģ�
			//cout << "Area = " << fabs(contourArea(contours_poly[i], false)) << endl;
			//cout << "squares = " << fabs(contourArea(contours_poly[i], false)) << endl;
			triangle.push_back(contours_poly[i]);
			//drawContours(image, contours_poly, i, Scalar(0, 0, 0), 2, 8);  //����
		}

	}
}


void findCircles(Mat& src, vector<Vec3f> &circles,  int minarea, int maxarea)
{
	Mat image = src.clone();

	//ǰ����
	int GREEN_range[6] = { 20, 60, 120, 200, 90, 220 };
	Mat gray;
	int ksize1 = 3;
	int ksize2 = 3;
	double sigma1 = 10;
	double sigma2 = 20;
	GaussianBlur(image, image, cv::Size(ksize1, ksize2), sigma1, sigma2);
	doubleThreshold(image, gray, GREEN_range);
	cvtColor(gray, gray, CV_BGR2GRAY);

	//��Եģ��������
	ksize1 = 11;
	ksize2 = 11;
	sigma1 = 10;
	sigma2 = 20;
	GaussianBlur(gray, gray, cv::Size(ksize1, ksize2), sigma1, sigma2);
	
	//��ֵ��
	//threshold(gray, gray, 160, 255, THRESH_BINARY);
	//Canny(gray, gray, 100, 100 * 2, 3);
	//imshow("Canny", gray);

	//����Բ�任
	/*
	src_gray: ����ͼ�� (�Ҷ�ͼ)
	circles: �洢������������: x_{c}, y_{c}, r ���ϵ���������ʾÿ����⵽��Բ.
	CV_HOUGH_GRADIENT: ָ����ⷽ��. ����OpenCV��ֻ�л����ݶȷ�
	dp = 1: �ۼ���ͼ��ķ��ȷֱ���
	min_dist = src_gray.rows/8: ��⵽Բ��֮�����С����
	param_1 = 200: Canny��Ե�����ĸ���ֵ
	param_2 = 100: Բ�ļ����ֵ.
	min_radius = 0: �ܼ�⵽����СԲ�뾶, Ĭ��Ϊ0.
	max_radius = 0: �ܼ�⵽�����Բ�뾶, Ĭ��Ϊ0
	*/
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 5, 10, 200, 100, 0, 0);
}

//////////////////////////////////////////////////////////////////
//�������ܣ��������о���
//���룺   img ԭͼ��
//          squares ��������
//          wndname ��������
//�����   ͼ���б�Ǿ���
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
void on_mouse(int event, int x, int y, int flags, void *ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���
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
	VideoCapture cap(1); //������ͷ
	if (!cap.isOpened()) //
	{
		cout << "Open cam failed!!" << endl;
		getchar();
		return -1;
	}
	Mat frame;
	cap >> image; //��׽һ֡
	Mat blue_result, red_result, green_result;
	
	//��������
	string windowName = "original";
	string windowName1 = "Canny";
	namedWindow(windowName);
	namedWindow(windowName1);
	//������궯�������ڵ���
	setMouseCallback(windowName, on_mouse, 0);
	bool stop = false;
	while (!stop)
	{
		// Canny();
		cap >> image;
		//preProcess(image, blue_result, red_result, green_result);
		
		//??���ӻ���У��
		
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