/**
Example C++ OpenCV filter plugin that doesn't do anything. Copy/paste this
to create your own awesome filter plugins for mjpg-streamer.

At the moment, only the input_opencv.so plugin supports filter plugins.
*/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#include "filter_cpp.h"

// exports for the filter
extern "C" {
	bool filter_init(const char * args, void** filter_ctx);
	void filter_process(void* filter_ctx, Mat &src, Mat &trans);
	void filter_free(void* filter_ctx);
}


/**
Initializes the filter. If you return something, it will be passed to the
filter_process function, and should be freed by the filter_free function
*/
bool filter_init(const char * args, void** filter_ctx) {
	cout << "############################" << endl
		<< "##      init filter!      ##" << endl
		<< "############################" << endl;
	return true;
}

//色彩识别，默认蓝色，不进行开闭区间操作
Mat ColorFinder(Mat src, HSVRange hsvRange = { 100,120,0,255,0,255 }, int elementSize = 0)
{
	Mat imgHSV;
	vector<Mat> hsvSplit;

	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

										  //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat imgThresholded;

	//Threshold the image
	inRange(imgHSV, Scalar(hsvRange.iLowH, hsvRange.iLowS, hsvRange.iLowV),
		Scalar(hsvRange.iHighH, hsvRange.iHighS, hsvRange.iHighV), imgThresholded);

	if (elementSize != 0) {
		//开操作 (去除一些噪点)
		Mat element = getStructuringElement(MORPH_RECT, Size(elementSize, elementSize));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

		//闭操作 (连接一些连通域)
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	}

	return imgThresholded;
}


////////////////////////////////////////////
//求两直线交点
Point CrossPoint(const lines_s *line1, const lines_s *line2)
{
	Point pt = { 0,0 };
	// line1's cpmponent
	double X1 = line1->pt2.x - line1->pt1.x;//b1
	double Y1 = line1->pt2.y - line1->pt1.y;//a1
											// line2's cpmponent
	double X2 = line2->pt2.x - line2->pt1.x;//b2
	double Y2 = line2->pt2.y - line2->pt1.y;//a2
											// distance of 1,2
	double X21 = line2->pt1.x - line1->pt1.x;
	double Y21 = line2->pt1.y - line1->pt1.y;
	// determinant
	double D = Y1*X2 - Y2*X1;// a1b2-a2b1
							 //
	if (D == 0) return pt;
	// cross point
	pt.x = (X1*X2*Y21 + Y1*X2*line1->pt1.x - Y2*X1*line2->pt1.x) / D;
	// on screen y is down increased !
	pt.y = -(Y1*Y2*X21 + X1*Y2*line1->pt1.y - X2*Y1*line2->pt1.y) / D;
	// segments intersect.
	if ((abs(pt.x - line1->pt1.x - X1 / 2) <= abs(X1 / 2)) &&
		(abs(pt.y - line1->pt1.y - Y1 / 2) <= abs(Y1 / 2)) &&
		(abs(pt.x - line2->pt1.x - X2 / 2) <= abs(X2 / 2)) &&
		(abs(pt.y - line2->pt1.y - Y2 / 2) <= abs(Y2 / 2)))
	{
		return pt;
	}
	return pt;
}
//////////////////////////////////////////////////////////////////////////
//梯形校正

//计算水平直线rho平均值
int HorizontalLineRhoAverage(vector<Vec2f> lines)
{
	int rhoAverageH = 0;
	for (size_t i = 0; i < lines.size(); i++)
	{
		float theta = lines[i][1];
		if (theta > CV_PI / 4 && theta < CV_PI * 3 / 4) {
			rhoAverageH += lines[i][0];
		}
	}
	rhoAverageH /= lines.size();
	return rhoAverageH;
}

//直线分类
lines_s4v DistinguishLines(vector<Vec2f> lines)
{
	lines_s4v lineDist;
	
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));

		lines_s l = { lines[i],rho,theta,pt1,pt2 };

		if (theta > CV_PI / 4 && theta < CV_PI * 3 / 4)
		{
			if (rho > HorizontalLineRhoAverage(lines))//下
			{
				lineDist.lineDowns.push_back(l);
			}
			else//上
			{
				lineDist.lineUps.push_back(l);
			}
		}
		else if (theta > CV_PI * 3 / 4 && theta < CV_PI * 5 / 4)//右
		{
			lineDist.lineRights.push_back(l);
		}
		else//左
		{
			lineDist.lineLefts.push_back(l);
		}
	}

	return lineDist;
}

//拟合直线
lines_s4 LineFitting(lines_s4v lineDist)
{
	lines_s4 line4;
	//检测非零
	if (lineDist.lineDowns.size() == 0 ||
		lineDist.lineUps.size() == 0 ||
		lineDist.lineRights.size() == 0 ||
		lineDist.lineLefts.size() == 0)
	{
		//TODO: do sth here
		// example: throw ex
		throw "直线数量为零！";
	};

	line4.lineDown = lineDist.lineDowns[0];
	line4.lineUp = lineDist.lineUps[0];
	line4.lineLeft = lineDist.lineLefts[0];
	line4.lineRight = lineDist.lineRights[0];
	//TODO: 更换直线拟合算法
	for (size_t i = 0; i < lineDist.lineDowns.size(); i++)
	{
		line4.lineDown = (lineDist.lineDowns[i].rho < line4.lineDown.rho) ?
			lineDist.lineDowns[i] : line4.lineDown;
	}
	//line(trans, lineDown.pt1, lineDown.pt2, Scalar(0, 0, 255), 3, CV_AA);
	//上面，找rho最大
	for (size_t i = 0; i < lineDist.lineUps.size(); i++)
	{
		line4.lineUp = (lineDist.lineUps[i].rho > line4.lineUp.rho) ?
			lineDist.lineUps[i] : line4.lineUp;
	}
	//line(trans, lineUp.pt1, lineUp.pt2, Scalar(0, 255, 255), 3, CV_AA);
	//左面，找rho最大
	for (size_t i = 0; i < lineDist.lineLefts.size(); i++)
	{
		line4.lineLeft = (lineDist.lineLefts[i].rho > line4.lineDown.rho) ?
			lineDist.lineLefts[i] : line4.lineLeft;
	}
	//line(trans, lineLeft.pt1, lineLeft.pt2, Scalar(0, 255, 0), 3, CV_AA);
	//右面，找rho最大（rho负数）
	for (size_t i = 0; i < lineDist.lineRights.size(); i++)
	{
		line4.lineRight = (lineDist.lineRights[i].rho < line4.lineDown.rho) ?
			lineDist.lineRights[i] : line4.lineRight;
	}
	//line(trans, lineRight.pt1, lineRight.pt2, Scalar(255, 255, 0), 3, CV_AA);

	return line4;
}

//计算由四条直线组成封闭图形角点坐标
vector<Point> RectCrossCalc(lines_s4 rectLine)
{
	vector<Point> r;

	r.push_back(CrossPoint(&rectLine.lineLeft, &rectLine.lineUp));
	r.push_back(CrossPoint(&rectLine.lineRight, &rectLine.lineUp));
	r.push_back(CrossPoint(&rectLine.lineLeft, &rectLine.lineDown));
	r.push_back(CrossPoint(&rectLine.lineRight, &rectLine.lineDown));

	return r;
}

//返回标准矩形
vector<Point> getCorners(Mat src)
{
	vector<Point> corners(4);//标准正方形

	corners[0] = Point(0, 0);
	corners[1] = Point(src.cols - 1, 0);
	corners[2] = Point(0, src.rows - 1);
	corners[3] = Point(src.cols - 1, src.rows - 1);

	return corners;
}

//梯形校正
Mat KeystoneCorrection(Mat src,Mat oriSrc)//去除背景图像，原始图像
{
	Mat dst;
	//canny
	Mat can;
	Canny(src, can, 3, 9, 3);

	//Mat canBgr;
	//cvtColor(can, canBgr, COLOR_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(can, lines, 1, CV_PI / 180, 100, 0, 0);

	//区分上下左右
	lines_s4v lineDist = DistinguishLines(lines);
	
	//直线拟合，每个方向留下一根
	lines_s4 line4 = LineFitting(lineDist);
	//计算直线交点坐标
	vector<Point> cornersRect = RectCrossCalc(line4);

	//准备标准矩形
	vector<Point> corners = getCorners(src);

	//梯形矫正
	Mat transform = findHomography(cornersRect, corners);
	warpPerspective(oriSrc, dst, transform, src.size());

	return dst;
}
//////////////////////////////////////////////////////////////////////////

/**
Called by the OpenCV plugin upon each frame
*/
Mat LastImg;
bool work = false;

void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
	// TODO insert your filter code here

	Mat trans(src);

	Mat calMat;

	try
	{
		calMat = ColorFinder(src); //背景提取
		calMat = KeystoneCorrection(calMat, src);//梯形校正
		HSVRange hsv = { 100,120,0,255,0,255 };
		dst = ColorFinder(calMat,hsv,11); //背景提取





		//save last
		LastImg = dst;
		work = true;
	}
	catch (char const* ex)
	{
		cout << "error: " << ex << endl;
		if(work)//read last
			dst = LastImg;
	}
	catch (cv::Exception ex)
	{
		cout << "cvExp: " << ex.what() << endl;
	}
	
}

/**
Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
	// empty
	cout << "############################" << endl
		<< "##      free filter!      ##" << endl
		<< "############################" << endl;

}

