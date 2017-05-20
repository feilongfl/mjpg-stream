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
	size_t i;
	for (i = 0; i < lines.size(); i++)
	{
		float theta = lines[i][1];
		if (theta > CV_PI / 4 && theta < CV_PI * 3 / 4) {
			rhoAverageH += lines[i][0];
		}
	}
	rhoAverageH /= i;
	return rhoAverageH;
}


lines_s lines2lines_s(Vec2f lines)
{
    float rho = lines[0], theta = lines[1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));

    lines_s l = { lines,rho,theta,pt1,pt2 };
    return l;
}
//直线分类
lines_s4v DistinguishLines(vector<Vec2f> lines)
{
	lines_s4v lineDist;

	for (size_t i = 0; i < lines.size(); i++)
	{
        lines_s l = lines2lines_s( lines[i] );

        if (l.theta > CV_PI / 4 && l.theta < CV_PI * 3 / 4)
		{
			/*
			if (l.rho > HorizontalLineRhoAverage(lines))//下
			{
				lineDist.lineDowns.push_back(l);
			}
			else//上
			{
				lineDist.lineUps.push_back(l);
			}
			*/
			lineDist.lineUps.push_back(l);

		}
		else if (l.theta > CV_PI * 3 / 4 && l.theta < CV_PI * 5 / 4)//右
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
Mat KeystoneCorrection(Mat src,Mat oriSrc,bool debug = false)//去除背景图像，原始图像
{
	Mat dst;
	//canny
	Mat can;
	Canny(src, can, 1, 3, 3);
    //return can;
    //cvtColor(src,can,COLOR_BGR2GRAY);
	//Mat canBgr;
	//cvtColor(can, canBgr, COLOR_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(can, lines, 1, CV_PI / 360, 70, 0, 0);

//////////////////////////////////////

    //debug
    dst = oriSrc;

	/*
	cout << "###############################" << endl;
    for (size_t i = 0; i < lines.size();i++)
    {
        line(dst,lines2lines_s(lines[i]).pt1,lines2lines_s(lines[i]).pt2,Scalar(0,0,255));
		cout << lines2lines_s(lines[i]).rho << "," << lines2lines_s(lines[i]).theta << endl;
    }
*/
    //return dst;
//////////////////////////////////////
	//区分上下左右
	lines_s4v lineDist = DistinguishLines(lines);

	cout << "################################" << endl;
	for (size_t i = 0; i < lineDist.lineUps.size();i++)
	{
		line(dst,lineDist.lineUps[i].pt1,lineDist.lineUps[i].pt2,Scalar(0,0,255));
		cout << lineDist.lineUps[i].rho << "," << lineDist.lineUps[i].theta << endl;
	}

	return dst;

	//直线拟合，每个方向留下一根
	lines_s4 line4 = LineFitting(lineDist);

    ///this is BUG!!!
    if(abs(line4.lineUp.rho - line4.lineDown.rho) < 10)
        throw "上下直线误判";
	//计算直线交点坐标
	vector<Point> cornersRect = RectCrossCalc(line4);

	//准备标准矩形
	vector<Point> corners = getCorners(src);

	//梯形矫正
	Mat transform = findHomography(cornersRect, corners);
    if(!debug)
	    warpPerspective(oriSrc, dst, transform, src.size());
    else
    {
        dst = oriSrc;
        line(dst,line4.lineUp.pt1,line4.lineUp.pt2,Scalar(0,0,255));
        line(dst,line4.lineDown.pt1,line4.lineDown.pt2,Scalar(0,0,255));
        line(dst,line4.lineLeft.pt1,line4.lineLeft.pt2,Scalar(0,0,255));
        line(dst,line4.lineRight.pt1,line4.lineRight.pt2,Scalar(0,0,255));
    }


	return dst;
}
//////////////////////////////////////////////////////////////////////////

/**
Called by the OpenCV plugin upon each frame
*/
Mat LastImg;
bool work = false;

//近似
bool Approximate(int num1, int num2, int error)
{
	return (abs(num1 - num2) < error);
}

void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
	// TODO insert your filter code here

	Mat trans(src);

	Mat calMat;

	try
	{
        HSVRange hsvR = {80,140,0,200,0,255};
		calMat = ColorFinder(src,hsvR); //背景提取

		calMat = KeystoneCorrection(calMat, src);//梯形校正
        dst = calMat;//存储彩图
        return;

		//HSVRange hsv = { 0,180,30,60,254,255 };
		calMat = ColorFinder(calMat,hsvR,5); //背景提取

		vector<vector<cv::Point>> contours;
		//连通域
		bitwise_not(calMat, calMat);//反色
		cv::findContours(calMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        if (contours.size()<=2)
            throw "error:区域数量过少";

		for (size_t i = 0; i < contours.size(); i++)
		{
            //for test
#define whlimit0 15
#define whlimit1 26
#define whlimit2 80
			cv::Rect r = cv::boundingRect(contours[i]);
			if (Approximate(r.height, r.width, 40)
                && r.x != 0 && r.y != 0
                && r.x + r.width < 640 && r.y + r.height < 480
                && (r.width > whlimit0 || r.height > whlimit0)
                    ) {
                cout << r.x << "," << r.y << "," << r.height << "," << r.width << ",";
                if(r.height < whlimit1 && r.width < whlimit1) {
                    cv::rectangle(dst, r, cv::Scalar(0, 0, 255),5);
                    cout << "r" << endl;
                }
                else if(r.height < whlimit2 && r.width < whlimit2) //亮起
                {
                    cv::rectangle(dst, r, cv::Scalar(0, 255, 0),5);
                    cout << "g" << endl;
                }
                else
                {
                    cv::rectangle(dst, r, cv::Scalar(0, 255, 255),5);
                    cout << "y" << endl;
                }
            }
		}
        cout << "####################" << endl;
        //dst = calMat;
		//////////////////////////////////////////////////////////////////////////
		//save last
		LastImg = dst;
		work = true;
	}
	catch (char const* ex)
	{
		cout << "error: " << ex << endl;
		if(work)//read last
			dst = LastImg;
        else
            dst = src;
	}
	catch (cv::Exception ex)
	{
		cout << "cvExp: " << ex.what() << endl;
		if(work)//read last
			dst = LastImg;
        else
            dst = src;
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

