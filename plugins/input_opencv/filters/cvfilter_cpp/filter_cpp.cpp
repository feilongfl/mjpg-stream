/**
    Example C++ OpenCV filter plugin that doesn't do anything. Copy/paste this
    to create your own awesome filter plugins for mjpg-streamer.

    At the moment, only the input_opencv.so plugin supports filter plugins.
*/

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

// exports for the filter
extern "C" {
bool filter_init(const char * args, void** filter_ctx);
void filter_process(void* filter_ctx, Mat &src, Mat &dst);
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

struct HSVRange{
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
};

Mat ColorFinder(Mat src,HSVRange hsvRange = {100,120,0,255,0,255})
{
    Mat imgHSV;
    vector<Mat> hsvSplit;

    cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,imgHSV);
    Mat imgThresholded;

    //Threshold the image
    inRange(imgHSV, Scalar(hsvRange.iLowH, hsvRange.iLowS, hsvRange.iLowV),
            Scalar(hsvRange.iHighH, hsvRange.iHighS, hsvRange.iHighV), imgThresholded);

    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    //闭操作 (连接一些连通域)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    return imgThresholded;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
    // TODO insert your filter code here
    //dst = src;
    //cvtColor(src,dst,CV_BGR2GRAY);
    Mat calMat;
    calMat = ColorFinder(src);
    Mat can;
    Canny(calMat,can,3,9,3);
    dst = calMat;
    Mat bin;
    cvtColor(can, bin, COLOR_GRAY2BGR);
    vector<Vec2f> lines;
    HoughLines(can, lines, 1, CV_PI/180, 100, 0, 0 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        //line( dst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
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

