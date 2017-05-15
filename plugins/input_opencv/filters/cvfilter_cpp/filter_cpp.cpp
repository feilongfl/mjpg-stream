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

Mat ColorFinder(Mat src,HSVRange hsvRange = {100,120,0,255,0,255},int elementSize = 0)
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

    if (elementSize != 0) {
        //开操作 (去除一些噪点)
        Mat element = getStructuringElement(MORPH_RECT, Size(elementSize, elementSize));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

        //闭操作 (连接一些连通域)
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
    }

    return imgThresholded;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
    // TODO insert your filter code here
    dst = src;
    //cvtColor(src,dst,CV_BGR2GRAY);
    Mat calMat;
    calMat = ColorFinder(src);
    Mat can;
    Canny(calMat,can,3,9,3);
    //dst = can;
    Mat bin;
    cvtColor(can, bin, COLOR_GRAY2BGR);
    vector<Vec2f> lines;
    HoughLines(can, lines, 1, CV_PI/180, 100, 0, 0 );

    int rhoAverageH=0;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float theta = lines[i][1];
        if(theta > CV_PI / 4 && theta < CV_PI * 3 / 4) {
            rhoAverageH += lines[i][0];
        }
    }
    rhoAverageH /= lines.size();


//    cout << "################################" << endl;
    struct lines_s {
        Vec2f line;
        float rho;
        float theta;
        Point pt1;
        Point pt2;
    };
    vector<lines_s> lineUps,lineDowns,lineLefts,lineRights;
    lines_s lineUp,lineDown,lineLeft,lineRight;
    //区分上下左右
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

        lines_s l = {lines[i],rho,theta,pt1,pt2};
        
        if(theta > CV_PI / 4 && theta < CV_PI * 3 / 4) {
            if(rho > rhoAverageH){//下
                lineDowns.push_back(l);
            }
            else//上
            {
                lineUps.push_back(l);
            }
        }
        else if(theta > CV_PI * 3 / 4 && theta < CV_PI * 5 / 4)//右
        {
            lineRights.push_back(l);
        }
        else//左
        {
            lineLefts.push_back(l);
        }
    }
    //cout << "################################" << endl;

    //下面，找rho最小
    //TODO: 注意此处数组越界！！！
    lineDown = lineDowns[0];
    lineUp = lineUps[0];
    lineLeft = lineLefts[0];
    lineRight = lineRights[0];

    for (size_t i = 0;i < lineDowns.size();i++)
    {
        lineDown = (lineDowns[i].rho < lineDown.rho)?
                   lineDowns[i] : lineDown;
    }
    line(dst, lineDown.pt1, lineDown.pt2, Scalar(0, 0, 255), 3, CV_AA);
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

