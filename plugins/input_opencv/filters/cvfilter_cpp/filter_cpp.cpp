/**
    Example C++ OpenCV filter plugin that doesn't do anything. Copy/paste this
    to create your own awesome filter plugins for mjpg-streamer.

    At the moment, only the input_opencv.so plugin supports filter plugins.
*/

#include "opencv2/opencv.hpp"

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

Mat ColorFinder(Mat src,HSVRange hsvRange = {100,124,0,255,0,255})
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
    Mat calcMat,dst_norm_scaled;
    calcMat = ColorFinder(src);//赛道轮廓识别
    //cornerHarris(calcMat, dst, 2, 3, 0.01);//搜索赛道角点

    int maxCorners = 4;
    if( maxCorners < 1 ) { maxCorners = 1; }
    /// Parameters for Shi-Tomasi algorithm
    vector<Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    /// Copy the source image
    //Mat cormat;
    /// Apply corner detection :Determines strong corners on an image.
    goodFeaturesToTrack( calcMat,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         Mat(),
                         blockSize,
                         useHarrisDetector,
                         k );
    /// Draw corners detected
    for( int i = 0; i < corners.size(); i++ ){
        circle( dst_norm_scaled,  corners[i], 5,  Scalar(255), 2, 8, 0 );
        //circle( src, corners[i], 4, Scalar(0,255,0), 2, 8, 0 );
    }
    dst = dst_norm_scaled;
    //////////////////////////////////////////////////////////////////////
    //TODO:
    //赛道仿射变换
    //前景检测
}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
    // empty
}

