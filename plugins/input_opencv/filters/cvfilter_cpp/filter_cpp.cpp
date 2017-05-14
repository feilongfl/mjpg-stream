/**
    Example C++ OpenCV filter plugin that doesn't do anything. Copy/paste this
    to create your own awesome filter plugins for mjpg-streamer.

    At the moment, only the input_opencv.so plugin supports filter plugins.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>  
#include <termios.h>  
#include <unistd.h>  
#include <fcntl.h>  


int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

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


int changeVal(int add,int cut,int val,int step,int max = 255,int min = 0)
{
	int key;
	if (!kbhit())
		return val;
	else
		key = getchar();
	cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;

	if (key == add)
		val = (val < max - step)? val + step : val;
	else if (key == cut)
		val = (val > min + step) ? val - step : val;

	return val;
}
/**
    Called by the OpenCV plugin upon each frame
*/
int a = 60, b = 50, c = 0;
void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
    // TODO insert your filter code here
    //dst = src;
    //cvtColor(src,dst,CV_BGR2GRAY);
    Mat calMat;
    calMat = ColorFinder(src);
    Mat can;
    Canny(calMat,can,100,200,3);


    vector<Vec4i> lines;
	

	changeVal('q', 'w', a, 1);
	changeVal('a', 's', b, 1);
	changeVal('z', 'x', c, 1);
    //cout << "input a b c:";
    //cin >> a >> b >> c;
    //cout << endl;
    cout << "[a,b,c] = [" << a <<"." << b << "," << c << "]" << endl;
    HoughLinesP(can,lines,1,CV_PI / a, b, c);
    //dst = Mat(src.rows,src.cols,src.type());
	dst = src;
	cout << "lines.size = " << lines.size() << endl;
    for (size_t i = 0;i < lines.size();i++)
    {
        Vec4i l = lines[i];
        line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,255,0));
    }
    //dst = can;

}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
    // empty
}

