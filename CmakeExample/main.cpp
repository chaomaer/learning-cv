#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void rotate_img(int x, void *img){
    Mat rimg;
    Mat src = *(Mat*)img;
    Mat rotate = getRotationMatrix2D(Point(src.cols/2, src.rows/2), x, 1);
    warpAffine(src, rimg,rotate, src.size(), INTER_LINEAR);
    imshow("demo", rimg);
}


int main(){
    Mat img = imread("sq.png", CV_LOAD_IMAGE_COLOR);
    namedWindow("demo");
    createTrackbar("rotate", "demo", nullptr, 360, rotate_img, (void*)&img);
    imshow("demo",img);
    waitKey(0);
}
