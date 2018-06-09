//
// Created by chaomaer on 18-6-9.
//

#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat compress_svd(Mat & mat_, const Size &size, int deep = 1){
    Mat w, u, vt;
    SVD::compute(mat_, w, u, vt,SVD::FULL_UV);
    // 重新构建一个sigma
    Mat sigma = Mat::zeros(size, CV_32F);
    for (int i = 0; i < deep; ++i) {
        sigma.at<float>(i, i) = w.at<float>(i, 0);
    }
    return u*sigma*vt;
}
Mat svd(Mat img, int deep=1){
    Mat chanel[3];
    split(img, chanel);
    vector<Mat> vector1;
    //转化为float
    for (auto &i : chanel) {
        i.convertTo(i, CV_32F);
        vector1.push_back(compress_svd(i, img.size(), deep));
    }
    merge(vector1, img);
    return img;
}
int main(){
    Mat img = imread("data/lenna.png", CV_32F);
    for (int i = 1; i < 10; ++i) {
        Mat rimg = svd(img, 10*i);
        imwrite("data/rimg.png", rimg);
        rimg = imread("data/rimg.png");
        imshow("svd"+to_string(i*10),rimg);
        waitKey(0);
    }
}
