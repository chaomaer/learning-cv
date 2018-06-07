#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

static void
StereoCalib(const vector<string> &imagelist, Size boardSize) {
    const float squareSize = 1.f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int) imagelist.size() / 2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for (i = j = 0; i < nimages; i++) {
        for (k = 0; k < 2; k++) {
            const string &filename = imagelist[i * 2 + k];
            Mat img = imread(filename, 0);
            imageSize = img.size();
            vector<Point2f> &corners = imagePoints[k][j];
            findChessboardCorners(img, boardSize, corners,
                                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
                                      30, 0.01));
        }
        if (k == 2) {
            goodImageList.push_back(imagelist[i * 2]);
            goodImageList.push_back(imagelist[i * 2 + 1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if (nimages < 2) {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for (i = 0; i < nimages; i++) {
        for (j = 0; j < boardSize.height; j++)
            for (k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;
    // 重要函数1
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
                                 CV_CALIB_FIX_ASPECT_RATIO +
                                 CV_CALIB_ZERO_TANGENT_DIST +
                                 CV_CALIB_SAME_FOCAL_LENGTH +
                                 CV_CALIB_RATIONAL_MODEL +
                                 CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;


    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    // 重要函数2
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    Mat rmap[2][2];

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    // 对对第一张图片进行处理
    for (k = 0; k < 2; k++) {
        Mat img = imread(goodImageList[k], 0), rimg, cimg;
        remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
        imshow(goodImageList[k] + "", rimg);
        waitKey(0);
    }
}


static bool readStringList(const string &filename, vector<string> &l) {
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string) *it);
    return true;
}

int main(int argc, char **argv) {
    Size boardSize(9, 6);
    string imagelistfn = "imagelist.yaml";;

    vector<string> imagelist;
    readStringList(imagelistfn, imagelist);
    StereoCalib(imagelist, boardSize);
    return 0;
}