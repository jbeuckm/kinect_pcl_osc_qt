// from http://areshopencv.blogspot.com/2011/09/blobs-with-opencv-internal-function.html

#ifndef BLOBFINDER_H
#define BLOBFINDER_H


#include <opencv/cv.h>

using namespace cv;

class BlobFinder
{

public:
    vector< vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    vector<cv::Moments> mu;
    vector<cv::Point2f> mc;

    vector<Point2f>center;
    vector<float>radius;

    int numBlobs;

    void find(cv::Mat img);

};


#endif // BLOBFINDER_H
