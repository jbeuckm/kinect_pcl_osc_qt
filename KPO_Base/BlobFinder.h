// from http://areshopencv.blogspot.com/2011/09/blobs-with-opencv-internal-function.html

#ifndef BLOBFINDER_H
#define BLOBFINDER_H


#include <opencv/cv.h>

using namespace cv;

class BlobFinder
{

public:

    BlobFinder(cv::Mat src)
    {
        numBlobs = 0;
        cv::Mat img; //must create a temporary Matrix to hold the gray scale or wont work
        cv::cvtColor(src, img, CV_BGR2GRAY); //Convert image to GrayScale
        img = img > 1; //create the binary image
        ////cv::adaptiveThreshold(src,src,64,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,13); //create a binary image

        findContours( img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); //Find the Contour BLOBS
        vector<cv::Moments> _mu(contours.size() );
        vector<cv::Point2f> _mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
            _mu[i] = moments( Mat(contours[i]), false );
            _mc[i] = Point2f( _mu[i].m10/_mu[i].m00 , _mu[i].m01/_mu[i].m00);
        }
        mu = _mu;
        mc = _mc;
        numBlobs = contours.size();
    }

/*
    void Draw(cv::Mat &dst)
    {
        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        for( int i = 0; i < contours.size(); i++ )
        {
            Scalar color(  rng.uniform(0,255),  rng.uniform(0,255),  rng.uniform(0,255) );
            drawContours( dst, contours, i, color, CV_FILLED, 8, hierarchy );
            // drawCross(mc[i],Scalar(0,0,255), 5,dst); //put a cross
            char buff[255];
            sprintf(buff, "%d", i);

            string text = std::string(buff);
            cv::putText(dst,text,mc[i],0,0.5,Scalar(0,0,255),1,8,false);
        }
    }
*/

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    vector<cv::Moments> mu;
    vector<cv::Point2f> mc;
    int numBlobs;
};


#endif // BLOBFINDER_H
