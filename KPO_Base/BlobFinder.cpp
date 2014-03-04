
#include "BlobFinder.h"

void BlobFinder::find(cv::Mat img)
{
    numBlobs = 0;

    img = img > 1; //create the binary image
    ////cv::adaptiveThreshold(src,src,64,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,13); //create a binary image

    findContours( img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); //Find the Contour BLOBS

    numBlobs = contours.size();

    vector<vector<Point> > contours_poly( numBlobs );
    vector<cv::Moments> _mu( numBlobs );
    vector<cv::Point2f> _mc( numBlobs );
    vector<Point2f> _center( numBlobs );
    vector<float> _radius( numBlobs );

    for( int i = 0; i < contours.size(); i++ )
    {
        _mu[i] = moments( Mat(contours[i]), false );
        _mc[i] = Point2f( _mu[i].m10/_mu[i].m00 , _mu[i].m01/_mu[i].m00);

        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], _center[i], _radius[i] );

        contours[i] = contours_poly[i];
    }

    mu = _mu;
    mc = _mc;

    center = _center;
    radius = _radius;
}
