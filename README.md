kinect_pcl_osc_qt
=================

An object recognition and reporting engine built for "Autonomous" by Peter Sowinski. A Qt interface allows manually triggered capture of object depth contours and pointclouds. Saved object models are correlated with the live camera input to find the objects in the scene.

Two recognition methods are used to find objects in the OpenNI stream:

1. The depth image is thresholded to a fixed distance. OpenCV is used to find contours in the binary image, then for each contour in the scene, [matchShapes()](http://opencv.jp/opencv-2.2_org/cpp/imgproc_structural_analysis_and_shape_descriptors.html#cv-matchshapes) is used to find the known contour with lowest error. The resulting object ID and 1.0/error is sent to the path /kinect/contour via OSC.

2. The incoming pointcloud is cropped to a set rectangular prism to avoid extra processing of the irrelevant surroundings. [A PCL pipeline](http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping) is used to correlate sampled keypoints and SHOT descriptors with those in a set of known models. If matches are found with their 6DOF pose, object ID and x,y,z are sent to OSC path /kinect/object.


Context:
==============

PCL 1.7, VTK 5.8, Qt Creator, Ubuntu 12.04

