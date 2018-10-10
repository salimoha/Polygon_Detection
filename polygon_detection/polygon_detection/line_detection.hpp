/*
  line_detection.hpp
  polygon_detection

  Created by Andrew Ho on 6/24/18.
  Copyright © 2018 Andrew Ho. All rights reserved.
*/

#ifndef line_detection_hpp
#define line_detection_hpp

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <functional>
#include <set>

#include <opencv2/line_descriptor.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"

using namespace cv;
using namespace std;

// Functional
Mat otsu_threshold(Mat, int);
Mat adaptive_thresh(Mat, int, int, int);
Mat global_thresh(Mat, int, int);
float calcMagnitude(Point, Point) ;
float dotproduct(vector<float>, vector<float>);
float angleBetween(Point, Point, Point);
vector<Point2f> goodFeaturesToTrack_Callback(Mat, int);
tuple<float, vector<Point2f>> calcCost(vector<Point2f>, Point2f);
vector<Point2f> TSA(vector<Point2f>);
vector<Point2f> removeCollinear(vector<Point2f>);
vector<Point2f> removeFalseCorners(vector<Point2f>, Mat);
double getOtsuVal( const cv::Mat& );
Mat colorReduction(Mat, int);
void thinning(Mat& im);
void thinningIteration(Mat& im, int);


// Auxiliary
void drawPoints(Mat* canvas, vector<Point2f> corners, int radius);
void drawPointsAndEdges(Mat* canvas, vector<Point2f> corners, int radius, int thickness);


struct Node{
    float x;
    float y;
    
    struct Node *next;
    
    Node(Point2f input_point){
        x = input_point.x;
        y = input_point.y;
    }
};



#endif /* line_detection_hpp */
