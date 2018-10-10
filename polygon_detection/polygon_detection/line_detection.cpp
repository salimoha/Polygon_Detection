/*
 line_detection.cpp
 polygon_detection
 
 Created by Andrew Ho on 6/24/18.
 Copyright © 2018 Andrew Ho, California Institute of Technology. All rights reserved.
 */

#include "line_detection.hpp"

Mat otsu_threshold(Mat src_gray, int input_operation){
    Mat threshold_output;
    
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
     */
    int operation = input_operation;
    
    threshold(src_gray,
              threshold_output,
              0,
              255,
              operation | CV_THRESH_OTSU);
    
    return threshold_output;
}

Mat adaptive_thresh(Mat src_gray, int input_blocksize, int input_cval, int input_operation){
    int blocksize = input_blocksize;
    int c_val = input_cval;
    
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
     */
    int operation = input_operation;
    
    Mat threshold_output;
    
    adaptiveThreshold(src_gray,
                      threshold_output,
                      255,
                      ADAPTIVE_THRESH_GAUSSIAN_C,
                      operation,
                      blocksize,
                      c_val);
    
    return threshold_output;
}

Mat global_thresh(Mat src_gray, int thresh_val, int input_operation) {
    int threshold_value = thresh_val;
    int const max_binary_value = 255;

    
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
     */
    int operation = input_operation;
    
    Mat threshold_output;

    threshold(src_gray,
              threshold_output,
              threshold_value,
              max_binary_value,
              operation);
    
    return threshold_output;
}

float calcMagnitude(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

float dotproduct(vector<float> v1, vector<float> v2) {
    return (v1[0]*v2[0]) + (v1[1]*v2[1]);
}

float angleBetween(Point p1, Point p2, Point p3) {
    vector<float> v1;
    v1.push_back(p1.x - p2.x);
    v1.push_back(p1.y - p2.y);

    vector<float> v2;
    v2.push_back(p3.x - p2.x);
    v2.push_back(p3.y - p2.y);

    float dot_product = dotproduct(v1, v2);

    float mag1 = calcMagnitude(p1, p2);
    float mag2 = calcMagnitude(p3, p2);
    return acos(dot_product/(mag1 * mag2));
}

vector<Point2f> goodFeaturesToTrack_Callback(Mat input, int numCorners ) {
    /// Parameters for Shi-Tomasi algorithm
    int maxCorners = numCorners;
    
    maxCorners = MAX(maxCorners, 1);
    vector<Point2f> corners;
    double qualityLevel = 0.3;
    double minDistance = 10;
    int blockSize = 3, gradientSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    
    /// Copy the source image
    Mat copy = input.clone();
    
    /// Apply corner detection
    goodFeaturesToTrack(input,
                        corners,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        gradientSize,
                        useHarrisDetector,
                        k );
    
    return corners;
}

tuple<float, vector<Point2f>> calcCost(vector<Point2f> corners, Point2f node){

    if(corners.size() == 1){
        vector<Point2f> new_path{corners[0]};
        tuple <float, vector<Point2f>> TSA_obj = make_tuple(calcMagnitude(node, corners[0]), new_path);
        return TSA_obj;
    }

    vector<float> costs;
    vector<vector<Point2f>> paths;
    for(int v_num = 1; v_num < corners.size(); ++v_num ){
        // Calculate distance
        Point2f frontier_node = corners[v_num];
        float dist = calcMagnitude(node, frontier_node);

        // Calculate cost
        vector<Point2f> corners_subset(corners);
        corners_subset.erase (corners_subset.begin() + v_num);
        tuple<float, vector<Point2f>> TSA_obj = calcCost(corners_subset, frontier_node);

        // Store cost
        float cost = dist + get<0>(TSA_obj);
        costs.push_back(cost);

//         Store path
        vector<Point2f> temp_path(get<1>(TSA_obj));
        temp_path.push_back(frontier_node);
        paths.push_back(temp_path);
    }

    float minCostIndex = 0;
    for(int i = 0; i < costs.size(); ++i){
        if(i != 0){
            if(costs[i] < costs[minCostIndex]){
                minCostIndex = i;
            }
        }
    }

    tuple<float, vector<Point2f>> ret_tuple = make_tuple(costs[minCostIndex], paths[minCostIndex]);

    return ret_tuple;
}

vector<Point2f> TSA(vector<Point2f> corners){
    corners.push_back(corners[0]);
    Point2f init_node = corners[0];
    tuple<float, vector<Point2f>> optimalTSA = calcCost(corners, init_node);
    
    vector<Point2f> ret_corners;
    for(int i = 1; i < get<1>(optimalTSA).size(); ++i){
        ret_corners.push_back(get<1>(optimalTSA)[i]);
    }
    
    return ret_corners;
}

vector<Point2f> removeCollinear(vector<Point2f> TSA_corners){
    vector<Point2f> non_collinear_corners;
    TSA_corners.push_back(TSA_corners[0]);
    TSA_corners.push_back(TSA_corners[1]);
    for(int i = 1; i < TSA_corners.size()-1; ++i){
        float angleBetweenSegments = (angleBetween(TSA_corners[i-1],
                                                   TSA_corners[i],
                                                   TSA_corners[i+1]))*(180/3.1415926535);
        if (angleBetweenSegments < 160.0){
            non_collinear_corners.push_back(TSA_corners[i]);
        }
    }
    
    return non_collinear_corners;
    
}

vector<Point2f> removeFalseCorners(vector<Point2f> non_collinear_corners, Mat Hough_output){
    vector<Point2f> significant_corners;
    
    non_collinear_corners.push_back(non_collinear_corners[0]);
    non_collinear_corners.push_back(non_collinear_corners[1]);
    for(int i = 1; i < non_collinear_corners.size()-1; ++i){
        Point2f p1(non_collinear_corners[i-1]);
        Point2f p2(non_collinear_corners[i]);
        Point2f p3(non_collinear_corners[i+1]);
        
        float D1 = calcMagnitude(p1, p2);
        float D2 = calcMagnitude(p2, p3);
        
        float param_t = 0.1;
        int first_WPix_count = 0;
        while(param_t < 1){
            
            float test_x = p1.x + (param_t)*(p2.x - p1.x);
            float test_y = p1.y + (param_t)*(p2.y - p1.y);
            
            Point test_point(test_x, test_y);
            
            int pix=(int)Hough_output.at<uchar>(test_y, test_x);
            if(pix != 0){
                ++first_WPix_count;
            }
            param_t = param_t + 0.1;
        }
        
        int second_WPix_count = 0;
        param_t = 0.1;
        while(param_t < 1){
            float test_x = p2.x + (param_t)*(p3.x - p2.x);
            float test_y = p2.y + (param_t)*(p3.y - p2.y);
            
            Point test_point(test_x, test_y);
            
            int pix=(int)Hough_output.at<uchar>(test_y, test_x);
            if(pix != 0){
                ++second_WPix_count;
            }
            param_t = param_t + 0.1;
        }
        if(first_WPix_count > 5 || second_WPix_count > 5){
            significant_corners.push_back(non_collinear_corners[i]);
        }
    }
    
    return significant_corners;
}

void drawPoints(Mat* canvas, vector<Point2f> corners, int radius){
    for( size_t i = 0; i < corners.size(); i++ ) {
        circle(*canvas, corners[i], radius, Scalar(255, 0, 0), FILLED );
    }
}


void drawPointsAndEdges(Mat* canvas, vector<Point2f> corners, int radius, int thickness){
    for( size_t i = 0; i < corners.size(); i++ ) {
        circle(*canvas, corners[i], radius, Scalar(255, 0, 0), FILLED );
    }
    
    for( size_t i = 0; i < corners.size()-1; i++ ) {
        line(*canvas, corners[i], corners[i+1], Scalar(0, 255, 0), thickness);
    }
    line(*canvas, corners[0], corners[corners.size()-1], Scalar(0, 255, 0), thickness, CV_AA);
}

double getOtsuVal( const cv::Mat& _src ) {
    cv::Size size = _src.size();
    if ( _src.isContinuous() )
    {
        size.width *= size.height;
        size.height = 1;
    }
    const int N = 256;
    int i, j, h[N] = {0};
    for ( i = 0; i < size.height; i++ )
    {
        const uchar* src = _src.data + _src.step*i;
        for ( j = 0; j <= size.width - 4; j += 4 )
        {
            int v0 = src[j], v1 = src[j+1];
            h[v0]++; h[v1]++;
            v0 = src[j+2]; v1 = src[j+3];
            h[v0]++; h[v1]++;
        }
        for ( ; j < size.width; j++ )
            h[src[j]]++;
    }
    
    double mu = 0, scale = 1./(size.width*size.height);
    for ( i = 0; i < N; i++ )
        mu += i*h[i];
    
    mu *= scale;
    double mu1 = 0, q1 = 0;
    double max_sigma = 0, max_val = 0;
    
    for ( i = 0; i < N; i++ )
    {
        double p_i, q2, mu2, sigma;
        
        p_i = h[i]*scale;
        mu1 *= q1;
        q1 += p_i;
        q2 = 1. - q1;
        
        if ( std::min(q1,q2) < FLT_EPSILON || std::max(q1,q2) > 1. - FLT_EPSILON )
            continue;
        
        mu1 = (mu1 + i*p_i)/q1;
        mu2 = (mu - q1*mu1)/q2;
        sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
        if ( sigma > max_sigma )
        {
            max_sigma = sigma;
            max_val = i;
        }
    }
    
    return max_val;
}

Mat colorReduction(Mat input, int num_colors) {
    Mat data;
    input.convertTo(data, CV_32F);
    data = data.reshape(1,data.total());
    
    // do kmeans
    Mat labels, centers;
    kmeans(data, num_colors, labels, TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3,
           KMEANS_PP_CENTERS, centers);
    
    // reshape both to a single row of Vec3f pixels:
    centers = centers.reshape(3,centers.rows);
    data = data.reshape(3,data.rows);
    
    // replace pixel values with their center value:
    Vec3f *p = data.ptr<Vec3f>();
    for (size_t i=0; i<data.rows; i++) {
        int center_id = labels.at<int>(i);
        p[i] = centers.at<Vec3f>(center_id);
    }
    
    // back to 2d, and uchar:
    input = data.reshape(3, input.rows);
    input.convertTo(input, CV_8U);
    
    return input;
}

void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);
    
    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);
            
            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
            (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
            (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
            (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);
            
            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }
    
    im &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinning(cv::Mat& im)
{
    im /= 255;
    
    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;
    
    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    }
    while (cv::countNonZero(diff) > 0);
    
    im *= 255;
}
