/*
 main.cpp
 polygon_detection
 
 Created by Andrew Ho on 6/24/18.
 Copyright © 2018 Andrew Ho, California Institute of Technology. All rights reserved.
 */

#include "line_detection.hpp"

int main(int argc, char* argv[]) {
    String sequence_name = "tests";
    String sequencepath = "/Users/andrewho/repos/Polygon_Detection/polygon_detection/polygon_detection/static/" + sequence_name + "/*.png";
    
    String outputpath = "/Users/andrewho/repos/Polygon_Detection/polygon_detection/polygon_detection/video_output/"+ sequence_name + "/";
    vector<String> filenames;
    cv::glob(sequencepath, filenames);
    
    int limiter = 200;
    
    if(filenames.size() < limiter){
        limiter = filenames.size();
    }
    
    cout << sequence_name << endl;
    
    for (size_t image_num = 0; image_num<limiter; image_num++){
        cout << "Image " << image_num << endl;
        // SOURCE IMAGE
        // Load source image into src
        Mat src = imread(filenames[image_num]);
        
        string src_filename = outputpath + to_string(image_num) + "_src.png";
        imwrite(src_filename, src);
        
        
//      IMAGE PREPARATION
        
        // Remove text at bottom
//        cv::Rect init_ROI(0, 100, 1200, 550);
//        cv::Mat cropped_src = src(init_ROI);
//        Mat src_gray;
//        cvtColor( cropped_src, src_gray, CV_BGR2GRAY);
//        GaussianBlur( src_gray, src_gray, Size(3,3), 0, 0 );

        Mat src_gray;
        cvtColor(src, src_gray, CV_BGR2GRAY);
        GaussianBlur( src_gray, src_gray, Size(3,3), 0, 0 );
        
        
//      FIND ROI
        Mat ROI_temp;
        
        ROI_temp = adaptive_thresh(src_gray, 151, 12, 1);
        int leftBound = src_gray.cols;
        int rightBound = 0;
        bool topFlag = false;
        int topBound = src_gray.rows;
        int botBound = 0;
        
        for(int height_iter = 0; height_iter < src_gray.rows; ++height_iter){
            for(int width_iter = 0; width_iter < src_gray.cols; ++width_iter){
                int pix=(int)ROI_temp.at<uchar>(height_iter, width_iter);
                
                if(pix != 0){
                    if(!topFlag){
                        topFlag = true;
                        topBound = height_iter;
                    }
                    else{
                        if(height_iter > botBound){
                            botBound = height_iter;
                        }
                    }
                    
                    if(width_iter < leftBound){
                        leftBound = width_iter;
                    }
                    else if (width_iter > rightBound){
                        rightBound = width_iter;
                    }
                }
            }
        }
        
        Rect ROI(leftBound, topBound, rightBound - leftBound, botBound - topBound);
        Mat focused_src = src(ROI);
        
        int border=200;
        Mat padded_src(focused_src.rows + border*2, focused_src.cols + border*2, focused_src.depth());

        copyMakeBorder(focused_src, padded_src, border, border,
                       border, border, BORDER_REPLICATE);
        
        cv::resize(padded_src, padded_src, cv::Size(), 2, 2);
        
        string padded_filename = outputpath + to_string(image_num) + "_padded.png";
        imwrite(padded_filename, padded_src);
        
// COLOR REDUCTION
        Mat reduced_src = colorReduction(padded_src, 2);
        
        string reduced_filename = outputpath + to_string(image_num) + "_reduced.png";
        imwrite(reduced_filename, reduced_src);
       
//
//
//        string canvas1_filename = outputpath + to_string(image_num) + "_canvas1.png";
//        imwrite(canvas1_filename, canvas_1);
//        string canvas2_filename = outputpath + to_string(image_num) + "_canvas2.png";
//        imwrite(canvas2_filename, canvas_2);
//        string canvas3_filename = outputpath + to_string(image_num) + "_canvas3.png";
//        imwrite(canvas3_filename, canvas_3);
        
        
//        string reduced_filename = outputpath + to_string(image_num) + "_reduced.png";
//        imwrite(reduced_filename, reduced_src);
        

//      THRESHOLD
        int otsuVal = getOtsuVal(reduced_src);
        otsuVal = otsuVal + 0.5*(255 - otsuVal);

        Mat threshold_output;
        threshold(reduced_src, threshold_output, otsuVal, 255, 0);

        cvtColor(threshold_output, threshold_output, CV_BGR2GRAY);

//        erode(threshold_output, threshold_output, Mat());
        
//        thinning(threshold_output);
//        dilate(threshold_output, threshold_output, Mat());
        
        string threshold_filename = outputpath + to_string(image_num) + "_threshold.png";
        imwrite(threshold_filename, threshold_output);

//        Mat canny_output;
//        Canny(threshold_output, canny_output, otsuVal, 255, 3, true);
//
//        string canny_filename = outputpath + to_string(image_num) + "_canny.png";
//        imwrite(canny_filename, canny_output);

        

        
//      LINE SEGMENT DETECTION
//        cv::Ptr<cv::LineSegmentDetector> det;
//        det = cv::createLineSegmentDetector();
//        vector<Vec4f> lines_4f;
//
//        det->detect(threshold_output, lines_4f);
//
//        Mat LSD_output = Mat::zeros(padded_src.size(), CV_8UC3 );
//        det->drawSegments(LSD_output, lines_4f);
//        cvtColor(LSD_output, LSD_output, CV_BGR2GRAY);
//
//        string LSD_filename = outputpath + to_string(image_num) + "_LSD.png";
//        imwrite(LSD_filename, LSD_output);

// PROBABILISTIC HOUGH LINE TRANSFORM
//        Mat Hough_output = Mat::zeros(reduced_src.size(), CV_8UC3 );
//
//        vector<Vec4f> Hough_lines;
//        HoughLinesP(canny_output, Hough_lines, 1, CV_PI/180, 50, 50, 10 );
//
//        for( size_t i = 0; i < Hough_lines.size(); i++ ) {
//            Vec4i l = Hough_lines[i];
//            line(Hough_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 1, CV_AA);
//        }
//
//        string Hough_filename = outputpath + to_string(image_num) + "_Hough.png";
//        imwrite(Hough_filename, Hough_output);
        

//      CORNER DETECTION
    //        vector<Point2f> detected_corners = goodFeaturesToTrack_Callback(LSD_output, 8);
    //        Mat ShiTomasi_output = padded_src.clone();
    //        drawPoints(&ShiTomasi_output, detected_corners, 10);
    //
    //        string ShiTomasi_filename = outputpath + to_string(image_num) + "_ShiTomasi.png";
    //        imwrite(ShiTomasi_filename, ShiTomasi_output);
        
        
        
//
//        TSA
//        vector<Point2f> TSA_corners = TSA(detected_corners);
//        Mat TSA_output = padded_src.clone();
//        drawPointsAndEdges(&TSA_output, TSA_corners, 4, 2);
//
//        string TSA_filename = outputpath + to_string(image_num) + "_TSA.png";
//        imwrite(TSA_filename, TSA_output);
        
//        // COLLINEAR FILTERING
//        vector<Point2f> non_collinear_corners = removeCollinear(TSA_corners);
////        Mat nonCollinear_output = src.clone();
////        drawPointsAndEdges(&nonCollinear_output, non_collinear_corners, 4, 2);
//
//        // FALSE CORNER DETECTION
//        vector<Point2f> significant_corners = removeFalseCorners(non_collinear_corners, Hough_output);
////        Mat true_corners_output = cropped_src.clone();
//        Mat true_corners_output = src.clone();
//        if(significant_corners.size() !=0){
//            drawPointsAndEdges(&true_corners_output, significant_corners, 4, 2);
//        }
//        string image_name = "/Users/andrewho/repos/Polygon_Detection/polygon_detection/polygon_detection/video_output/tests/" + to_string(image_num) + ".png";
//        imwrite(image_name, true_corners_output);
//        string output = to_string(image_num) + ".png";
//        cout << output << " written" << endl;
    }
    waitKey();
 
    return 0;
}


