#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "line_detection.cpp"

using namespace cv;
using namespace std;


int thresh = 100;
int max_thresh = (thresh*3);

Mat thresh_callback( int, void*, Mat);
Mat canny_detect( int, void*, Mat);
vector<vector< Point >> contour_detect(Mat);
vector<vector< Point >> approx_straight_contours(vector<vector<Point>>, double, double);
double calculate_r_value(vector<Point>);
double calculate_r_slope(vector<Point>, double);
double calculate_y_int(vector<Point>, double);
double calculate_simple_slope(Point, Point);
double calculate_simple_y_int(Point, double);
vector<double> make_line(Point, Point);
Point intersection(vector<double>, vector<double>);
Point closest_point_on_line(vector<double>, Point, double slope);
void insertion_sort(vector<double>&, vector<vector<Point>>&);
double computeDistance(Point, Point);
bool checkOverlapping(vector<Point>, vector<Point>);
double calculate_angle_between_lines_deg(vector<double>, vector<double>);
double calculate_angle_between_lines_rad(vector<double>, vector<double>);
Point lower_point_of_line_seg(vector<Point>);
Point higher_point_of_line_seg(vector<Point>);
bool checkCollinear(vector<Point>, vector<Point>);

int main( int, char** argv ){
    
    // Canny-detected edges copied to dst for imshow
    // not necessary will comment corresponding block later
    Mat dst;

    // SOURCE IMAGE
    // Load source image into src
    Mat src;
    double width = src.cols;
    double height = src.rows;
    
    src = imread("/Users/andrewho/Desktop/ARCL/polygon_detection/polygon_detection/static/images/test2.png", CV_LOAD_IMAGE_COLOR);
    
    // SOURCE
    // Show src image
//    const char* src_window = "Source";
//    namedWindow( src_window, WINDOW_AUTOSIZE );
//    imshow( src_window, src );
    //cv::createTrackbar( "Canny thresh:", "Original", &thresh, max_thresh, canny_detect );

    
    
    // IMAGE PREPARATION
    Mat src_gray; // Holds Grayed-out source image
    // 1. Convert to grey scale
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    // 2. Blur image, write to original image
    GaussianBlur( src_gray, src_gray, Size(3,3), 0, 0 );
//    const char* blurred_window = "Blurred";
//    namedWindow(blurred_window, WINDOW_AUTOSIZE);
//    imshow(blurred_window, src_gray);

    
    
    // THRESHOLDING
    Mat threshold_output; // Holds result of thresholding
    // 1. Call thresholding
    threshold_output = thresh_callback( 0, 0, src_gray);
    GaussianBlur( threshold_output, threshold_output, Size(3,3), 0, 0 );
//    GaussianBlur( threshold_output, threshold_output, Size(3,3), 0, 0 );
//    GaussianBlur( threshold_output, threshold_output, Size(3,3), 0, 0 );
    // Show results
//    const char* threshold_window = "Thresholding";
//    namedWindow( threshold_window, WINDOW_AUTOSIZE );
//    imshow( threshold_window, threshold_output);
    
    
    
    // CANNY EDGE DETECTION
    Mat canny_output; // Holds canny edges
    // 1. Call canny edge detectionx
    canny_output = canny_detect( 0, 0, threshold_output);
    // Show results
//    const char* canny_window = "Canny Edges";
//    namedWindow( canny_window, WINDOW_AUTOSIZE );
//    imshow( canny_window, canny_output);

    
    
    //CONTOUR DETECTION
    vector<vector<Point> > contours; // Holds contours
    vector<Vec4i> hierarchy; // Holds hierarchy of contours
    //1. Call contour detection
    contours = contour_detect(canny_output);
//    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//    for( int i = 0; i< contours.size(); i++ ) {
//        Scalar color = Scalar( 0, 255, 0 );
//        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//    }
//
//    namedWindow( "Contours", WINDOW_AUTOSIZE );
//    imshow( "Contours", drawing );

    
    
    // STRAIGHT CONTOUR APPROXIMATION
    vector<vector<Point> > uncorrected_straight_contours; // Holds equation of straight lines
    Scalar color = Scalar(0, 255, 0);
    
    //1. Call straight contour approximation
    uncorrected_straight_contours = approx_straight_contours(contours, width, height);
    
    vector<vector<Point> > straight_contours;
    
    for(int i = 0; i < uncorrected_straight_contours.size(); i++){
        if(computeDistance(uncorrected_straight_contours[i][0], uncorrected_straight_contours[i][1]) < 500){
            straight_contours.push_back(uncorrected_straight_contours[i]);
        }
    }
    
    Mat straight_contours_drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

    vector<double> slopes;

    for(int i = 0; i < straight_contours.size(); ++i){
        
        vector<Point> temp;
        
        Point p1 = straight_contours[i][0];
        Point p2 = straight_contours[i][1];
        
        double temp_slope = calculate_simple_slope(p1, p2);
 
        slopes.push_back(temp_slope);
    }

    
    insertion_sort(slopes, straight_contours);

//    int counter = 0;
//    while(counter < slopes.size() - 1){
//
//        if(abs(slopes[counter] - slopes[counter+1]) < 0.5 ||
//           abs(slopes[counter]) > 0.80 * abs(slopes[counter + 1]) ||
//           abs(slopes[counter + 1]) > 0.80 * abs(slopes[counter])
//           ){
//            if(checkOverlapping(straight_contours[counter], straight_contours[counter+1])){
//                double sc1_len = computeDistance(straight_contours[counter][0], straight_contours[counter][1]);
//                double sc2_len = computeDistance(straight_contours[counter+1][0], straight_contours[counter+1][1]);
//
//                if(sc1_len > sc2_len){
//                    slopes.erase(slopes.begin() + counter + 1);
//                    straight_contours.erase(straight_contours.begin() + counter + 1);
//                }
//                else{
//                    slopes.erase(slopes.begin() + counter);
//                    straight_contours.erase(straight_contours.begin() + counter);
//                }
//            }
//            else{
//                counter = counter + 1;
//            }
//        }
//        else{
//            counter = counter + 1;
//        }
//    }

    int counter = 0;
    while(counter < straight_contours.size() - 1){
        
        vector<double> line_1 = make_line(straight_contours[counter][0], straight_contours[counter][1]);
        vector<double> line_2 = make_line(straight_contours[counter+1][0], straight_contours[counter+1][1]);
        
        if(calculate_angle_between_lines_deg(line_1, line_2) < 10   ){
            if(checkOverlapping(straight_contours[counter], straight_contours[counter+1])){
                
                double sc1_len = computeDistance(straight_contours[counter][0], straight_contours[counter][1]);
                double sc2_len = computeDistance(straight_contours[counter+1][0], straight_contours[counter+1][1]);
                
                if(sc1_len > sc2_len){
                    straight_contours.erase(straight_contours.begin() + counter + 1);
                }
                else{
                    straight_contours.erase(straight_contours.begin() + counter);
                }
            }
            else{
                counter = counter + 1;
            }
        }
        else{
            counter = counter + 1;
        }
    }
    
    
    Mat filtered_straight_contours_drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    
    for(int i = 0; i < straight_contours.size(); ++i){
        
        Point p1 = straight_contours[i][0];
        Point p2 = straight_contours[i][1];
        
        line(filtered_straight_contours_drawing, p1, p2, color, 1, 8, 0);
        
    }
    
    namedWindow( "Filtered Blank Test", WINDOW_AUTOSIZE );
    imshow( "Filtered Blank Test", filtered_straight_contours_drawing );
    
    // Find collinear lines
    
//    counter = 0;
//    while(counter < straight_contours.size() - 1){
//
//        vector<double> line_1 = make_line(straight_contours[counter][0], straight_contours[counter][1]);
//        vector<double> line_2 = make_line(straight_contours[counter+1][0], straight_contours[counter+1][1]);
//
//        if(checkCollinear(line_1, line_2)){
//            double distance_1 = computeDistance(straight_contours[counter][0], straight_contours[counter+1][0]);
//            double distance_2 = computeDistance(straight_contours[counter][1], straight_contours[counter+1][1]);
//
//            double distance_3 = computeDistance(straight_contours[counter][1], straight_contours[counter+1][0]);
//            double distance_4 = computeDistance(straight_contours[counter][0], straight_contours[counter+1][1]);
//
//            double max_distance = min( distance_1, distance_2, distance_3, distance_4 );
//
//            if(max_distance == distance_1){
//
//            }
//            else if(max_distance == distance_2){
//
//            }
//            else if(max_distance == distance_3){
//
//            }
//            else{
//
//            }
//
//        }
//        else{
//            counter = counter + 1;
//        }
//    }
    
    
    waitKey(0);
    return(0);
}
Mat thresh_callback(int, void*, Mat src_gray){
    int threshold_value = 150; // The thresh value with respect to which the thresholding operation is made
    int threshold_type = 3; // One of the 5 params below. We use 3 for best results
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
     */
    int const max_binary_value = 255; // The value used with the Binary thresholding operations (to set the chosen pixels)

    Mat threshold_output; // return value

    threshold( src_gray, threshold_output, threshold_value, max_binary_value, threshold_type );

    return threshold_output;
}

Mat canny_detect(int, void*, Mat threshold_output) {
    Mat canny_output;
    /// Canny edge detection, output to canny_output
    /// Check threshold number, max
    Canny( threshold_output, canny_output, thresh, thresh*3, 3 );

    return canny_output;

}


vector<vector<Point> > contour_detect(Mat canny_output) {
    vector<vector<Point> > contours;
    vector<vector<Point> > ret_contours;
    vector<Vec4i> hierarchy;
    // Find all contours
    // Input:
    // Canny_output; preprepared and stored in canny_output
    // Output:
    // contours; stored in a vector<vector<Point> >
    // hierarchy; used to filter out overlapping contours
    // Params:
    // CV_RETR_EXTERNAL;
    // CHAIN_APPROX_SIMPLE: Reduces number of unnecessary points
    
    findContours( canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    return contours;

}

vector<vector<Point> > approx_straight_contours(vector<vector<Point>> contours, double width, double height){
    vector<vector<Point> > straight_contours;
    vector<Point> queue;
    vector<int> checked;
    double r_val;

    for(int i = 0; i < contours.size(); ++i){
        queue.clear();

        for(int j = 0; j < contours[i].size(); j += 2){
            // If first time or right after discovering line
            if(queue.size() == 0 && j < queue.size()-6){
                r_val = 0;
                for(int c = 0; c < 6; ++c){
                    queue.push_back(contours[i][j+c]);
                }
                j = j + 6;
            }

            // Otherwise add two more points and cycle out two points
            else{
                queue.push_back(contours[i][j]);
                queue.push_back(contours[i][j+1]);

                queue.erase(queue.begin());
                queue.erase(queue.begin());
            }

            r_val = calculate_r_value(queue);

            if (abs(r_val) > 0.9 && j + 8 < contours[i].size()-8){
                // Add two more points

                queue.push_back(contours[i][j+6]);
                queue.push_back(contours[i][j+7]);

                double new_r_val = calculate_r_value(queue);

                // If the r_value continues to grow,
                // then we begin checking a lot of points
                // because we know that a line exists

                if(abs(new_r_val) > abs(r_val)){

                    r_val = new_r_val;
                    bool cont = true;
                    int counter = 8;
                    // At this point, there should be 8 points
                    // in the queue. From 8, we add on 2 points
                    // at a time, checking that the r-value
                    // continues to grow.

                    while(cont){

                        // Case: If we can't add anymore points because we're
                        // at the end of the contour, then we just add the line
                        if(j+counter > contours[i].size() - 2){

                            vector<Point> straight; //Straight holds the first and last point of the line.

                            straight.push_back(queue[0]);
                            straight.push_back(queue[queue.size()-1]);

                            straight_contours.push_back(straight);

                            queue.clear();

                            j = j + counter - 1;
                            cont = false;
                        }
                        // if not at the end of the contour
                        else{
                            // Push back two points to check

                            queue.push_back(contours[i][j+counter]);
                            queue.push_back(contours[i][j+counter+1]);

                            new_r_val = calculate_r_value(queue);
                            // If new_r_val decreases, then we are adding points not
                            // on the line. So we take the end points as our
                            // new line
                            if(abs(new_r_val) < abs(r_val) ){
                                // Erase points that lowered correlation coefficient
                                queue.erase(queue.begin() + queue.size() - 1);

                                new_r_val = calculate_r_value(queue);

                                if(abs(new_r_val) < abs(r_val)){
                                    queue.erase(queue.begin() + queue.size() - 1);
                                }

                                // straight holds the end points of the potential line
                                vector<Point> straight;

                                // values to be used for comparison
                                straight.push_back(queue[0]);
                                straight.push_back(queue[queue.size()-1]);

                                straight_contours.push_back(straight);

                                queue.clear();

                                j = j + counter - 1;
                                cont = false;
                            }
                            else{
                                r_val = new_r_val;
                                counter = counter + 2;
                            }
                        }
                    }
                }
                else{
                    j = j + 2;
                }
            }
        }
    }

    return straight_contours;
}

double calculate_r_value(vector<Point> queue){
    double r_val;
    
    double n = queue.size();
    double S_xy = 0;
    double S_x = 0;
    double S_y = 0;
    double S_x2 = 0;
    double S_y2 = 0;
    
    // Calculate values for calculation
    for(int c = 0; c < n; ++c){
        S_xy = S_xy + (queue[c].x)*(queue[c].y);
        S_x = S_x + queue[c].x;
        S_y = S_y + queue[c].y;
        S_x2 = S_x2 + (queue[c].x)*(queue[c].x);
        S_y2 = S_y2 + (queue[c].y)*(queue[c].y);
    }
    
    double numerator = ( ( n*(S_xy) ) - (S_x)*(S_y) );
    double denom = (n*(S_x2) - (S_x)*(S_x))*(n*(S_y2) - (S_y)*(S_y));
    
    r_val = numerator/(sqrt(denom));
    
    return r_val;
}

double calculate_r_slope(vector<Point> queue, double r_val){
    double r_slope;
    double Sum_x = 0;
    double Sum_y = 0;
    double Mean_x;
    double Mean_y;
    double Std_x= 0;
    double Std_y = 0;
    double n = queue.size();
    
    for(int i = 0; i < queue.size(); ++i){
        Sum_x = Sum_x + queue[i].x;
        Sum_y = Sum_y + queue[i].y;
    }
    
    Mean_x = Sum_x / n;
    Mean_y = Sum_y / n;
    
    for(int j = 0; j < queue.size(); ++j){
        Std_x = Std_x + (queue[j].x - Mean_x)*(queue[j].x - Mean_x);
        Std_y = Std_y + (queue[j].y - Mean_y)*(queue[j].y - Mean_y);
    }
    
    Std_x = sqrt(Std_x/n);
    Std_y = sqrt(Std_y/n);
    
    r_slope = r_val*(Std_y/Std_x);
    
    return r_slope;
}

double calculate_y_int(vector<Point> queue, double r_slope){
    double y_int;
    double Sum_x = 0;
    double Sum_y = 0;
    double Mean_x;
    double Mean_y;
    double n = queue.size();
    
    for(int i = 0; i < queue.size(); ++i){
        Sum_x = Sum_x + queue[i].x;
        Sum_y = Sum_y + queue[i].y;
    }
    
    Mean_x = Sum_x / n;
    Mean_y = Sum_y / n;
    
    y_int = Mean_y - r_slope * Mean_x;
    
    return y_int;
}

double calculate_simple_slope(Point p1, Point p2){
    double rise = p1.y - p2.y;
    double run = p1.x - p2.x;
    
    return rise/run;
}

double calculate_simple_y_int(Point p1, double simple_slope){
    double simple_y_int = p1.y - simple_slope * p1.x;
    
    return simple_y_int;
}

vector<double> make_line(Point p1, Point p2){
    // Using the standard line form:
    // Ax + By = C
    // line_coeff holds <A, B, C>
    vector<double> line_coeff;
    
    double A = (p1.y - p2.y);
    double B = (p2.x - p1.x);
    double C = (p1.x*p2.y - p2.x*p1.y);
    
    line_coeff.push_back(A);
    line_coeff.push_back(B);
    line_coeff.push_back(C);
    
    return line_coeff;
    
}

Point intersection(vector<double> line1, vector<double> line2){
    // Uses general Cramer's rule to solve for line equations
    double D  = line1[0] * line2[1] - line1[1] * line2[0];
    double Dx = line1[2] * line2[1] - line1[1] * line2[2];
    double Dy = line1[0] * line2[2] - line1[2] * line2[0];
    
    Point intersection;
    
    if(D != 0){
        intersection.x = Dx/D; //x coordinate of intersection
        intersection.y = Dy/D; //y coordinate of intersection
        return intersection;
    }
    else {
        intersection.x = 100;
        intersection.y = 100;
        return intersection;
    }
}

Point closest_point_on_line(vector<double> line1, Point p1, double slope){
    
    // We set a new point p2 far away enough that the slope will not be skewed
    // rounding errors with int type
    Point p2;
    p2.x = p1.x + 300;
    p2.y = p1.y + double(300*slope);
    
    vector<double> line2 = make_line(p1, p2);
    
    Point ret_point = intersection(line1, line2);
    
    return ret_point;
    
}
// Implementation of insertion sort
// Sort contours based on slope magnitude
// Since we expect the array sizes to be small, we implement Insertion Sort
// https://www.google.com/search?q=fastest+sort+for+small+arrays&oq=fastest&aqs=chrome.1.69i57j69i59l2j0l3.7778j1j7&sourceid=chrome&ie=UTF-8

void insertion_sort(vector<double> &slopes, vector<vector<Point>> &straight_contours){
    double slope_to_move;
    vector<Point> sc_to_move;
    
    int n = slopes.size();
    
    for (int i = 1; i < n; i++) {
        slope_to_move = slopes[i];
        sc_to_move = straight_contours[i];
        
        for(int j = 0; j < i; j++){
            if(slope_to_move < slopes[j]){
                slopes.erase(slopes.begin() + i);
                slopes.insert(slopes.begin() + j, slope_to_move);
                
                straight_contours.erase(straight_contours.begin() + i);
                straight_contours.insert(straight_contours.begin() + j, sc_to_move);
                
                j = i;
            }
        }
    }
}


bool checkOverlapping(vector<Point> line_1, vector<Point> line_2){
    
    Point high_pt1(higher_point_of_line_seg(line_1));
    Point high_pt2(higher_point_of_line_seg(line_2));
    
    Point low_pt1(lower_point_of_line_seg(line_1));
    Point low_pt2(lower_point_of_line_seg(line_2));
    
    double distance_top = computeDistance(high_pt1, high_pt2);
    double distance_low = computeDistance(low_pt1, low_pt2);
    
    if(distance_top < 100 && distance_low < 100){
        return true;
    }
    else{
        return false;
    }
    
}

Point lower_point_of_line_seg(vector<Point> straight_contour){
    if(straight_contour[0].y < straight_contour[1].y){
        return straight_contour[0];
    }
    else{
        return straight_contour[1];
    }
    
}

Point higher_point_of_line_seg(vector<Point> straight_contour){
    if(straight_contour[0].y > straight_contour[1].y){
        return straight_contour[0];
    }
    else{
        return straight_contour[1];
    }
    
}

bool checkCollinear(vector<Point> line_1, vector<Point> line_2 ){

//    using formula P =4θs(g+I)(πI2)
    
    //    double theta = calculate_angle_between_lines_rad(line_1, line_2) ;
    //
    //    double s =
    //
    //    double P_CO = (4 * theta * s * (g + I))/I_2   4θs(g+I)/(πI2)

    
//    double len_line1 =
//    double len_line2 =

// using simplified criteria
    
    Point l1_lowest = lower_point_of_line_seg(line_1);
    Point l1_highest = higher_point_of_line_seg(line_1);
    
    Point l2_lowest = lower_point_of_line_seg(line_2);
    Point l2_highest = higher_point_of_line_seg(line_2);
    
    double distance_1 = computeDistance(l1_lowest, l2_highest);
    double distance_2 = computeDistance(l1_highest, l2_lowest);

    double min_dist = min(distance_1, distance_2);
    
    if(min_dist < 200){
        return true;
    }
    else{
        return false;
    }
}

// Helper function to compute distance between two points
double computeDistance(Point p1, Point p2){
    double delta_x = p1.x - p2.x;
    double delta_y = p1.y - p2.y;
    
    double ret_val = sqrt(delta_x*delta_x + delta_y*delta_y);
    
    return ret_val;
}

double calculate_angle_between_lines_deg(vector<double> line_1, vector<double> line_2){
    double slope_1 = -1 * (line_1[0] / line_1[1]);
    
    double slope_2 = -1 * (line_2[0] / line_2[1]);
    
    double rad_angle = atan( (slope_1 - slope_2)/(1 + slope_1*slope_2) );
    
    double deg_angle = rad_angle * 180/3.1415926535;
    
    return deg_angle;

}
      
double calculate_angle_between_lines_rad(vector<double> line_1, vector<double> line_2){
    double slope_1 = -1 * (line_1[0] / line_1[1]);
          
    double slope_2 = -1 * (line_2[0] / line_2[1]);
          
    double rad_angle = atan( (slope_1 - slope_2)/(1 + slope_1*slope_2) );
          
    return rad_angle;
          
}
