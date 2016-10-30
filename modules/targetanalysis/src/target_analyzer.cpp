/**
 * @file target_analyzer.cpp
 * @author WARG
 *
 * @section LICENSE
 *
 *  Copyright (c) 2015-2016, Waterloo Aerial Robotics Group (WARG)
 *  All rights reserved.
 *
 *  This software is licensed under a modified version of the BSD 3 clause license
 *  that should have been included with this software in a file called COPYING.txt
 *  Otherwise it is available at:
 *  https://raw.githubusercontent.com/UWARG/computer-vision/master/COPYING.txt
 */

/*TODO: First thing to do; 
1) Convert Pixel location into actual location; Pixel Area into actual area
2) Add classfiers for targets using:
Area, Centroid, Perimeter, Colour, Error, contour, edge magnitude
Evaluate each to classify as each target and apply percentage to 
3) 
*/

#include "frame.h"
#include "pixel_object.h"
#include "target_analyzer.h"
#include "target.h"

void analyze_pixel_object(PixelObject* po){
    //Generate GPS coordinates for each PixelObject
    Frame* f = po->get_image();
    Metadata* m = f->get_metadata();
    double longitude = m->lon;
    double latitude = m->lat;
    double altitude = m->altitude;
    getGPSCorners(<cameraalpha>,longitude,latitude,altitude,f->get_img(),po);
    getGPSCentroid();
    //Identify unique PixelObjects and combine non-unique objects - Compare with PixelObjectList
 
}

void generate_targets(PixelObjectList pol){ 
    //Iterate through list and create targets


    
    /*if (
    //Create new Target object if a duplicate has not been found
    Target t = new Target(); //TODO: Fill with appropriate constructor dataset

    //*/


}

//Based on the GPS location of the image, calculates the
//GPS location of a certain pixel in the image.
void getGPSCentroid(cv::Point2d point){

//Gets the GPS location of each corner of the image based on the center GPS
//coordinate acquired by the GPS
// cameraAlpha is in degrees
//TODO: The image data fed into this function should have the camera distortion
//corrected for.
//TODO: Add compensation for roll and pitch angles. This should skew the
//photo/gps grid. The lens profile may have a big effect here. See previous
//todo.
void getGPSCorners(cv::Point2d cameraAlpha, double longitude, double latitude,
double altitude, double heading, cv::Mat* img, PixelObject* po){
    //Note: The cameraAlpha value represents the half angle of the x and y//direction of the image.
    double cameraX = altitude * tan(cameraAlpha.x); //meters from center of photo to edge
    double cameraY = altitude * tan(cameraAlpha.y); //meters from center of photo to edge
    
    //Haversine formula - rearranged
    //Note: These are false coordinates, as they still need to be rotated (with
    //a rotation matrix)
    double cameraXEdge = acos(1 - (1 -
    cos(cameraX/EARTH_RADIUS))/(cos(DEG2RAD(latitude))*cos(DEG2RAD(latitude)))) +
    longitude;
    double cameraYEdge = cameraY/EARTH_RADIUS + latitude;


    //Rotation Matrix - Heading
    //Note: The '-heading' compensates for the fact that directional heading is
    //a clockwise quantity, but cos(theta) assumes theta is a counterclockwise
    //quantity.
    double realX = cos(DEG2RAD(-heading)) * cameraXEdge - sin(DEG2RAD(-heading)) *
    cameraYEdge;
    double realY = sin(DEG2RAD(-heading)) * cameraXEdge + cos(DEG2RAD(-heading)) *
    cameraYEdge;

    double unitX = realX/img->cols;
    double unitY = realY/img->rows;

    po->set_pixel_distance(unitX,unitY);


    //For future thought: THIS IS THE LEAST RELIABLE PART, WHAT IS THE ACTUAL
    //HEADING??? We use GPS Heading, but the wind can alter this by upto
    //30degreesC
    //This code assumes the top of the image is also pointing towards the heading
    //measured by the aircraft
}

void analyzeTargetDuplicateProbability(){
    //Run functions to determine probability of the image displaying a duplicate based on visual, telemetry data.

    //*TargetList*
    //-Feature Matching
    //-GPS coordinate matching
    //-Area Matching
    //-Perimeter Matching
    //-Contour Matching
    //-Colour Matching
    //-Consider expected error


}
