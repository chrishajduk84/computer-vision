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
#include "pixel_object_list.h"
#include "object_list.h"
#include "target_analyzer.h"
#include "target.h"

#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/core.hpp>

using namespace std;

TargetAnalyzer* TargetAnalyzer::analyzer = NULL;
vector<Object*> mostRecentObjectList;

void TargetAnalyzer::analyze_pixelobject(PixelObject* po){
    
    //K I messed up, we still need a pixelObjectList. It still needs to sort
    //through pixelobjects and group them, but not based on GPS coordinates, but
    //relative dimensions, scale, colours, contour, area, and photo similarity
    //Essentially, comapre and add into pixelObjectList
    PixelObjectList* pol = PixelObjectList::getInstance();
    pol->addAndCompare(po);
    BOOST_LOG_TRIVIAL(debug) << "End Analysis";

}


/* void SOMEFUNCTIONHERE - Needs to run once in a while to extract objects to
 * pixelObjects*/
vector<Object*> TargetAnalyzer::extract_objects(){
    PixelObjectList* pol = PixelObjectList::getInstance();
    pol->getObjects(&mostRecentObjectList);
    return mostRecentObjectList;
}

int TargetAnalyzer::get_unique_objects_length(){
    PixelObjectList* pol = PixelObjectList::getInstance();
    return pol->getListLength();
}
/*
void analyze_object(Object* o){
    
    //Generate GPS coordinates for each PixelObject
    std::vector<PixelObject*> objects = o->get_pobjects();
    Frame* f = objects.at(0)->get_image();
    const Metadata* m = f->get_metadata();
    double longitude = m->lon;
    double latitude = m->lat;
    double altitude = m->altitude;
    cv::Point2d cameraalpha(45,45);
    getGPSCorners(cameraalpha,longitude,latitude,altitude,o->get_image(),o);
    //getGPSCentroid(); //Do I need this? or is 4 corners enough? Or maybe use
    //this instead of the 4 corners
    
    //Identify unique Objects and combine non-unique objects - Compare with ObjectList
    ObjectList::addAndCompare(o)

    //Extract unique objects and reformat into targets
    Target* t = new Target();
    t->add_object(<INSERTOHERE>); 

    //Return
}
*/

/* WILL DECIDE THIS LATER, BUT NEED A WAY TO STORE TARGETS FOR REPORT GENERATION
void generate_target_list(ObjectList ol){ 
    //Iterate through list and create targets
    for (int i = 0; i < ol.length(); i++){
        Object* t = pol.pop(); //Pop an array - it contains all duplicates
    }

    //Place in TargetList
    //TODO: Make TargetList class

    if (
    //Create new Target object if a duplicate has not been found
    Target t = new Target(); //TODO: Fill with appropriate constructor dataset
   


}*/

//Based on the GPS location of the image, calculates the
//GPS location of a certain pixel in the image.
void TargetAnalyzer::getGPSCentroid(cv::Point2d point){

//Gets the GPS location of each corner of the image based on the center GPS
//coordinate acquired by the GPS
// cameraAlpha is in degrees
//TODO: The image data fed into this function should have the camera distortion
//corrected for.
//TODO: Add compensation for roll and pitch angles. This should skew the
//photo/gps grid. The lens profile may have a big effect here. See previous
//todo.

}

void TargetAnalyzer::getGPSCorners(cv::Point2d cameraAlpha, double longitude, double latitude,
double altitude, double heading, cv::Mat* img, Object* o){
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

    //o->set_pixel_distance(unitX,unitY);  UNDEFINED FOR THE TIME BEING - ADD TO Object.cpp


    //For future thought: THIS IS THE LEAST RELIABLE PART, WHAT IS THE ACTUAL
    //HEADING??? We use GPS Heading, but the wind can alter this by upto
    //30degreesC
    //This code assumes the top of the image is also pointing towards the heading
    //measured by the aircraft
}

void TargetAnalyzer::analyzeTargetDuplicateProbability(){
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
