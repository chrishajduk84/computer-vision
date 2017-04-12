/**
 * @file target_analyzer.cpp
 * @author WARG
 *
 * @section LICENSE
 *
 *  Copyright (c) 2015-2017, Waterloo Aerial Robotics Group (WARG)
 *  All rights reserved.
 *
 *  This software is licensed under a modified version of the BSD 3 clause license
 *  that should have been included with this software in a file called COPYING.txt
 *  Otherwise it is available at:
 *  https://raw.githubusercontent.com/UWARG/computer-vision/master/COPYING.txt
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
    
    PixelObjectList* pol = PixelObjectList::getInstance();
    pol->addAndCompare(po);
    BOOST_LOG_TRIVIAL(trace) << "End of Analysis";

}


vector<Object*> TargetAnalyzer::extract_objects(){
    PixelObjectList* pol = PixelObjectList::getInstance();
    pol->getObjects(&mostRecentObjectList);
    return mostRecentObjectList;
}

int TargetAnalyzer::get_unique_objects_length(){
    PixelObjectList* pol = PixelObjectList::getInstance();
    return pol->getListLength();
}

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

double TargetAnalyzer::getGPSDistance(double lat1, double lon1, double lat2, double lon2){
    double dLat = DEG2RAD(lat2 - lat1);
    double dLon = DEG2RAD(lon2 - lon1);

    float a = sin(dLat / 2) * sin(dLat / 2) + cos(DEG2RAD(lat1)) * cos(DEG2RAD(lat2)) * sin(dLon / 2) * sin(dLon / 2);

    /*if ((dLat >= 0 && dLon >=0)||(dLat < 0 && dLon < 0)){*/
        return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a)));
    /*}*/
    /*else {
        return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a))) * -1;
    }*/

}

double TargetAnalyzer::getGPSDistance(cv::Point2d gps1, cv::Point2d gps2){
    return getGPSDistance(gps1.x, gps1.y, gps2.x, gps2.y);
}
   //For future thought: GPS GEOLOCATION IS THE LEAST RELIABLE PART, WHAT IS THE ACTUAL
    //HEADING??? We use GPS Heading, but the wind can alter this by upto
    //30degreesC
    //This code assumes the top of the image is also pointing towards the heading
    //measured by the aircraft

   //Run functions to determine probability of the image displaying a duplicate based on visual, telemetry data.
    //Things to consider for the future:
    //*TargetList*
    //-Feature Matching
    //-GPS coordinate matching
    //-Area Matching
    //-Perimeter Matching
    //-Contour Matching
    //-Colour Matching
    //-Consider expected error

