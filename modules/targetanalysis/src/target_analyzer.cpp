/* 
    This file is part of WARG's computer-vision

    Copyright (c) 2015-2016, Waterloo Aerial Robotics Group (WARG)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Usage of this code MUST be explicitly referenced to WARG and this code
       cannot be used in any competition against WARG.
    4. Neither the name of the WARG nor the names of its contributors may be used
       to endorse or promote products derived from this software without specific
       prior written permission.

    THIS SOFTWARE IS PROVIDED BY WARG ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL WARG BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

void analyze_targets_in_frame(Frame * f, PixelObject t){
    
}

//Based on the GPS location of the image (location of 4 corners), calculates the
//GPS location of a single point.
void getGPSCentroid(cv::Point2D point){

}

//Gets the GPS location of each corner of the image based on the center GPS
//coordinate acquired by the GPS
// cameraAlpha is in degrees
//TODO: The image data fed into this function should have the camera distortion
//corrected for.
//TODO: Add compensation for roll and pitch angles. This should skew the
//photo/gps grid. The lens profile may have a big effect here. See previous
//todo.
void getGPSCorners(cv::Point2d cameraAlpha, double longitude, double latitude,
double altitude, double heading, cv::Mat* img){
    //Note: The cameraAlpha value represents the half angle of the x and y
    //direction of the image.
    cameraX = altitude * tan(cameraAlpha.x); //meters from center of photo to edge
    cameraY = altitude * tan(cameraAlpha.y); //meters from center of photo to edge
    
    //Haversine formula - rearranged
    //Note: These are false coordinates, as they still need to be rotated (with
    //a rotation matrix)
    //TODO: Add macros
    cameraXEdge = acos(1 - (1 -
    cos(cameraX/EARTH_RADIUS))/(cos(DEG2RAD(latitude))*cos(DEG2RAD(latitude))) +
    longitude;
    cameraYEdge = cameraY/EARTH_RADIUS + latitude;


    //Rotation Matrix - Heading
    

    //Compensate heading - THIS IS THE LEAST RELIABLE PART, WHAT IS THE ACTUAL
    //HEADING??? We use GPS Heading, but the wind can alter this by upto
    //30degreesC
    //Note: Assuming the top of the image is also pointing towards the heading
    //measured by the aircraft
    
    realX = 
    realY = 

    
}
