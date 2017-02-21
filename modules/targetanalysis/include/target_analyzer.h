/* 
    This file is part of WARG's computer-vision

    Copyright (c) 2015, Waterloo Aerial Robotics Group (WARG)
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

#ifndef TARGET_ANALYZER_H_INCLUDED
#define TARGET_ANALYZER_H_INCLUDED

/**
 * @file target_analyzer.h
 *
 * @brief Module for analyzing individual PixelTargets and combining them into 
 *        a set of unique Targets
 *
 * Module geolocates targets using their pixel locations
 * and photo metadata, determines target type and calculates 
 * possible error. As targets are processed unique targets will 
 * be identified and the data combined into a single object.
 *
 * The TargetAnalyzer does not delete any Frames or PixelTargets
 *
**/

#include "frame.h"
#include "pixel_object.h"
#include "object.h"
#include <math.h>

//Macros and Definitions
#define DEG2RAD(deg) (deg*180.0/M_PI)
#define RAD2DEG(rad) (rad*M_PI/180.0)
#define EARTH_RADIUS 6371000

//Classes

class TargetAnalyzer {
    static TargetAnalyzer * analyzer;
    TargetAnalyzer(){};

    void cleanup();                
    public:

        static TargetAnalyzer * getInstance(){
            if (!analyzer){
                analyzer = new TargetAnalyzer;
            }
            return analyzer;
        }
    
        /**
         * @brief Analyzes a pixeltarget and returns a pointer to the unique target 
         *        it belongs to
         *
         * @param p PixelTarget to be analyzed
         * @param f Frame that the PixelTarget belongs to
         * @return pointer to the Target that the PixelTarget belongs to
         */
  //      void analyze_targets_in_frame(PixelObject * p, Frame * f);
        //This will eventually need to be:
        //Target * analyze_targets_in_frame(PixelTarget?, Frame?...)

        /**
         * @brief retrieves a vector containing all of the unique Targets that 
         *        have been indentified. 
         *        The caller of the function should not delete the Targets as they
         *        will be deleted by the TargetAnalyzer destructor
         *        
         * @return vector containing the unique Targets which have been analyzed
         */
        //vector<Target *> get_processed();
        
        /**
         * @brief   Determines the longitude and latitude of the corners of the
         *          image, assuming the geolocation provided in the telemetry is the
         *          centroid of the photo. This function takes into account
         *          possible rotations of the photo from the heading of the
         *          aircraft.
         *          
         * @param   cameraAlpha Alpha_X and Alpha_Y values for the horizontal and vertical
         *          values for the camera viewing (half-)angle. Measured from
         *          the center of the photo to the outer most edge of the camera
         *          image.
         * @param   longitude The GPS longitude of the aircraft when the photo was
         *          taken.
         * @param   latitude The GPS latitude of the aircraft when the photo was
         *          taken.
         * @param   altitude The altitude of the aircraft from where it took the
         *          photo.
         * @param   heading The heading (GPS heading, not magnetic heading) from
         *          where the aircraft took the photo.
         * @param   img The image data for which the GPS coordinates are
         *          calculated for. 
         *        
         * @return  vector containing the unique Targets which have been analyzed
         */
        void getGPSCorners(cv::Point2d cameraAlpha, double longitude, double latitude,
double altitude, double heading, cv::Mat* img, Object* o);

        void getGPSCentroid(cv::Point2d point);
        void analyze_pixelobject(PixelObject* po);
        void analyzeTargetDuplicateProbability();
};

#endif // TARGET_ANALYZER_H_INCLUDED
