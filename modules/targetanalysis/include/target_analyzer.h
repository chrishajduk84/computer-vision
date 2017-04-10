/**
 * @file target_analyzer.h
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
#include <vector>

//Classes

/**
 * @class TargetAnalyzer
 *  
 * TargetAnalyzer is a wrapper module for the PixelObjectList class. It handles
 * the entry and exit points for every form of interaction with the target
 * analysis module.
 * This class is a singleton.
 *
 * @brief Provides a procedure for analyzing pixel objects, as well as
 * extracting data from the results. PixelObjectList is a singleton.
 *
 */
class TargetAnalyzer {
    /*
     * analyzer is the static singleton instance of the TargetAnalyzer class
     */
    static TargetAnalyzer * analyzer;
    /*
     * Private constructor due to singleton design pattern.
     */ 
    TargetAnalyzer(){};
      
    public:
        /*
         * getInstance() returns the singleton instance of this class. If it is not
         * instantiated it is initialized.
         * @return the singleton TargetAnalyzer
         */
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

        /*
         * analyze_pixelobject(PixelObject* po) is the entry function into this
         * module, it initializes the comparisons that are made for each
         * PixelObject.
         * @param po the PixelObject that as being analyzed.
         */
        void analyze_pixelobject(PixelObject* po);

        /*
         * A function which based on any point in an image, provides the GPS
         * coordinates of that point.
         * -> THIS FUNCTION IS STILL WIP AND WILL BE FULLY DEVELOPED IN A
         * SEPERATE PR
         */
        void getGPSCentroid(cv::Point2d point);
        
        /*
         * @brief getGPSDistance() calculates the distance between two GPS coordinates in meters.
         * @param lat1 the latitude of the first point
         * @param lon1 the longitude of the first point
         * @param lat2 the latitude of the second point
         * @param lon2 the longitude of the second point
         * @return the distance in meters between the two GPS coordinates.
         */
        double getGPSDistance(double lat1, double lon1, double lat2, double lon2);

        /*
         * extract_objects() provides a list of unique objects that have been
         * identified, where the non-unique ones have been grouped.
         * @return the vector of Objects with identifies the unique ones found.
         */
        std::vector<Object*> extract_objects();

        /*
         * get_unique_objects_length() provides the total number of objects that
         * have been found within the data acquired.
         * @return the integer number of Objects that can be extracted by using
         * the extract_objects() function.
         */ 
        int get_unique_objects_length();
};

#endif // TARGET_ANALYZER_H_INCLUDED
