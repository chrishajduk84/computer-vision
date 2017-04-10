/**
 * @file pixel_object_list.cpp
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
#include "object.h"
#include "pixel_object.h"
#include "pixel_object_list.h"
#include "contour_comparison.h"
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/core.hpp>


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace boost;
using namespace std;
using namespace cv;


PixelObjectList* PixelObjectList::firstInstance = NULL;

PixelObjectList::~PixelObjectList(){
    int i = 0;
    poNode* tempPointer;
    while(i<listLength){
        tempPointer = tail;
        tail = tail->next;
        delete(tempPointer);
        i++;
    }
}

void PixelObjectList::addNode(PixelObject* po){
    //Initialize node
    struct poNode* newNode = new struct poNode;
    //Initialize object within node
    Object* newObject = new Object;
    newNode->o = newObject;
        
    newNode->o->add_pobject(po);
    newNode->o->update();
    newNode->next = 0; //Nullify pointer (Since there is no next list item)

    //Update old node with the new node
    if (head){  //If HEAD has been initialized already
        head->next = newNode;
    }
    else{ //If HEAD hasn't been initialized already
        tail = newNode;
    }
    //Change head to represent the new node
    head = newNode;
    //Update list length
    listLength++;      
    BOOST_LOG_TRIVIAL(debug) << "Node added. New List Length: " << listLength;
}

double PixelObjectList::compareNode(PixelObject* po1, Object* o2){
    //Use FLANN here?
    //Compare Pixel Objects based on Metadata and visual queues
    //Uses Hierarchical Structure: GPS data is most important, followed by
    //Contour/Shape, followed by Colour
        
    //TODO: GPS Implementation
    //double gps = compareGPS(PixelObject* po1, Object* o2);
    //if (gps > GPS_THRESHOLD){
        double visual = compareContours(po1, o2);
        if (visual > VISUAL_THRESHOLD){
            //double colour = compareColour(po1, o2);
            //if (colour > COLOUR_THRESHOLD){
            //  return gps*visual*colour;
            //}
            return visual; //Temporary return statement while I get all the other algorithms working
        }
    //}
    return 0;
}

double PixelObjectList::compareGPS(PixelObject* po1, Object* o2){
    //Calculations assume ideal scenario (no tilt in photos)
    Frame* f = po1->get_image();
        

    //FINISH
        
}

int PixelObjectList::getGPS(cv::Point2d point, cv::Point2d cameraAlpha,
Frame* f, cv::Point2d* returnResult){
    const Metadata* m = f->get_metadata();
    cv::Mat img = f->get_img();
    int h = img.cols;
    int w = img.rows;
    
    if (w <= 0 || h <= 0){ 
        return 0;
    }

    cv::Point2d imgCenter(w/2, h/2);
    
    //(0,0) is in the center of the image
    cv::Point2d biasedPoint = point - imgCenter;

    double altitude = m->altitude;
    double heading = m->heading;
    double latitude = m->lat;
    double longitude = m->lon;

    //Note: The cameraAlpha value represents the half angle of the x and y//direction of the image.
    double cameraXEdge = altitude / tan(DEG2RAD(90 - cameraAlpha.x)); //meters from center of photo to edge
    double cameraYEdge = altitude / tan(DEG2RAD(90 - cameraAlpha.y)); //meters from center of photo to edge

    //Rotation Matrix - Heading
    //Note: The '-heading' compensates for the fact that directional heading is
    //a clockwise quantity, but cos(theta) assumes theta is a counterclockwise
    //quantity.
    double realXEdge = cos(DEG2RAD(-heading)) * cameraXEdge - sin(DEG2RAD(-heading)) *
    cameraYEdge;
    double realYEdge = sin(DEG2RAD(-heading)) * cameraXEdge + cos(DEG2RAD(-heading)) *
    cameraYEdge;

    double realX = cos(DEG2RAD(-heading)) * biasedPoint.x/(w/2)*cameraXEdge - sin(DEG2RAD(-heading)) *
    biasedPoint.y/(h/2)*cameraYEdge;
    double realY = sin(DEG2RAD(-heading)) * biasedPoint.x/(w/2)*cameraXEdge + cos(DEG2RAD(-heading)) *
    biasedPoint.y/(h/2)*cameraYEdge;
    
    //Haversine formula - rearranged
    //Note: These are false coordinates, as they still need to be rotated (with
    //a rotation matrix)
    //double lon = acos(1 - (1 - cos(realX/EARTH_RADIUS))/(cos(DEG2RAD(latitude))*cos(DEG2RAD(latitude)))) + longitude;
    //double lat = realY/EARTH_RADIUS + latitude;
    double lon = RAD2DEG(realX/EARTH_RADIUS)/cos(DEG2RAD(latitude)) + longitude;
    double lat = RAD2DEG(realY/EARTH_RADIUS) + latitude;

    double unitX = realXEdge/img.cols;
    double unitY = realYEdge/img.rows;

    f->set_pixel_distance(unitX,unitY);    
    *returnResult = cv::Point2d(lat,lon);
    return 1;

}


//TODO: ADD OPTION TO ONLY LOOK AT PREDEFINED SHAPES
double PixelObjectList::compareContours(PixelObject* po1, Object* o2){
    const vector<PixelObject*>& poList = o2->get_pobjects();
        
    double maxSimilarity = 0;
    vector<cv::Point> minimumContour;
    for (PixelObject* po2 : poList){
        vector<cv::Point> v1 = po1->get_contour();
        vector<cv::Point> v2 = po2->get_contour();
        Point c1 = po1->get_centroid();
        Point c2 = po2->get_centroid();

        //TODO: Choose A1, A2, po1, po2 based on the smaller area...saves time on computation
        int a1 = po1->get_area();
        int a2 = po2->get_area();
            
        if (a1 == 0 || a2 == 0){
            continue;
        }

        //cArea determines size and accuracy of the matching. Should be at
        //least 100.
        double areaScale1 = (double)COMPARE_AREA/a1;
        double areaScale2 = (double)COMPARE_AREA/a2;

        //Rotate different Angles
        int numIntervals = 36; //Should vary based on computation time requirements
        vector<vector<cv::Point>> v2Mod; 
        vector<cv::Point> v1Mod;
        
        //Center the contour at (0,0)
        for (Point i : v1){
            v1Mod.push_back(areaScale1*(i - c1));
        }

        //Center the contour at (0,0) and rotate it and scale it
        for (int interval = 0; interval < numIntervals; interval++){
            double theta = 2*M_PI/numIntervals * interval;
            BOOST_LOG_TRIVIAL(debug) << "theta: " << theta << ", aS2: " <<
            areaScale2 << ", C2: " << c2 << endl;

            vector<cv::Point> tempVec;
            for (Point i : v2){
                tempVec.push_back(Point(areaScale2*cos(theta)*(i.x-c2.x) -
                areaScale2*sin(theta)*(i.y-c2.y),
                areaScale2*sin(theta)*(i.x-c2.x) + areaScale2*cos(theta)*(i.y-c2.y)));
            }    
            v2Mod.push_back(tempVec);
        }
            
        BOOST_LOG_TRIVIAL(debug) << v2Mod[0];
            
        float radius1, radius2;
        Point2f center1, center2;
        minEnclosingCircle(v1Mod,center1,radius1);
        minEnclosingCircle(v2Mod[0],center2,radius2);
        //If areas are similar, check size. If its way off (factor of 2), move onto next iteration.
        if (abs(radius1 - radius2) > radius1||abs(radius1 - radius2) > radius2){
            //Skip analysis to save computation time
            continue;
        }

        for (int i = 0; i < numIntervals; i++){
            //Reposition the contours for Mat operations (in the positive quadrant)
            Point matReference(radius1-center1.x > radius2-center2.x?radius1-center1.x:radius2-center2.x, radius1-center1.y > radius2-center2.y?radius1-center1.y:radius2-center2.y);
            for (int j = 0; j < v1Mod.size();j++){
                v1Mod[j] = v1Mod[j] + matReference;
            }
            for (int j = 0; j < v2Mod[i].size(); j++){
                v2Mod[i][j] = v2Mod[i][j] + matReference;
            }
            vector<vector<Point>> contoursWrapperA, contoursWrapperB;
            contoursWrapperA.push_back(v1Mod);
            contoursWrapperB.push_back(v2Mod[i]);
                
            //No point comparing contours if there is only one.
            if (listLength >= 1){
                double sim = compare_contours(contoursWrapperA, contoursWrapperB);
                if (maxSimilarity < sim){
                    BOOST_LOG_TRIVIAL(debug) << "Found new optimized position: " << sim;
                    maxSimilarity = sim;
                    minimumContour = v2Mod[i];
                        
                   /*//Visualization
                    if (sim > 0.7){
                        Mat drawing = Mat::zeros(matReference.x*2, matReference.y*2, CV_8UC3);

                        drawContours(drawing, contoursWrapperA, 0, cv::Scalar(255,0,255), FILLED);
                        drawContours(drawing, contoursWrapperB, 0, cv::Scalar(0,255,255), FILLED);
 
                        namedWindow( "Duplicate Contours", CV_WINDOW_AUTOSIZE );
                        imshow( "Duplicate Contours", drawing );
                        waitKey(0);
                    }*/   
                }
            }
        }
    } 
    return maxSimilarity;
}

void PixelObjectList::addAndCompare(PixelObject* po){
    //Iterate over list
    int i = 0;
    struct comparitor listMatch[listLength];
    double maxSimilarity = 0;
    int iMax = 0;

    poNode* tempPointer;
    tempPointer = tail;
    BOOST_LOG_TRIVIAL(debug) << "List Length: " << listLength;
    while(i<listLength){
        //Returns similarity percentage - 1 is an ideal match, 0 is the worst
        //match possible
        listMatch[i].similarity = compareNode(po, tempPointer->o);
        BOOST_LOG_TRIVIAL(debug) << "Compare Done";
        listMatch[i].node = tempPointer;

        //Update the best match if a better one is available
        if (listMatch[i].similarity > maxSimilarity){
            maxSimilarity = listMatch[i].similarity;
            iMax = i;
        }
        i++;
        tempPointer = tempPointer->next;
    }
    //Is the best match good enough - FUZZY LOGIC
    //TODO: This should be a configurable value via a socket connection or a
    //learning algorithm
    if (maxSimilarity >= MATCH_THRESHOLD){
        //The add_pobject function should also recalculate all parameters of the
        //object.
        listMatch[iMax].node->o->add_pobject(po);
        listMatch[iMax].node->o->update();
   }
    else{
        //Add the node to the end of the list for future comparisons
        //Note, that objects that have already matched are not included for
        //future comparisons...this may be unwise. Check the results and see
        //what happens during test cases.    
        addNode(po);
    }

}

int PixelObjectList::getListLength(){
    return listLength;
}

Object* PixelObjectList::getObject(int index){
    struct poNode* node = tail; 
    for (int i = 0; i < index; i++){
        node = node->next;
    }
    return node->o;
}

void PixelObjectList::getObjects(vector<Object*>* v){
    struct poNode* node = tail;
    for (int i = 0; i < listLength; i++){
        v->push_back(node->o);
        node = node->next;
    }
}

/* NOT USING THIS
bool PixelObjectList::getGPSDuplicates(){
    //TODO: Consider changing the return type.
    //Should return array of probabilities (% error based on known GPS error and
    //other parameters)
    //After multiplying by the visual probability, a good estimate of whether or
    //not it is a duplicate will be known
    int i = 0;
    while(i < listLength){
        tail->o->
        i++;
    }
    return true;
}
*/

