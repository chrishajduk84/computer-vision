/**
 * @file pixel_object_list.h
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


#include "frame.h"
#include "object.h"
#include "pixel_object.h"
#include "pixel_object_list.h"
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
        
        BOOST_LOG_TRIVIAL(debug) << "Adding new node";
        newNode->o->add_pobject(po);
        BOOST_LOG_TRIVIAL(debug) << "TEST";
        newNode->next = 0; //Nullify pointer (Since there is no next list item)
        BOOST_LOG_TRIVIAL(debug) << "TEST2";

        //Update old node with the new node
        if (head){  //If HEAD has been initialized already
            head->next = newNode;
        }
        else{ //If HEAD hasn't been initialized already
            tail = newNode;
        }
        BOOST_LOG_TRIVIAL(debug) << "TEST3";
        //Change head to represent the new node
        head = newNode;
        //Update list length
        listLength++;      
        BOOST_LOG_TRIVIAL(debug) << "New List Length: " << listLength;
    }

    double PixelObjectList::compareNode(PixelObject* po1, Object* o2){
        const vector<PixelObject*>& poList = o2->get_pobjects();
        
        double minimumError = 1;
        vector<cv::Point> minimumContour;
        for (PixelObject* po2 : poList){
            vector<cv::Point> v1 = po1->get_contour();
            vector<cv::Point> v2 = po2->get_contour();
            Point c1 = po1->get_centroid();
            Point c2 = po2->get_centroid();

            int a1 = po1->get_area();
            int a2 = po2->get_area();
        
            double areaScale = (double)a1/a2;

            //Rotate different Angles
            int numIntervals = 36; //Should vary based on computation time requirements
            vector<cv::Point> v2Mod[numIntervals]; 
            vector<cv::Point> v1Mod;
        
            //Center the contour at (0,0)
            for (Point i : v1){
                v1Mod.push_back(i - c1);
            }

            //Center the contour at (0,0) and rotate it
            for (int interval = 0; interval < numIntervals; interval++){
                double theta = 2*M_PI/numIntervals * interval;
            
                for (Point i : v2){
                    v2Mod[interval].push_back(Point(areaScale*cos(theta) - areaScale*sin(theta) - c2.x, areaScale*sin(theta) + areaScale*cos(theta)));
                }    
            }
            vector<vector<cv::Point>> v2hull;
            convexHull( Mat(v2Mod[0]), v2hull[0], false ); 
            Mat drawing = Mat::zeros(500,500, CV_8UC3);
            //TODO: figure out how to draw it to test out the algorithm
            //drawContours(drawing, vector<vector<Point>>(v2Mod,v2Mod+numIntervals*sizeof(v2Mod[0])), 0, cv::Scalar(255,255,255), FILLED);
            BOOST_LOG_TRIVIAL(debug) << "MADE IT";
        }
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

            BOOST_LOG_TRIVIAL(debug) << "Updating next one";
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

        }
        else{
            //Add the node to the end of the list for future comparisons
            //Note, that objects that have already matched are not included for
            //future comparisons...this may be unwise. Check the results and see
            //what happens during test cases.    
            addNode(po);
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

/*
//TODO:Add later to compare duplicate targets from a visual perspective
bool ObjectList::getVisualDuplicates(){
        
}
*/
