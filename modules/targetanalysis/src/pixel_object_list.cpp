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

using namespace boost;

namespace PixelObjectList{

    const double MATCH_THRESHOLD = 0.5;
    
    void cleanup(){
        int i = 0;
        poNode* tempPointer;
        while(i<listLength){
            tempPointer = tail;
            tail = tail->next;
            delete(tempPointer);
            i++;
        }
    }

    void addNode(PixelObject* po){
        //Initialize node
        struct poNode* newNode = new struct poNode;
        //Initialize object within node
        Object* newObject = new Object;
        newNode->o = newObject;
        
        newNode->o->add_pobject(po);
        newNode->next = 0; //Nullify pointer (Since there is no next list item)
        //Update old node with the new node
        head->next = newNode;
        //Change head to represent the new node
        head = newNode;
        //Update list length
        listLength++;       
    }

    double compareNode(PixelObject* po1, Object* o2){
        std::vector<cv::Point> v1 = po1->get_contour();
        //std::vector<cv::Point> v2 = po2->get_contour();
    
        for (cv::Point i : v1){
            BOOST_LOG_TRIVIAL(debug) << i;
        }
        /*for (cv::Point i : v2){
            BOOST_LOG_TRIVIAL(debug) << i;
        } */  
    }

    void addAndCompare(PixelObject* po){
        //Iterate over list
        int i = 0;
        struct comparitor listMatch[listLength];
        double maxSimilarity = 0;
        int iMax = 0;

        poNode* tempPointer;
        tempPointer = tail;
        while(i<listLength){
            //Returns similarity percentage - 1 is an ideal match, 0 is the worst
            //match possible
            listMatch[i].similarity = compareNode(po, tempPointer->o);
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

        }
        else{
            //Add the node to the end of the list for future comparisons
            //Note, that objects that have already matched are not included for
            //future comparisons...this may be unwise. Check the results and see
            //what happens during test cases.    
            addNode(po);
        }

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
