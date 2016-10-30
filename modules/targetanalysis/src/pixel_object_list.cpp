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
#include "pixel_object.h"
#include "pixel_object_list.h"

PixelObjectList::PixelObjectList(){
    listLength = 0;
    tail = head;
}

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

bool PixelObjectList::addNode(PixelObject* po){
    struct poNode* newNode = new struct poNode;
    if (newNode){
        newNode->po = po;
        newNode->next = 0; //Nullify pointer (Since there is no next list item)
        //Update old node with the new node
        head->next = newNode;
        //Change head to represent the new node
        head = newNode;
        //Update list length
        listLength++;
        return true;
    }

    return false;

}

bool PixelObjectList::getGPSDuplicates(){
    //TODO: Consider changing the return type.
    //Should return array of probabilities (% error based on known GPS error and
    //other parameters)
    //After multiplying by the visual probability, a good estimate of whether or
    //not it is a duplicate will be known
    int i = 0;
    while(i < listLength){
        tail->po->
        i++;
    }
    return true;
}

//TODO:Add later to compare duplicate targets from a visual perspective
//bool PixelObjectList::getVisualDuplicates(){
    
//}
