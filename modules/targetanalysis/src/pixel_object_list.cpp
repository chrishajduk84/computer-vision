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
        //tail->po-> 
        i++;
    }
    return true;
}

//TODO:Add later to compare duplicate targets from a visual perspective
//bool PixelObjectList::getVisualDuplicates(){
    
//}
