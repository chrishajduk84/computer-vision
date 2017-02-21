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

#ifndef PIXEL_OBJECT_LIST_H_INCLUDED
#define PIXEL_OBJECT_LIST_H_INCLUDED

/**
 * @file pixel_object_list.h
 *
 * @brief Class which describes a structure for storing pixel targets and then
 * later finding a matching set of targets.
 *
 * Module geolocates targets using their pixel locations
 * and photo metadata, determines target type and calculates 
 * possible error. As targets are processed unique targets will 
 * be identified and the data combined into a single object.
 *
 *
**/

#include "frame.h"
#include "object.h"
#include "pixel_object.h"

class PixelObjectList{

const double MATCH_THRESHOLD = 0.5;

private:
    struct poNode{
        Object* o;
        struct poNode* next;

    };

    struct comparitor{
        double similarity;
        poNode* node;
    };


    poNode* head = NULL;
    poNode* tail = NULL;
    int listLength = 0;
    
    static PixelObjectList* firstInstance;

    //This list is a singleton instance - These definitions need to be private.
    PixelObjectList(){};
    PixelObjectList& operator=(PixelObjectList*){}; // Private assignment operator
    PixelObjectList(PixelObjectList const&){};
    ~PixelObjectList();
public:
    
    static PixelObjectList* getInstance(){
        if (!firstInstance){
            firstInstance = new PixelObjectList;
        }
        return firstInstance;
    }

    void addNode(PixelObject* o);
    double compareNode(PixelObject* po1, Object* o2);
    void addAndCompare(PixelObject* po);
        
    //bool getGPSDuplicates();
    //bool getVisualDuplicates();
};
#endif // PIXEL_OBJECT_LIST_H_INCLUDED
