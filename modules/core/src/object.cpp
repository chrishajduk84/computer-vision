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

#include "object.h"
#include "pixel_object.h"

Object::Object(){

}

void Object::add_pobject(PixelObject * po){
    pixelObjects.push_back(po);
}

void Object::update(){
    int n = Object::pixelObjects.size();
    this->centroid = cv::Point2f(0,0);
    cv::Point2d maxDistance(0,0);
    cv::Scalar cSum;
    for (PixelObject* po : Object::pixelObjects){
        this->centroid += po->get_gps_centroid();
        this->area += po->get_gps_area();
        cSum += po->get_colour();

        if (sqrt(pow(maxDistance.x,2) + pow(maxDistance.y,2)) <
        sqrt(pow(po->get_gps_centroid().x,2) + pow(po->get_gps_centroid().y,2))){
            maxDistance = po->get_gps_centroid();
        }
        //TODO: Determine image quality, choose best image

    }
    this->centroid /= n;
    this->colour = cSum/n;

    this->error = maxDistance - this->centroid;
    this->errorAngle = atan2(this->centroid.y - maxDistance.y, this->centroid.x - maxDistance.x);
}

const std::vector<PixelObject*>& Object::get_pobjects(){
    return pixelObjects; 
}


