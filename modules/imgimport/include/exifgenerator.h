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
	
#ifndef EXIF_GENERATOR_H_INCLUDED
#define EXIF_GENERATOR_H_INCLUDED


#include "frame.h"
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

/** 
 * @class ExifGenerator
 *
 * @brief Module for placing (specific) metadata into frames extracted from either video or photos.
 *
 * Opens a frame with each instantiation of this object, then adds the appropriate metadata and saves the frame. 
 * The results are later used in "Human Vision". 
 *
 */

class ExifGenerator{
    public:
        /**
         * Creates an Exif object from a OpenCV Mat object
         */
        ExifGenerator(cv::Mat* img){
            	init(img);
       	}	

        ~ExifGenerator(){
        	save();
	}

        /**
         * Begins capture. Creates a video stream which a frame may be grabbed from.
        *
         * @return Status indicating whether the operation was successful or not.
         */
        int appendMetadata(std::string data);

        /**
         * Stops capture and releases the device for use by other software.
         *
         * @return Status indicating whether the operation was successful or not.
         */
	int clearMetadata();

        /**
         * Instantaneously retrieves a frame from the video stream in OpenCV format. 
         * 
         * @param frame Allocated OpenCV frame (Mat) to dump the frame data into.
         * 
         * @return Status indicating whether the operation was successful or not.
         */
        int changeExif(std::string field, std::string data);
       
        /**
         * Instantaneously retrieves a frame from the video stream in OpenCV format. 
         * 
         * @param frame Allocated OpenCV frame (Mat) to dump the frame data into.
         * 
         * @return Status indicating whether the operation was successful or not.
         */
        std::string readExif(std::string field);
        
        /**
         * Instantaneously retrieves a frame from the video stream in OpenCV format. 
         * 
         * @param frame Allocated OpenCV frame (Mat) to dump the frame data into.
         * 
         * @return Status indicating whether the operation was successful or not.
         */        
        int save();
    private:
        /**
         * Function called when the ExifGenerator object is initialized. The requires an 
         * image to add metadata to. 
         * 
         */
        void init(cv::Mat* img);
};

#endif // EXIF_GENERATOR_H_INCLUDED
