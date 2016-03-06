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

#include <iostream>
#include <utility>
#include <vector>
#include <errno.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <exiv2/exiv2.hpp>

#include "exifgenerator.h"

#define METADATA_NAMESPACE "Exif.Photo."
#define METADATA_DATA "UserComment"
#define METADATA_SIZELIMIT 255

using namespace boost;
using namespace std;
using namespace Exiv2;
using namespace cv;

namespace logging = boost::log;

Exiv2::Image::AutoPtr image;
Exiv2::ExifData exifData;

void ExifGenerator::init(Mat* img){
    size_t size = img->step[0] * img->rows;
    image = Exiv2::ImageFactory::open((const byte*)img, (long)size);
    image->readMetadata();
    exifData = image->exifData();
}

int ExifGenerator::appendMetadata(string data){
    string currentData = readExif(METADATA_DATA);
    return changeExif(METADATA_DATA, data);
}

int ExifGenerator::clearMetadata(){
    return changeExif(METADATA_DATA, readExif(METADATA_DATA));
}

int ExifGenerator::changeExif(string field, string data){
   if (currentData.size() + (*data).size() >= 255){
        BOOST_LOG_TRIVIAL(error) << "EXIF Data is too large! Cannot append metadata!";
        return 1;
    }
    exifData[METADATA_NAMESPACE + field] = currentData + (*data);
    BOOST_LOG_TRIVIAL(info) << "EXIF Data Size: " << currentData;
    return 0;
}

string ExifGenerator::readExif(string field){
    return exifData[METADATA_NAMESPACE + field];
}

int ExifGenerator::save(){
    return image->writeMetadata();
}
