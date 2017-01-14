/**
 * @file unique_po_test.cpp
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
            
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TargetAnalysis

#include <boost/test/unit_test.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/property_tree/ptree.hpp>
#include "target_analyzer.h"

using namespace std;
using namespace boost;

namespace logging = boost::log;
using namespace boost::filesystem;

typedef struct _poList{
    PixelObject* po;
    _poList* next;

} POList;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}


BOOST_AUTO_TEST_CASE(UniquePOTest){
    if(boost::unit_test::framework::master_test_suite().argc <= 1) {
        BOOST_ERROR("Invalid number of arguments");
    }
    
    string root_path = boost::unit_test::framework::master_test_suite().argv[1];
    string description = boost::unit_test::framework::master_test_suite().argv[2];    
    
    //Read directory
    BOOST_REQUIRE( filesystem::exists( root_path ) );
    directory_iterator end_itr; // default construction yields past-the-end
    int numImage = 0;
    for ( directory_iterator itr( root_path ); itr != end_itr; ++itr ){
        if ( is_regular_file(itr->path()) ){
            string current_file = itr->path().string();
            if (has_suffix(current_file, ".jpg") || has_suffix(current_file,
            ".jpeg")){
                //imread() - FINISH LATER
                //Generate cv::mat
                //PixelObject po(//FILL THIS IN );

                numImage++;
            }

        }
    }
    
    //Artifically construct pixel objects
    PixelObject POTests[numImage];
    for (int i = 0; i < numImage; i++){
        POTests[i]   ; //Do things
        
        //Inject into analysis
        analyze_pixelobject(&(POTests[i])); 
    }

    //BOOST_REQUIRE();

    //NEGATIVE CHECK - Check how many different targets there are
    

    //POSITIVE CHECK - Check how many targets got grouped together and how many
    //in each according to the predefined images


    //BOOST_CHECK();
    
   
}
