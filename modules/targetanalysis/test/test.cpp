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

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/property_tree/ptree.hpp>
#include "target_loader.h"

using namespace std;
using namespace boost;

namespace logging = boost::log;
/*
class LoadFileTest : public BoolTest<string &> {
    public:
        LoadFileTest(string s): BoolTest(s) { }

    protected:
        bool test(string & arg) {
            TargetLoader loader(arg);
	    //loader.print(*loader.jsonParameters);
	    //property_tree::ptree* root = loader.getJSON();
	    return false;
        }
};
*/
int main(int argc, char ** argv) {
    logging::core::get()->set_filter
    (
       logging::trivial::severity >= logging::trivial::info
    );
    if(argc <= 1) {
        BOOST_LOG_TRIVIAL(info) << "Invalid arguments for test";
        return 1;
    }
    const char* file = argv[1];
    string description = argv[2];    
    TargetLoader loader(file);
    property_tree::ptree* pt = loader.getJSON(); 
    loader.print();
    /*LoadFileTest test("Simple JSON File Load Process Using BOOST");
    double result = test.do_test(file, description, false);
    cout << result;*/
    return 0;//!(result == 10); // arbitrary bounds for success of test
}
