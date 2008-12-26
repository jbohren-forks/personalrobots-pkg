// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// author: Rosen Diankov
#include <gtest/gtest.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <openrave-core.h>

using namespace OpenRAVE;
using namespace std;

vector<string> tokenizer(string str, string delims)
{
	vector<string> tokens;
	int pos, pos2;
	pos = str.find_first_not_of(delims, 0);

	while (pos >= 0)
	{
		pos2 = str.find_first_of(delims, pos);
		if (pos2 < 0) pos2 = str.length();
		tokens.push_back(str.substr(pos, pos2-pos));
		pos = str.find_first_not_of(delims, pos2);
	}
	return tokens;
}

TEST(URDF, LoadRobots)
{
    boost::shared_ptr<EnvironmentBase> penv(CreateEnvironment());
        
    bool bSuccess = true;
    vector<string> files = tokenizer(ROBOT_FILES,":; \r\n");
    for(vector<string>::iterator it = files.begin(); it != files.end(); ++it) {
        cout << "testing: " << *it << "... ";
        penv->Reset();
        GetXMLErrorCount();

        RobotBase* probot = penv->CreateRobot("GenericRobot");
        if( !probot ) {
            cout << "fail to create a GenericRobot: " << endl;
            bSuccess = false;
            continue;
        }

        if( !probot->Init(it->c_str(), NULL) ) {
            cout << "fail to init file" << endl;
            bSuccess = false;
            continue;
        }

        int xmlerror = GetXMLErrorCount();
        if( xmlerror != 0 ) {
            cout << "fail: xml error count: " << xmlerror << endl;
            bSuccess = false;
            continue;
        }

        // check that all velocities are > 0
        vector<dReal> velocities;
        probot->GetJointMaxVel(velocities);

        for(size_t i = 0; i < velocities.size(); ++i) {
            if( velocities[i] == 0 ) {
                bSuccess = false;
                cout << "joint " << i << " has 0 max velocity!" << endl;
            }
        }

        cout << "success." << endl;
    }
    
    EXPECT_TRUE(bSuccess);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
