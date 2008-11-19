#include <gtest/gtest.h>
#include <cstdlib>
#include <string>
#include <iostream>
using namespace std;

int runExternalProcess(const std::string &executable, const std::string &args)
{
    return system((executable + " " + args).c_str());
}

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
    bool bSuccess = true;
    vector<string> files = tokenizer(ROBOT_FILES,"; \r\n");
    for(vector<string>::iterator it = files.begin(); it != files.end(); ++it) {
        cout << "testing: " << *it << "... ";
        int result = runExternalProcess(OPENRAVE_EXECUTABLE, *it + string(" -testscene"));
        if( result != 0 ) {
            bSuccess = false;
            cout << "fail." << endl;
        }
        else
            cout << "success." << endl;
    }
    
    EXPECT_TRUE(bSuccess);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
