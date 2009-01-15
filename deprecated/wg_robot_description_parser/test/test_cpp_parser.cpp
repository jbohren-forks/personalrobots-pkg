/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \Author Ioan Sucan, Stu Glassner */

#include <urdf/URDF.h>
#include <urdf/parser.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <string>
using namespace robot_desc;

int runExternalProcess(const std::string &executable, const std::string &args)
{
    return system((executable + " " + args).c_str());
}

TEST(URDF, EmptyFile)
{
    URDF file;
    file.loadFile("test/data/test1.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);

    std::ofstream f("/tmp/test1.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test1.txt /tmp/test1.txt");

    EXPECT_TRUE(result == 0);
}

TEST(URDF, SimpleFile)
{
    URDF file;
    file.loadFile("test/data/test2.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);

    std::ofstream f("/tmp/test2.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test2.txt /tmp/test2.txt");

    EXPECT_TRUE(result == 0);
}


TEST(URDF, ComplexFile)
{
    URDF file;
    file.loadFile("test/data/test3.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);

    std::ofstream f("/tmp/test3.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test3.txt /tmp/test3.txt");

    EXPECT_TRUE(result == 0);
}

// A great test, but I'm commenting it out, to avoid adding an installation
// dependency on valgrind. - BPG
/*
TEST(URDF, Valgrind)
{
    int result = runExternalProcess("./run_valgrind.py", "./parse test/data/test3.xml");

    EXPECT_TRUE(result == 0);
}
*/

//------------------------------------------------------------------------------
//  Normalization

bool xmlMatches(TiXmlElement *a, TiXmlElement *b)
{
  if (a == NULL && b == NULL)
    return true;
  if (a == NULL || b == NULL)
    return false;

  if (a->ValueStr() != b->ValueStr())
    return false;

  for (TiXmlAttribute *att = a->FirstAttribute(); att; att = att->Next())
  {
    std::string va = att->ValueStr();
    std::string vb = b->Attribute(att->Name());

    if (atof(va.c_str()) != 0)
    {
      if (fabs(atof(va.c_str()) - atof(vb.c_str())) > 0.00000001)
        return false;
    }
    else if (att->ValueStr() != b->Attribute(att->Name()))
      return false;
  }

  if (!xmlMatches(a->FirstChildElement(), b->FirstChildElement()))
    return false;
  if (!xmlMatches(a->NextSiblingElement(), b->NextSiblingElement()))
    return false;

  return true;
}

TEST(URDF, NormalizeASimpleDoc)
{
  TiXmlDocument xml, desired;
  xml.Parse("<should do='nothing'>hopefully</should>");
  desired.Parse("<should do='nothing'>hopefully</should>");
  EXPECT_TRUE(urdf::normalizeXml(xml.RootElement()));
  EXPECT_TRUE(xmlMatches(xml.RootElement(), desired.RootElement()));
}

TEST(URDF, NormalizeOneConstant)
{
  TiXmlDocument xml, desired;
  xml.Parse("<x><const name='huzzah' value='3.0' /><foo bar='huzzah' /></x>");
  desired.Parse("<x><foo bar='3.0' /></x>");
  EXPECT_TRUE(urdf::normalizeXml(xml.RootElement()));
  EXPECT_TRUE(xmlMatches(xml.RootElement(), desired.RootElement()));
}

TEST(URDF, NormalizeOneBlock)
{
  TiXmlDocument xml, desired;
  xml.Parse("<x><const_block name='foo'><bar meaning='nothing' /></const_block> \
<insert_const_block name='foo' /></x>");
  desired.Parse("<x><bar meaning='nothing' /></x>");
  EXPECT_TRUE(urdf::normalizeXml(xml.RootElement()));
  EXPECT_TRUE(xmlMatches(xml.RootElement(), desired.RootElement()));
}

TEST(URDF, NormalizeAnExpressionWithConstants)
{
  TiXmlDocument xml, desired;
  xml.Parse("<x> \
<const name='aaa' value='4.5'/> \
<const name='bbb' value='7.3'/> \
<const name='ccc' value='2.0'/>  \
<y><z foo='(aaa*bbb)/ccc' /></y></x>");
  desired.Parse("<x><y><z foo='16.425' /></y></x>");
  EXPECT_TRUE(urdf::normalizeXml(xml.RootElement()));
  EXPECT_TRUE(xmlMatches(xml.RootElement(), desired.RootElement()));
}

TEST(URDF, NormalizeAVectorWithConstants)
{
  TiXmlDocument xml, desired;
  xml.Parse("\
<x> <const name='foo' value='0.4' />  \
<grump arg='foo 0 0' />  \
</x> ");
  desired.Parse("<x><grump arg='0.4 0 0' /></x>");
  EXPECT_TRUE(urdf::normalizeXml(xml.RootElement()));
  EXPECT_TRUE(xmlMatches(xml.RootElement(), desired.RootElement()));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
