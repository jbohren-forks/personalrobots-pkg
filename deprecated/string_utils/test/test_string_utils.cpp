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

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include <string_utils/string_utils.h>


TEST(Trim, Empty)
{
    EXPECT_EQ("", string_utils::trim(""));
}

TEST(Trim, WhiteSpace)
{
    EXPECT_EQ("", string_utils::trim("   \t \n "));
}

TEST(Trim, WordStart)
{
    EXPECT_EQ("word", string_utils::trim(" \tword"));
}

TEST(Trim, WordEnd)
{
    EXPECT_EQ("word", string_utils::trim("word   \n"));
}

TEST(Trim, Word)
{
    EXPECT_EQ("my word", string_utils::trim("\tmy word   \n"));
}


TEST(Case, Upper)
{
    EXPECT_EQ("TEST", string_utils::toupper("tEst"));
}

TEST(Case, Lower)
{
    EXPECT_EQ("test", string_utils::tolower("tEst"));
}

TEST(ConvertToStr, Int)
{
    EXPECT_EQ("12", string_utils::convert2str<int>(12));    
}

TEST(ConvertToStr, Double)
{
    EXPECT_EQ("1.23", string_utils::convert2str<double>(1.23));    
}

TEST(ConvertToStr, Char)
{
    EXPECT_EQ("a", string_utils::convert2str<char>('a'));    
}

TEST(ConvertFromStr, Int)
{
    EXPECT_EQ(12, string_utils::fromString<int>(" 12 "));    
}

TEST(ConvertFromStr, Double)
{
    EXPECT_EQ(1.23, string_utils::fromString<double>("1.23 "));    
}

TEST(ConvertFromStr, Char)
{
    EXPECT_EQ('a', string_utils::fromString<char>("a"));    
}

TEST(Split, Sentence)
{
    std::vector<std::string> words;
    string_utils::split("  my sentence     test ", words);
    int count = words.size();
    EXPECT_EQ(count, 3);
    EXPECT_EQ(words[0], "my");
    EXPECT_EQ(words[1], "sentence");
    EXPECT_EQ(words[2], "test");
    
    words.clear();    
    string_utils::split("  myDsentence DtestDD ", words, "D ");
    count = words.size();
    EXPECT_EQ(count, 3);
    EXPECT_EQ(words[0], "my");
    EXPECT_EQ(words[1], "sentence");
    EXPECT_EQ(words[2], "test");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
