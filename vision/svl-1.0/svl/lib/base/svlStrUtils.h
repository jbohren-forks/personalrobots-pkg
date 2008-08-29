/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlStrUtils.h
** AUTHOR(S):   Ian Goodfellow <ia3n@stanford.edu>
**              Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Generic string utilities.
**
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <sstream>
#include <map>

using namespace std;

// Templated function to make conversion from simple data types like
// ints and doubles to strings easy for debugging. This avoids messy
// char buffers.
template<class T>
std::string toString(const T& v);

template<class T>
std::string toString(const std::vector<T>& v);

template<class T>
std::string toString(const std::set<T>& v);

// Specialized toString() routines
std::string toString(const map<std::string, std::string>& m);

// Conversion from a string.
template<class T>
int parseString(const std::string& str, std::vector<T>& v);

map<string, string> parseNameValueString(string str);

// Pad string with character to given size.
std::string padString(const std::string& str, int padLength,
                      unsigned char padChar = '0');

// Replaces any occurrences in str of substr with rep
string strReplaceSubstr(const string & str, const string & substr, const string & rep);

// Some filename/directory processing functions
string strBaseName(const string &fullPath);
string strFilename(const string &fullPath);
string strDirectory(const string &fullPath);
string strExtension(const string &fullPath);
int strFileIndex(const string &fullPath);

// Implementation -----------------------------------------------------------

template<class T>
std::string toString(const T& v)
{
    std::stringstream s;
    s << v;
    return s.str();
}

template<class T>
std::string toString(const std::vector<T>& v)
{
    std::stringstream s;
    for (unsigned i = 0; i < v.size(); i++) {
        s << " " << v[i];
    }
    return s.str();
}

template<class T>
std::string toString(const std::set<T>& v)
{
    std::stringstream s;
    s << "{";
    for (typename std::set<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
        s << " " << *it;
    }
    s << " }";
    return s.str();
}

// Conversion from a string
template<class T>
int parseString(const std::string& str, std::vector<T>& v)
{
    std::stringstream buffer;
    T data;
    int count;

    buffer << str;
    count = 0;
    while (1) {
        buffer >> data;
        if (buffer.fail()) {
            break;
        }
        v.push_back(data);
        count++;
    }

    return count;
}


