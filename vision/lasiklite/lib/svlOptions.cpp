/******************************************************************************
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
** DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
**
** FILENAME:    svlOptions.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <vector>
#include <map>
#include <iostream>

#include "svlOptions.h"

using namespace std;

#define PRINT_ERRORS
#undef PRINT_ERRORS

// svlOptions Public Members --------------------------------------------------

void svlOptions::setOption(const string& name, const string& value)
{
    // check that the option has been declared
#ifdef PRINT_ERRORS
    if (_options.find(name) == _options.end()) {
	cerr << "ERROR: option " << name.c_str()
	     << " not declared in svlOption::setOption()" << endl;
    }
    if (value.empty()) {
	cerr << "ERROR: value empty for option " << name.c_str()
	     << " in svlOption::setOption()" << endl;
    }
#endif
    assert(_options.find(name) != _options.end());
    assert(!value.empty());
    // set the option
    _options[name] = value;
    // notify derived class that option has changed
    optionChanged(name, value);
}

void svlOptions::setOption(const string& name, const char *value)
{
    setOption(name, string(value));
}

void svlOptions::setOption(const string& name, int value)
{
    setOption(name, toString(value));
}

void svlOptions::setOption(const string& name, double value)
{
    setOption(name, toString(value));
}

void svlOptions::setOption(const string& name, bool value)
{
    setOption(name, value ? string("true") : string("false"));
}

void svlOptions::setOptions(const map<string, string>& options)
{
    for (map<string, string>::const_iterator i = options.begin(); 
	 i != options.end(); i++) {
	setOption(i->first, i->second);
    }
}

void svlOptions::setOptionsFromString(const string& options)
{
    if (options.empty()) return;
    map<string, string> o = parseNameValueString(options);
    setOptions(o);
}

const string& svlOptions::getOption(const string& name) const
{
    // check that the option has been declared
#ifdef PRINT_ERRORS
    if (_options.find(name) == _options.end()) {
	cerr << "ERROR: option " << name.c_str()
	     << " not declared in svlOption::getOption()" << endl;
    }
#endif
    assert(_options.find(name) != _options.end());
    // return the option value
    return _options[name];
}

bool svlOptions::getOptionAsBool(const string& name) const
{
    return ((!strcasecmp(getOption(name).c_str(), "true")) ||
	(!strcasecmp(getOption(name).c_str(), "yes")));
}

int svlOptions::getOptionAsInt(const string& name) const
{
    return atoi(getOption(name).c_str());
}

double svlOptions::getOptionAsDouble(const string& name) const
{
    return atof(getOption(name).c_str());
}

vector<string> svlOptions::getOptionNames() const
{
    vector<string> keys;
    
    keys.reserve(_options.size());
    for (map<string, string>::const_iterator i = _options.begin();
	 i != _options.end(); i++) {
	keys.push_back(i->first);
    }

    return keys;
}

// svlOptions Protected Members ----------------------------------------------

void svlOptions::declareOption(const string& name, const string& defaultValue)
{
    // check that the option hasn't been declared already
#ifdef PRINT_ERRORS
    if (_options.find(name) != _options.end()) {
	cerr << "ERROR: option " << name.c_str()
	     << " already declared in svlOption::declareOption()" << endl;
    }
#endif
    assert(_options.find(name) == _options.end());
    // set the options default value
    _options[name] = defaultValue;
}

void svlOptions::declareOption(const string& name, const char *defaultValue)
{
    declareOption(name, (defaultValue == NULL) ? string("true") : string(defaultValue));
}

void svlOptions::declareOption(const string& name, int defaultValue)
{
    declareOption(name, toString(defaultValue));
}

void svlOptions::declareOption(const string& name, double defaultValue)
{
    declareOption(name, toString(defaultValue));
}

void svlOptions::declareOption(const string& name, bool defaultValue)
{
    declareOption(name, defaultValue ? string("true") : string("false"));
}

void svlOptions::undeclareOption(const string& name)
{
    // check that the option has been declared
#ifdef PRINT_ERRORS
    if (_options.find(name) == _options.end()) {
	cerr << "ERROR: option " << name.c_str()
	     << " not declared in svlOption::undeclareOption()" << endl;
    }
#endif
    assert(_options.find(name) != _options.end());
    // remove the option
    _options.erase(name);
}

// Function to break string of "<name>\s*=\s*<value>[,; ]" pairs
// into an stl map. If the value part does not exist then sets to
// "true".
map<string, string> svlOptions::parseNameValueString(string str) const
{
    // first tokenize into <name>=<value> pairs
    vector<string> tokens;
    string::size_type lastPos = str.find_first_not_of(" ", 0);
    string::size_type pos = str.find_first_of(",; ", lastPos);

    while ((string::npos != pos) || (string::npos != lastPos)) {
        // found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // skip delimiters
        lastPos = str.find_first_not_of(",; ", pos);
        // find next "non-delimiter"
        pos = str.find_first_of(",; ", lastPos);
    }

    // now break tokens into name and value pairs
    map<string, string> output;
    for (unsigned i = 0; i < tokens.size(); i++) {
	pos = tokens[i].find_first_of("=", 0);
	if (pos != string::npos) {
	    output[tokens[i].substr(0, pos)] = 
		tokens[i].substr(pos + 1, tokens[i].length() - pos);
	} else {
	    output[tokens[i]] = "true";
	}
    }
   
    return output;
}
