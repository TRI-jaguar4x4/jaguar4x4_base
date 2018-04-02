// Copyright 2018 Toyota Research Institute.  All rights reserved.

#pragma once

#include <string>

// Terminology from http://www.cplusplus.com/reference/string/string/compare/
bool startsWith(const std::string& compared,
                const std::string& comparing);

std::string dumpHex(const std::string& msg);

double str_to_d(const std::string& in);
