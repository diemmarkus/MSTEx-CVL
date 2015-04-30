/*******************************************************************************************************
 DkError.cpp
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Markus Diem <markus@nomacs.org>
 Copyright (C) 2014-2015 Fabian Hollaus <holl@caa.tuwien.ac.at>
 Vienna University of Technology, Computer Vision Lab
 This file is part of ViennaMS.

 ViennaMS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 ViennaMS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *******************************************************************************************************/

#include "DkError.h"
#include "DkUtils.h"

using std::string;

DkException::DkException(const std::string & msg) 
    : errMsg(msg),  errFile( "" ), errLine(-1) {
		exceptionName = "DkException";
}

DkException::DkException(const std::string & msg, int line, const std::string & file ) 
    : errMsg(msg), errFile(file), errLine(line)  {
		exceptionName = "DkException";
}


DkException::~DkException() throw() {
}


const char* DkException::what() const throw() {
	return errMsg.c_str();
}

const string DkException::Msg() const {
	string s;

	s = "\n>> " + exceptionName + ": " + errMsg + 
		"\n   in line: " + DkUtils::stringify(errLine) + " in file: " + errFile + "\n\n";

    return s;
}

int DkException :: Line() const {
    return errLine;
}

const string & DkException :: File() const {
    return errFile;
}

//DkIllegalArgumentException::DkIllegalArgumentException(const std::string &msg) : DkException(msg) {
//	exceptionName = "DkIllegalArgumentException";
//}
//DkIllegalArgumentException::DkIllegalArgumentException(const std::string &msg, int line, const std::string &file) 
//	: DkException(msg, line, file) {
//	exceptionName = "DkIllegalArgumentException";
//}
