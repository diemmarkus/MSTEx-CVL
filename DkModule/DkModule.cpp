/*******************************************************************************************************
 DkModule.cpp
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

#include "DkModule.h"

DkModule::DkModule() : releaseDebug(DK_RELEASE_IMGS) {
	
	imgs = 0;
	userDebugLevel = -1;

	// open log files
	wout.open(DkDebugStream::defaultLogFile);
	mout.open(DkDebugStream::defaultLogFile);
	iout.open(DkDebugStream::defaultLogFile);
	dout.open(DkDebugStream::defaultLogFile);

	className = "DkModule";

	wout = DkDebugStream("", DK_WARNING);
	mout = DkDebugStream("", DK_MODULE);
	iout = DkDebugStream("", DK_INFO);
	dout = DkDebugStream("", DK_DEBUG_INFO);

}

DkModule::DkModule(DkImageSource *imgs) : releaseDebug(DK_RELEASE_IMGS) {
	
	userDebugLevel = -1;
	
	// open log files
	wout.open(DkDebugStream::defaultLogFile);
	mout.open(DkDebugStream::defaultLogFile);
	iout.open(DkDebugStream::defaultLogFile);
	dout.open(DkDebugStream::defaultLogFile);

	className = "DkModule";

	wout = DkDebugStream("", DK_WARNING);
	mout = DkDebugStream("", DK_MODULE);
	iout = DkDebugStream("", DK_INFO);
	dout = DkDebugStream("", DK_DEBUG_INFO);

	this->imgs = imgs;
}

