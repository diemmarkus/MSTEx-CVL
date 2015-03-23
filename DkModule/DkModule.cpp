/***************************************************
 *   DkModule.cpp
 *   
 *   Created on: 05.07.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/

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

