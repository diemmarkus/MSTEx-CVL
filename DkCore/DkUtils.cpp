/***************************************************
 *   DkUtils.cpp
 *   
 *   Created on: 09.03.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/

#include "DkUtils.h"


int DkUtils::debugLevel = DK_MODULE;

cv::Scalar DkUtils::blue = cv::Scalar(130, 161, 206);
cv::Scalar DkUtils::blueDark = cv::Scalar(95, 116, 135);
cv::Scalar DkUtils::yellow = cv::Scalar(255, 210, 0);

std::string DkDebugStream::defaultLogFile = "";		// no default logfile

DkDebugStream DebugResources::wout_p = DkDebugStream("", DK_WARNING);
DkDebugStream DebugResources::mout_p = DkDebugStream("", DK_MODULE);
DkDebugStream DebugResources::iout_p = DkDebugStream("", DK_INFO);
DkDebugStream DebugResources::dout_p = DkDebugStream("", DK_DEBUG_INFO);
