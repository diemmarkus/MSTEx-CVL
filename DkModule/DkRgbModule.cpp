/*******************************************************************************************************
 DkMSModule.cpp
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

#include "DkRgbModule.h"

#include "DkTimer.h"
#include "DkSegmentation.h"
#include "DkRandomTrees.h"
#include "DkGrabCut.h"
#include "DkAce.h"

//#include <winsock2.h>	// needed since libraw 0.16

#include "opencv/highgui.h"

// DkMSModule --------------------------------------------------------------------
DkRgbModule::DkRgbModule(const std::wstring& fileName) {

	this->fileName = fileName;
}

void DkRgbModule::load() {

	DkTimer dt;
	std::vector<cv::Mat> msImgs;

	cv::Size lastSize;

	std::wstring filePath = fileName;
	std::string fpStr(filePath.begin(), filePath.end());

	cv::Mat img = cv::imread(fpStr);
	cv::split(img, msImgs);

	if (msImgs.size() < 3)
		wout << "The image has less than 3 channels - note that this method was NOT designed for gray-scale images." << dkendl;

	if (msImgs.empty())
		wout << "Error: could not load " << fpStr << dkendl;

	imgs = DkMSData(msImgs);
	iout << "images loaded in: " << dt << dkendl;
}

void DkRgbModule::compute() {

	DkTimer dt;
	cv::Mat img = imgs.getVisChannel();
	cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(255));

	// initial su segmentation
	DkSegmentationSu segM(img, mask);
	segM.compute();
	segM.filterSegImg(20);
	segSuImg = segM.getSegmented();
	cv::Mat fgdImg = imgs.estimateFgd(segSuImg);

	iout << "image segmented in: " << dt << dkendl;

	//// DkRandomTrees is an alternative to the ACE
	//DkRandomTrees rt(imgs, fgdImg);
	//rt.compute();
	//cv::Mat pImgRT = rt.getPredictedImage();
	//pImg = pImgRT;
	//DkIP::imwrite("pImg-RT.png", pImg);

	DkAce ace(imgs, fgdImg);
	ace.compute();
	pImg = ace.getPredictedImage();
	
	fgdImg = imgs.removeBackgroundBlobs(segSuImg);

	// grab cut
	DkGrabCut gb(imgs, pImg, fgdImg, false);
	gb.compute();

	segImg = gb.getSegImg();

	iout << "[DkMSModule] computed in " << dt << dkendl;
}

DkMSData DkRgbModule::getMSImages() const {

	return imgs;
}

cv::Mat DkRgbModule::getPredictedImage() const {

	return pImg;
}

cv::Mat DkRgbModule::getSegImg() const {

	return segImg;
}

