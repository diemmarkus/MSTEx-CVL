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

#include "DkMSModule.h"

#include "DkTimer.h"
#include "DkSegmentation.h"
#include "DkRandomTrees.h"
#include "DkGrabCut.h"
#include "DkAce.h"

//#include <winsock2.h>	// needed since libraw 0.16

#include "opencv/highgui.h"

// DkMSModule --------------------------------------------------------------------
DkMSModule::DkMSModule(const std::wstring& folderName) {

	this->folderName = folderName;
	strictInput = true;
}

void DkMSModule::load() {

	DkTimer dt;
	std::vector<std::wstring> files = indexFolder(folderName);
	std::vector<cv::Mat> msImgs;
	msImgs.resize(8);	// we expect 8 channels

	cv::Size lastSize;

	for (std::wstring cFile : files) {

		if (strictInput && cFile.find(L"F") == std::wstring::npos) {

			if (cFile.find(L".png") != std::wstring::npos)
				std::wcout << "[" << cFile << "] does not fit the specification - ignoring..." << std::endl;
			continue;
		}

		std::wstring filePath = folderName + L"\\" + cFile;
		std::string fpStr(filePath.begin(), filePath.end());

		cv::Mat img = cv::imread(fpStr);
		
		int chNum = getChannelNumber(cFile);

		if (!img.empty() && cFile.find(L"GT") == std::wstring::npos && chNum != -1) {
			
			if (lastSize.width != 0 && img.size() != lastSize) {
				mout << "image size does not match - ignoring..." << dkendl;
				continue;	
			}
			else if (lastSize.width == 0)
				lastSize = img.size();

			if (!msImgs[chNum].empty()) {
				std::string msg = "Image at channel  " + DkUtils::stringify(chNum) + " already exists!";
				throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
			}

			msImgs[chNum] = img;
			//std::wcout << cFile << " loaded into channel " << DkUtils::stringToWstring(DkUtils::stringify(chNum)) << std::endl;
		}
		else if (!img.empty()) {
			
			if (img.channels() > 1)
				cv::cvtColor(img, img, CV_RGB2GRAY);
			
			gtImg = img;
			//std::wcout << cFile << " GT loaded..." << std::endl;
		}
	}

	if (msImgs.size() != 8) {

		std::string msg = "Wrong number of input channels: " + DkUtils::stringify(msImgs.size()) + " [8 expected]";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}

	if (strictInput) {
		for (size_t idx = 0; idx < msImgs.size(); idx++) {
			if (msImgs[idx].empty()) {
				std::string msg = "Channel  " + DkUtils::stringify(idx+1) + " is empty! I need to abort sorry...";
				throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
			}
		}
	}
	imgs = DkMSData(msImgs);

	iout << "images loaded in: " << dt << dkendl;
}

bool DkMSModule::saveImage(const std::string& imageName) const {

	cv::Mat segImgInv = segImg.clone();
	DkIP::invertImg(segImgInv);
	bool ok = cv::imwrite(imageName, segImgInv);

	if (!ok)
		mout << "sorry, I could not write to: " << imageName << dkendl;

	return ok;
}

void DkMSModule::compute() {

	DkTimer dt;
	cv::Mat img = imgs.getVisChannel();
	cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(255));

	cv::Mat bImg = imgs.removeBackground(img, imgs.getBgChannel());

	// initial su segmentation
	DkSegmentationSu segM(bImg, mask);
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

DkMSData DkMSModule::getMSImages() const {

	return imgs;
}

cv::Mat DkMSModule::getPredictedImage() const {

	return pImg;
}

cv::Mat DkMSModule::getSegImg() const {

	return segImg;
}

cv::Mat DkMSModule::getGT() const {

	return gtImg;
}

int DkMSModule::getChannelNumber(const std::wstring& fileName) const {

	if (fileName.find_first_of(L"0123456789") == std::wstring::npos)
		return -1;

	// not elegant - but it does what it should
	// -1 because indexing (luckily) starts at 0
	if (fileName.find(L"1") != std::string::npos)
		return 0;
	else if (fileName.find(L"2") != std::string::npos)
		return 1;
	else if (fileName.find(L"3") != std::string::npos)
		return 2;
	else if (fileName.find(L"4") != std::string::npos)
		return 3;
	else if (fileName.find(L"5") != std::string::npos)
		return 4;
	else if (fileName.find(L"6") != std::string::npos)
		return 5;
	else if (fileName.find(L"7") != std::string::npos)
		return 6;
	else if (fileName.find(L"8") != std::string::npos)
		return 7;

	return -1;
}

std::vector<std::wstring> DkMSModule::indexFolder(const std::wstring& folderName) const {

	std::wstring folderIdxName = folderName + L"\\*.*";

	const wchar_t* fname = folderIdxName.c_str();

	
	WIN32_FIND_DATAW findFileData;
	HANDLE MyHandle = FindFirstFileW(fname, &findFileData);

	std::vector<std::wstring> fileNameList;
	std::wstring fileName;

	if( MyHandle != INVALID_HANDLE_VALUE) {

		do {

			fileName = findFileData.cFileName;
			fileNameList.push_back(fileName);
		} 
		while(FindNextFileW(MyHandle, &findFileData) != 0);
	}

	FindClose(MyHandle);

	//std::wcout << fileNameList.size() << " files indexed in " << folderName << std::endl;

	return fileNameList;
}