/*******************************************************************************************************
DkMVModule.cpp
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

#include "DkMVModule.h"

#include "DkTimer.h"
#include "DkSegmentation.h"
#include "DkRandomTrees.h"
#include "DkGrabCut.h"
#include "DkAce.h"

//#include <winsock2.h>	// needed since libraw 0.16

#ifdef DK_STANDALONE
#include "opencv/highgui.h"
#endif

// DkMVModule --------------------------------------------------------------------
DkMVModule::DkMVModule(const std::wstring& folderName) {

	mFolderName = folderName;
}

bool DkMVModule::load() {

	DkTimer dt;

	std::vector<std::wstring> files = indexFolder(mFolderName);
	std::vector<cv::Mat> msImgs;

	for (const std::wstring& cFile : files) {

		std::wstring filePath = mFolderName + L"\\" + cFile;
		std::string fpStr(filePath.begin(), filePath.end());

		msImgs.push_back(DkIP::imread(fpStr, CV_LOAD_IMAGE_UNCHANGED));
		
		std::string fn(cFile.begin(), cFile.end());
		mout << fn << " loaded" << dkendl;
	}

	bool ok = setImages(files, msImgs);
	mout << "images loaded in: " << dt << "\n" << dkendl;
	
	return ok;

}

bool DkMVModule::setImages(const std::vector<std::wstring>& imgPaths, const std::vector<cv::Mat>& msImgs) {

	cv::Size lastSize;
	std::vector<cv::Mat> cleanedImgs;

	mout << "\nchecking input..." << dkendl;

	for (int idx = 0; idx < imgPaths.size(); idx++) {

		const std::wstring& cFile = imgPaths[idx];

		std::wstring filePath = mFolderName + L"\\" + cFile;
		std::string fpStr(filePath.begin(), filePath.end());

		cv::Mat img = msImgs[idx];

		// load mask ?!
		std::string cf(cFile.begin(), cFile.end());
		if (!img.empty() && cf.find("mask") != std::wstring::npos) {
			mMaskImg = img;

			if (!img.empty() && img.channels() > 1)
				cv::cvtColor(img, img, CV_RGB2GRAY);

			mout << cf << " -> is the mask" << dkendl;
		}
		else if (!img.empty()) {
			cleanedImgs.push_back(img);
		}
		else if (cFile.size() > 3) {	// > 3: ignore . and ..
			mout << cf << " ignored" << dkendl;
		}
	}

	// check dimensions
	int nR = 0, nC = 0;
	std::vector<size_t> remIdx;

	for (int idx = 0; idx < cleanedImgs.size(); idx++) {
		
		const cv::Mat& img = cleanedImgs[idx];

		if (nR == 0 || nC == 0) {
			nR = img.rows;
			nC = img.cols;
		}

		if (img.rows != nR || img.cols != nC) {
			remIdx.push_back(idx);
			std::string fn(imgPaths[idx].begin(), imgPaths[idx].end());
			mout << "illegal dimensions: " << img.rows << "x" << img.cols << " -> removing" << dkendl;
		}
	}

	// remove illegal images
	std::reverse(remIdx.begin(), remIdx.end());

	for (auto idx : remIdx) {
		cleanedImgs.erase(cleanedImgs.begin() + idx);
	}

	mImgs = DkMSData(cleanedImgs);

	mout << "The MSI image has " << cleanedImgs.size() << " channels" << dkendl;
	return true;
}

bool DkMVModule::saveImage(const std::string& imageName) const {

	cv::Mat sImg = mPseudoImg.clone();
	
	bool ok = false;
	if (!mPseudoImg.empty()) {
		sImg = DkIP::rotateImg(sImg, -CV_PI*0.5);	// we don't read the orientation flag correctely
		ok = DkIP::imwrite(imageName, sImg);
	}
	else
		mout << "Result image is empty - something went wrong when processing, sorry!" << dkendl;

	if (!ok)
		mout << "sorry, I could not write to: " << imageName << dkendl;

	return ok;
}

void DkMVModule::compute() {

	DkTimer dt;

	if (mMaskImg.empty()) {

		wout << "I cannot process the image - for the foreground mask is empty" << dkendl;
		mout << "please provide a binary image named mask where ink pixels are stored as white" << dkendl;
		wout << "aborting..." << dkendl;
		return;
	}

	cv::Mat fgdImg = mMaskImg;

	DkAce ace(mImgs, fgdImg);
	ace.compute();
	mPImg = ace.getPredictedImage();

	mPseudoImg = mPImg;

	iout << "[DkMVModule] computed in " << dt << dkendl;
}

DkMSData DkMVModule::getMSImages() const {

	return mImgs;
}

void DkMVModule::setPseudoColor(bool active) {
	pseudoCol = active;
}

cv::Mat DkMVModule::getPredictedImage() const {

	return mPImg;
}

cv::Mat DkMVModule::getColImg() const {
	return mPseudoImg;
}

std::vector<std::wstring> DkMVModule::indexFolder(const std::wstring& folderName) const {

	std::wstring folderIdxName = folderName + L"\\*.*";

	const wchar_t* fname = folderIdxName.c_str();


	WIN32_FIND_DATAW findFileData;
	HANDLE MyHandle = FindFirstFileW(fname, &findFileData);

	std::vector<std::wstring> fileNameList;
	std::wstring fileName;

	if (MyHandle != INVALID_HANDLE_VALUE) {

		do {

			fileName = findFileData.cFileName;
			fileNameList.push_back(fileName);
		} while (FindNextFileW(MyHandle, &findFileData) != 0);
	}
	else {
		std::wcout << "[Warning] " << folderName << " is not accessible or does not exist" << dkendl;
	}

	FindClose(MyHandle);

	//std::wcout << fileNameList.size() << " files indexed in " << mFolderName << dkendl;

	return fileNameList;
}