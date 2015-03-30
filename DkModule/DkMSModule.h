/**************************************************
 * 	DkMSModule.h
 *
 *	Created on:	23.03.2015
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once;

#include "DkModuleInclude.h"

#include "DkUtils.h"
#include "DkMSData.h"

#include <string>
#include <vector>

class DK_MODULE_API DkMSModule {

public:
	DkMSModule(const std::wstring& folderName);

	void load();
	void compute();

	cv::Mat getPredictedImage() const;
	cv::Mat getSegImg() const;
	DkMSData getMSImages() const;
	cv::Mat getGT() const;

	std::vector<std::wstring> indexFolder(const std::wstring& folderName) const;
	bool saveImage(const std::string& imageName) const;

protected:
	std::wstring folderName;
	DkMSData imgs;
	cv::Mat segSuImg;	// Su binary image
	cv::Mat segImg;		// result image
	cv::Mat gtImg;		// optional
	cv::Mat pImg;		// predicted image

	bool strictInput;

	int getChannelNumber(const std::wstring& fileName) const;
};
