/*******************************************************************************************************
 DkMSModule.h
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
	void setImages(const std::vector<std::wstring>& imgPaths, const std::vector<cv::Mat>& msImgs);
	void compute();

	void setPseudoColor(bool active);

	cv::Mat getPredictedImage() const;
	cv::Mat getSegImg() const;
	cv::Mat getColImg() const;
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
	cv::Mat colImg;		// visualized segmentation image

	bool strictInput;
	bool pseudoCol;

	int getChannelNumber(const std::wstring& fileName) const;
};
