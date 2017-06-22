/*******************************************************************************************************
 DkAce.h
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

#pragma once

#include "DkModuleInclude.h"
#include "DkModule.h"
#include "DkMSData.h"

class DK_MODULE_API DkAce : public DkModule {

public:
	DkAce(const DkMSData& data, const cv::Mat& fgdImg);

	void compute();
	cv::Mat getPredictedImage() const;

	std::string toString() const;

protected:
	DkMSData msData;
	cv::Mat pImg;	// probability (result) image
	cv::Mat fgdImg;

	void checkInput() const {};		// dummy
	cv::Mat getTargetSignature(const cv::Mat& data, const cv::Mat& fgdImg) const;
	cv::Mat removeMean(const cv::Mat& data, cv::Mat& meanVec) const;
	cv::Mat hyperAce(const cv::Mat& data, const cv::Mat& signature) const;
	cv::Mat computeBlock(const cv::Mat& data64, const cv::Mat& icov, const cv::Mat& sig64) const;
};