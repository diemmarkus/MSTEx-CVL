/*******************************************************************************************************
 DkRandomTrees.h
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

#include "DkUtils.h"
#include "DkMSData.h"
#include "DkModule.h"

#ifdef DK_STANDALONE
#include "ml.h"

class DK_MODULE_API DkRandomTrees : public DkModule {

public:
	DkRandomTrees(const DkMSData& data = DkMSData(), const cv::Mat&  suImg = cv::Mat());

	void compute();
	std::string toString() const;

	cv::Mat getPredictedImage() const;

protected:
	DkMSData imgs;
	cv::Mat suImg;	// label input
	cv::Mat pImg;	// result
	int nSamples;

	void checkInput() const;
	cv::Ptr<cv::ml::RTrees> trainOnline(const cv::Mat& data, const cv::Mat& fgdImg) const;
	cv::Mat predictImage(const cv::Mat& data, const cv::Ptr<cv::ml::RTrees>& classifier) const;
	void converData(cv::Mat& trainData, cv::Mat& labels, int numPos) const;
};

#endif
