/*******************************************************************************************************
 DkMSData.h
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

class DK_MODULE_API DkMSData {

public:
	DkMSData(const std::vector<cv::Mat>& data = std::vector<cv::Mat>());

	std::vector<cv::Mat> getImages() const;
	cv::Mat getSignal() const;

	cv::Mat getVisChannel() const;
	cv::Mat getBgChannel() const;

	cv::Mat imageToColumnVector(const cv::Mat& img) const;
	cv::Mat columnVectorToImage(const cv::Mat& vec) const;

	cv::Mat removeSensorNoise(const cv::Mat& img) const;
	cv::Mat removeBackground(const cv::Mat& img, const cv::Mat& bgImg) const;
	cv::Mat estimateFgd(const cv::Mat& bwImg) const;
	cv::Mat removeBackgroundBlobs(const cv::Mat& bwImg) const;

protected:
	std::vector<cv::Mat> msImgs; // the data
	cv::Mat signal;				 // the corresponding signal
	
	void fixInput(std::vector<cv::Mat>& imgs) const;
	cv::Mat convertToSignal() const;

};
