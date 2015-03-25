/***************************************************
 *   DkSegmentation.h
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/


#pragma once

#include "DkModuleInclude.h"

#include "DkUtils.h"

class DK_MODULE_API DkMSData {

public:
	DkMSData(const std::vector<cv::Mat>& data = std::vector<cv::Mat>());

	cv::Mat convertToSignal() const;
	std::vector<cv::Mat> getImages() const;

	cv::Mat getVisChannel() const;
	cv::Mat getBgChannel() const;

	cv::Mat imageToColumnVector(const cv::Mat& img) const;
	cv::Mat columnVectorToImage(const cv::Mat& vec) const;

	cv::Mat removeSensorNoise(const cv::Mat& img) const;

protected:
	std::vector<cv::Mat> msImgs;
	void fixInput(std::vector<cv::Mat>& imgs) const;

};
