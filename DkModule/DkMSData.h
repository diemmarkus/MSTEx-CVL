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
