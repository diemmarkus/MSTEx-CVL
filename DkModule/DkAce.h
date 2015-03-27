/***************************************************
 *   DkSegmentation.h
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/


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
	cv::Mat pImg;	// probability image
	cv::Mat fgdImg;

	void checkInput() const {};		// dummy
	cv::Mat getTargetSignature(const cv::Mat& data, const cv::Mat& fgdImg) const;
	cv::Mat removeMean(const cv::Mat& data, cv::Mat& meanVec) const;
	cv::Mat hyperAce(const cv::Mat& data, const cv::Mat& signature) const;
};