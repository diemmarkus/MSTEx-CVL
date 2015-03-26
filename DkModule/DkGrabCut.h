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

class DK_MODULE_API DkGrabCut : public DkModule {

public:
	DkGrabCut(const DkMSData& data, const cv::Mat& pImg, const cv::Mat& segSuImg);

	void compute();
	cv::Mat getSegImg() const;

	std::string toString() const;

protected:
	DkMSData data;
	cv::Mat pImg;	// probability image
	cv::Mat segImg;
	cv::Mat segSuImg;

	void checkInput() const {};		// dummy
	cv::Mat createColImg(const DkMSData& data) const;
	cv::Mat createMask(const cv::Mat& pImg) const;
	cv::Mat maskToBwImg(const cv::Mat& mask) const;
	bool refineMask(const cv::Mat& mask, cv::Mat& pImg) const;
};