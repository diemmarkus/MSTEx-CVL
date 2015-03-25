/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/


#include "DkGrabCut.h"

DkGrabCut::DkGrabCut(const DkMSData& data, const cv::Mat& pImg, const cv::Mat& segSuImg) {

	this->data = data;
	this->pImg = pImg;
	this->segSuImg = segSuImg;
	className = "DkGrabCut";
}

void DkGrabCut::compute() {

	cv::Mat cImg = createColImg(data);
	cv::Mat mask = createMask(pImg);

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", cImg, true);
	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", mask, true);

	// apply grab cut on the whole image
	cv::Mat fgdModel;
	cv::Mat bgdModel;
	cv::Rect r(cv::Point(0,0), pImg.size());
	cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 5, GC_INIT_WITH_MASK);

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", mask, true);

	segImg = maskToBwImg(mask);

}

cv::Mat DkGrabCut::createColImg(const DkMSData& data) const {

	// create color image
	std::vector<cv::Mat> cImgs;
	cv::Mat pImg8U;
	pImg.convertTo(pImg8U, CV_8UC1, 255.0f);
	cImgs.push_back(pImg8U);
	cImgs.push_back(data.getVisChannel());
	cImgs.push_back(segSuImg);

	cv::Mat cImg(cImgs[0].size(), CV_8UC3);
	cv::merge(cImgs, cImg);

	return cImg;
}

cv::Mat DkGrabCut::createMask(const cv::Mat& pImg) const {

	// create the mask
	cv::Mat mask(pImg.size(), CV_8UC1);

	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

		unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);
		const unsigned char* sPtr = segSuImg.ptr<unsigned char>(rIdx);
		const float* pPtr = pImg.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {

			if (pPtr[cIdx] > 0.95f && sPtr[cIdx] == 255)
				mPtr[cIdx] = GC_FGD;
			else if (pPtr[cIdx] > 0.5f)
				mPtr[cIdx] = GC_PR_FGD;
			else if (pPtr[cIdx] < 0.1f && sPtr[cIdx] == 0)
				mPtr[cIdx] = GC_BGD;
			else
				mPtr[cIdx] = GC_PR_BGD;
		}
	}

	return mask;
}

cv::Mat DkGrabCut::maskToBwImg(const cv::Mat& mask) const {

	cv::Mat segImg = cv::Mat(mask.size(), CV_8UC1);

	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

		const unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);
		unsigned char* sPtr = segImg.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {

			if (mPtr[cIdx] == GC_FGD || mPtr[cIdx] == GC_PR_FGD)
				sPtr[cIdx] = 255;
			else
				sPtr[cIdx] = 0;
		}
	}

	return segImg;
}

cv::Mat DkGrabCut::getSegImg() const {

	return segImg;
}

std::string DkGrabCut::toString() const {

	std::string msg;
	msg += "[" + className + "]";

	return msg;
}
