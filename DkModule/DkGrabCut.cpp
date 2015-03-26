/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/


#include "DkGrabCut.h"

#include "DkMath.h"
#include "DkBlobs.h"
#include "DkTimer.h"

DkGrabCut::DkGrabCut(const DkMSData& data, const cv::Mat& pImg, const cv::Mat& segSuImg) {

	this->data = data;
	this->pImg = pImg;
	this->segSuImg = segSuImg;
	className = "DkGrabCut";
}

void DkGrabCut::compute() {

	DkTimer dt;
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
	
	DkTimer dtg;
	cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 1, GC_INIT_WITH_MASK);
	mout << "grab cut takes: " << dtg << dkendl;

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", mask, true);


	for (int idx = 0; idx < 100; idx++) {

		cv::Mat bwMask = maskToBwImg(mask);

		if (!refineMask(bwMask, pImg)) {
			mout << "optimum reached after " << idx << " iterations" << dkendl;
			break;
		}
		
		mask = createMask(pImg);
		cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 1, GC_EVAL);
	
		if (releaseDebug == DK_SAVE_IMGS)
			DkIP::imwrite(className + DkUtils::stringify(__LINE__) + "-" + DkUtils::stringify(idx) + ".png", mask, true);

		mout << "iterating..." << dkendl;
	}

	cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 4, GC_EVAL);

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + "-final.png", mask, true);

	segImg = maskToBwImg(mask);

	mout << "[" << className << "] computed in " << dt << dkendl;
}

bool DkGrabCut::refineMask(const cv::Mat& maskImg, cv::Mat& pImg) const {

	DkTimer dt;

	cv::Mat mask = maskImg.clone();
	cv::Mat segImg = segSuImg.clone();
	segImg = segImg > 0;
	segImg.convertTo(segImg, CV_32F, 1.0f/255.0f);

	double nPos = cv::sum(segImg)[0];

	for (int idx = 0; idx < 100; idx++) {
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
		cv::erode(mask, mask, element);
		//cv::dilate(mask, mask, element);

		if (!cv::sum(mask == 255)[0]) {
			moutc << "image empty after " << idx << " iterations" << dkendl;
			return false;
		}

		cv::Mat chkImg = mask.clone();
		chkImg.convertTo(chkImg, CV_32F, 1.0f/255.0f);
		chkImg = segImg+chkImg;

		//DkIP::imwrite("chkImg" + DkUtils::stringify(idx) + ".png", chkImg, true);

		if ((cv::sum(chkImg == 2.0f)[0]/2.0f)/nPos < 5.0) {
			moutc << "non-overlapping reached after " << idx << " iterations" << dkendl;
			break;
		}
		else
			moutc << "too many overlaps: " << ((cv::sum(chkImg == 2.0f)[0]/2.0f)/nPos) << dkendl;
	}

	mask = mask > 0 & segSuImg == 0;
	DkIP::invertImg(mask);

	if (!cv::sum(mask == 0)[0])
		return false;
	
	cv::Mat chkImg = pImg.clone();
	DkIP::mulMask(pImg, mask);	// remove eroded image from pImg
	moutc << "mask refined in " << dt << dkendl;

	return true;
}

cv::Mat DkGrabCut::createColImg(const DkMSData& data) const {

	// create color image
	std::vector<cv::Mat> cImgs;
	cv::Mat pImg8U;
	pImg.convertTo(pImg8U, CV_8UC1, 255.0f);
	cImgs.push_back(255-pImg8U);
	cImgs.push_back(data.removeBackground(data.getVisChannel(), data.getBgChannel()));
	cImgs.push_back(data.getBgChannel());

	cv::Mat cImg(cImgs[0].size(), CV_8UC3);
	cv::merge(cImgs, cImg);

	return cImg;
}

cv::Mat DkGrabCut::createMask(const cv::Mat& pImg) const {

	// create the mask
	cv::Mat mask(pImg.size(), CV_8UC1);

	//DkIP::imwrite("fgdGrabCut.png", segSuImg);
	//cv::Mat segSuSkel = DkIP::skeleton(segSuImg == 255);
	//DkIP::imwrite("fgdGrabCutSkel.png", segSuSkel);

	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

		unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);
		const unsigned char* sPtr = segSuImg.ptr<unsigned char>(rIdx);
		const float* pPtr = pImg.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {

			if (pPtr[cIdx] > 0.95f && sPtr[cIdx] == 255)
				mPtr[cIdx] = GC_FGD;
			else if (pPtr[cIdx] < 0.1f && sPtr[cIdx] == 0)
				mPtr[cIdx] = GC_BGD;
			else if (pPtr[cIdx] > 0.8f)
				mPtr[cIdx] = GC_PR_FGD;
			else if (pPtr[cIdx] < 0.4f)
				mPtr[cIdx] = GC_PR_BGD;
			else if (sPtr[cIdx] == 0)	// fallback to foreground
				mPtr[cIdx] = GC_BGD;
			else if (sPtr[cIdx] == 255)
				mPtr[cIdx] = GC_PR_FGD;
			else
				mPtr[cIdx] = GC_PR_FGD;

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
