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

DkGrabCut::DkGrabCut(const DkMSData& data, const cv::Mat& pImg, const cv::Mat& segSuImg, bool isVotingRT) {

	this->data = data;
	this->pImg = pImg;
	this->segSuImg = segSuImg;
	this->isVotingRT = isVotingRT;
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

		DkTimer dt;
		cv::Mat bwMask = maskToBwImg(mask);

		// is optimum reached? (no eroded areas outside su image)
		if (!refineMask(bwMask, pImg)) {
			break;
		}

		mask = createMask(pImg);
		cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 1, GC_EVAL);

		mout << "grab cut refined in " << dt << dkendl;
	}

	cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 1, GC_EVAL);

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + "-final.png", mask, true);

	segImg = maskToBwImg(mask);

	mout << "[" << className << "] computed in " << dt << dkendl;
}

/**
 * Refines the mask image
 * This method erodes the BW image until only 5% overlap with the Su image
 * is reached. If the image has non-zero pixels at this time, we know
 * that there are blobs which violate the Su stroke criterion. Hence,
 * we set these pixels as definite background in the grabcut.
 * @param maskImg the current BW mask
 * @param pImg the probability image - violating pixel are set to 0.
 * @return bool true if there were violiting pixel found
 **/ 
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

		// if the image is empty - we have reached the optiumum -> stop
		if (!cv::sum(mask == 255)[0]) {
			return false;
		}

		cv::Mat chkImg = mask.clone();
		chkImg.convertTo(chkImg, CV_32F, 1.0f/255.0f);
		chkImg = segImg+chkImg;

		// if overlap is less than 5% with su image stop
		if ((cv::sum(chkImg == 2.0f)[0]/2.0f)/nPos < 5.0) {
			break;
		}
	}

	// combine su & current mask
	mask = mask > 0 & segSuImg == 0;
	DkIP::invertImg(mask);

	if (!cv::sum(mask == 0)[0])
		return false;
	
	cv::Mat chkImg = pImg.clone();
	DkIP::mulMask(pImg, mask);	// remove eroded image from pImg
	ioutc << "mask refined in " << dt << dkendl;

	return true;
}

/**
 * Create color image for grab cut.
 * The color image is combined from the cleaned
 * vis channel (F2), the mean and the std of all channels.
 * @param data 
 * @return cv::Mat
 **/ 
cv::Mat DkGrabCut::createColImg(const DkMSData& data) const {
	
	cv::Mat signal = data.convertToSignal();
	cv::Mat meanImg(signal.rows, 1, CV_8UC1);
	cv::Mat stdImg(signal.rows, 1, CV_8UC1);

	unsigned char* mPtr = meanImg.ptr<unsigned char>();
	unsigned char* sPtr = stdImg.ptr<unsigned char>();

	for (int rIdx = 0; rIdx < signal.rows; rIdx++) {

		double ms = 0;
		double ss = 0;
		const unsigned char* sigPtr = signal.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < signal.cols; cIdx++) {

			ms += (double)sigPtr[cIdx];
			ss += (double)sigPtr[cIdx]*(double)sigPtr[cIdx];
		}

		double meanV = ms/signal.cols;
		double stdV = sqrt(ss/signal.cols - (meanV*meanV));

		mPtr[rIdx] = DkMath::cropToUChar((float)meanV);
		sPtr[rIdx] = DkMath::cropToUChar((float)stdV);
	}

	meanImg = data.columnVectorToImage(meanImg);
	stdImg = data.columnVectorToImage(stdImg);

	cv::normalize(meanImg, meanImg, 255, 0, NORM_MINMAX);
	cv::normalize(stdImg, stdImg, 255, 0, NORM_MINMAX);

	// create color image
	std::vector<cv::Mat> cImgs;
	
	cImgs.push_back(data.removeBackground(data.getVisChannel(), data.getBgChannel()));
	cImgs.push_back(meanImg);
	cImgs.push_back(stdImg);

	cv::Mat cImg(cImgs[0].size(), CV_8UC3);
	cv::merge(cImgs, cImg);

	return cImg;
}

cv::Mat DkGrabCut::createMask(const cv::Mat& pImg) const {

	// create the mask
	cv::Mat mask(pImg.size(), CV_8UC1);

	float fgdThresh, bgdThresh;

	if (!isVotingRT) {
		fgdThresh = 0.3f;
		bgdThresh = 0.1f;
	}
	else {
		fgdThresh = 0.95f;
		bgdThresh = 0.1f;
	}

	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

		unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);
		const unsigned char* sPtr = segSuImg.ptr<unsigned char>(rIdx);
		const float* pPtr = pImg.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {

			if (pPtr[cIdx] > fgdThresh && sPtr[cIdx] == 255)
				mPtr[cIdx] = GC_FGD;
			else if (pPtr[cIdx] <= bgdThresh && sPtr[cIdx] == 0)
				mPtr[cIdx] = GC_BGD;
			else if (pPtr[cIdx] > bgdThresh && sPtr[cIdx] == 255)
				mPtr[cIdx] = GC_PR_FGD;
			else if (pPtr[cIdx] <= fgdThresh && sPtr[cIdx] == 0)
				mPtr[cIdx] = GC_PR_BGD;
			else if (sPtr[cIdx] == 0)	// fallback to su
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

void DkGrabCut::setPChannel(const cv::Mat& pImgRT) {
	this->pImgRT = pImgRT;
}

cv::Mat DkGrabCut::getSegImg() const {

	return segImg;
}

std::string DkGrabCut::toString() const {

	std::string msg;
	msg += "[" + className + "]";

	return msg;
}
