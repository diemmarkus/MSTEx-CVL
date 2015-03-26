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


	//// TODO: 
	//// the idea is to remove growing potential foreground elements
	//// and then iteratively perform grabcut
	//for (int idx = 0; idx < 10; idx++) {


	//	cv::Mat segImgCleaned = mask == GC_FGD | mask == GC_PR_FGD;
	//	DkBlobs<DkAttr> segFilter(segImgCleaned);
	//	segFilter.calcArea();

	//	std::list<float> areas;

	//	for (int i = 0; i < segFilter.getSize(); i++) {
	//		areas.push_back((float)abs(segFilter.getBlob(i).getArea()));
	//	}

	//	double q25 = DkMath::statMoment<float>(&areas, .25f);
	//	double q75 = DkMath::statMoment<float>(&areas, .75f);
	//	double ob = q75 + 1.5*(q75-q25);

	//	mout << "max area: " << ob << " q25: " << q25 << " q75: " << q75 << dkendl;
	//	
	//	segImgCleaned = mask == GC_PR_FGD;
	//	cv::imwrite("imgBeforeCleaning.png", segImgCleaned);

	//	DkBlobs<DkAttr> sF(segImgCleaned);
	//	segFilter.imgFilterArea(0, cvRound(ob));

	//	cv::imwrite("imgCleaned.png", segImgCleaned);

	//	bool done = true;

	//	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

	//		const unsigned char* sPtr = segImgCleaned.ptr<unsigned char>(rIdx);
	//		unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);

	//		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {
	//			
	//			if (mPtr[cIdx] == GC_PR_FGD && sPtr[cIdx] == 0) {
	//				mPtr[cIdx] = GC_PR_BGD;
	//				done = false;
	//			}
	//		}
	//	}

	//	if (done)
	//		break;

	//	cv::grabCut(cImg, mask, r, bgdModel, fgdModel, 5, GC_EVAL);

	//	mout << "iter: " << idx << dkendl;
	//}

	if (releaseDebug == DK_SAVE_IMGS)
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", mask, true);

	segImg = maskToBwImg(mask);

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

	for (int rIdx = 0; rIdx < mask.rows; rIdx++) {

		unsigned char* mPtr = mask.ptr<unsigned char>(rIdx);
		const unsigned char* sPtr = segSuImg.ptr<unsigned char>(rIdx);
		const float* pPtr = pImg.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < mask.cols; cIdx++) {

			// TODO!!!!!
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
				mPtr[cIdx] = GC_FGD;
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
