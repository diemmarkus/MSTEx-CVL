/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/


#include "DkAce.h"

#include "DkMath.h"
#include "DkBlobs.h"
#include "DkTimer.h"

DkAce::DkAce(const DkMSData& data, const cv::Mat& fgdImg) {

	this->msData = data;
	this->fgdImg = fgdImg;
	className = "DkAce";
}

void DkAce::compute() {

	DkTimer dt;

	cv::Mat sig = msData.convertToSignal();
	cv::Mat tS = getTargetSignature(sig, fgdImg);
	sig.convertTo(sig, CV_32FC1, 1.0f/255.0f);

	cv::Mat meanVec;
	sig = removeMean(sig, meanVec);
	tS = removeMean(tS, meanVec);

	pImg = hyperAce(sig, tS);

	mout << "[" << className << "] computed in " << dt << dkendl;
}

cv::Mat DkAce::removeMean(const cv::Mat& data, cv::Mat& meanVec) const {

	cv::Mat nData(data.size(), CV_32FC1);

	if (meanVec.empty()) {
		meanVec = cv::Mat(1, data.cols, CV_32FC1, cv::Scalar(0));

		float* meanPtr = meanVec.ptr<float>();

		for (int rIdx = 0; rIdx < data.rows; rIdx++) {
			const float* dPtr = data.ptr<float>(rIdx);

			for (int cIdx = 0; cIdx < data.cols; cIdx++) {
				meanPtr[cIdx] += dPtr[cIdx];
			}
		}

		meanVec /= (float)data.rows;
	}

	float* mPtr = meanVec.ptr<float>();

	for (int rIdx = 0; rIdx < data.rows; rIdx++) {
		const float* dPtr = data.ptr<float>(rIdx);
		float* ndPtr = nData.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < data.cols; cIdx++) {
			ndPtr[cIdx] = dPtr[cIdx]-mPtr[cIdx];
		}
	}

	return nData;
}

cv::Mat DkAce::hyperAce(const cv::Mat& data, const cv::Mat& signature) const {

	DkTimer dt;

	cv::Mat meanVec;
	cv::Mat sig64, data64;

	signature.convertTo(sig64, CV_64FC1);
	data.convertTo(data64, CV_64FC1);

	// compute inverted covariance matrix
	cv::Mat cov, icov, meanC;
	cov = (data64.t()*data64)/(float)(data.rows-1);
	cv::invert(cov, icov, DECOMP_SVD);

	// pre-compute
	sig64 = sig64.t();

	double tmp = *(cv::Mat(sig64.t()*icov*sig64).ptr<double>());
	cv::Mat icData = data64*icov*sig64;
	cv::Mat om(data64.cols, 1, CV_64FC1, cv::Scalar(1));
	cv::Mat xInv = (data64*icov).mul(data64) * om;
	
	cv::Mat rImg = icData.mul(abs(icData));
	rImg /= (tmp * xInv);
		
	double* rPtr = rImg.ptr<double>();
	
	for (int rIdx = 0; rIdx < rImg.rows; rIdx++) {

		if (rPtr[rIdx] < 0 || rPtr[rIdx] > 1.0)
			rPtr[rIdx] = (rPtr[rIdx] < 0) ? 0 : 1.0 ;
	}

	rImg = msData.columnVectorToImage(rImg);
	cv::normalize(rImg, rImg, 1.0f, 0.0f, NORM_MINMAX);
	rImg.convertTo(rImg, CV_32FC1);

	moutc << "ACE computed in " << dt << dkendl;

	return rImg;
}

cv::Mat DkAce::getTargetSignature(const cv::Mat& data, const cv::Mat& fgdImg) const {

	
	cv::Mat signature(1, data.cols, CV_32FC1, cv::Scalar(0));
	cv::Mat labels = msData.imageToColumnVector(fgdImg);

	const unsigned char* labelPtr = labels.ptr<unsigned char>();
	float* sigPtr = signature.ptr<float>();
	int sumPos = 0;

	for (int rIdx = 0; rIdx < data.rows; rIdx++) {

		if (labelPtr[rIdx] > 0) {		// TODO: check with labelPtr == 255

			const unsigned char* dataPtr = data.ptr<unsigned char>(rIdx);
			sumPos++;

			for (int cIdx = 0; cIdx < data.cols; cIdx++) {

				sigPtr[cIdx] += dataPtr[cIdx]/255.0f;
			}
		}
	}

	signature /= (float)sumPos;

	return signature;
}

cv::Mat DkAce::getPredictedImage() const {

	return pImg;
}

std::string DkAce::toString() const {

	std::string msg;
	msg += "[" + className + "]";

	return msg;
}