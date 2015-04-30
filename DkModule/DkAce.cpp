/*******************************************************************************************************
 DkAce.cpp
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

	cv::Mat sig = msData.getSignal();
	cv::Mat tS = getTargetSignature(sig, fgdImg);
	sig.convertTo(sig, CV_32FC1, 1.0f/255.0f);

	cv::Mat meanVec;
	sig = removeMean(sig, meanVec);
	tS = removeMean(tS, meanVec);

	pImg = hyperAce(sig, tS);

	iout << "[" << className << "] computed in " << dt << dkendl;
}

/**
 * Returns the mean normalized data.
 * @param data the raw signal
 * @param meanVec the mean vector (if empty it's estimated)
 * @return cv::Mat the mean normalized data.
 **/ 
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

/**
 * Computes the ACE transformation
 * @param data the raw signal NxM where M is the number of channels.
 * @param signature the mean signature @see getTargetSignature.
 * @return cv::Mat the transformed image
 **/ 
cv::Mat DkAce::hyperAce(const cv::Mat& data, const cv::Mat& signature) const {

	DkTimer dt;

	cv::Mat meanVec;
	cv::Mat sig64, data64;

	signature.convertTo(sig64, CV_64FC1);
	data.convertTo(data64, CV_64FC1);

	// compute inverted covariance matrix
	cv::Mat cov, icov, meanC;
	cov = (data64.t()*data64)/(float)(data64.rows-1);
	cv::invert(cov, icov, DECOMP_SVD);

	// pre-compute
	sig64 = sig64.t();

	// apply ACE - sorry I know it's hard to read, but this implementation is fast
	double tmp = *(cv::Mat(sig64.t()*icov*sig64).ptr<double>());
	cv::Mat icData = data64*icov*sig64;
	cv::Mat om(data64.cols, 1, CV_64FC1, cv::Scalar(1));	// needed for sum
	cv::Mat xInv = (data64*icov).mul(data64) * om;
	
	cv::Mat rImg = icData.mul(abs(icData));
	rImg /= (tmp * xInv);
	
	// crop to [0 1]
	double* rPtr = rImg.ptr<double>();
	for (int rIdx = 0; rIdx < rImg.rows; rIdx++) {

		if (rPtr[rIdx] < 0 || rPtr[rIdx] > 1.0)
			rPtr[rIdx] = (rPtr[rIdx] < 0) ? 0 : 1.0 ;
	}

	// convert to our format
	rImg = msData.columnVectorToImage(rImg);
	cv::normalize(rImg, rImg, 1.0f, 0.0f, NORM_MINMAX);	// just for safety
	rImg.convertTo(rImg, CV_32FC1);

	ioutc << "ACE computed in " << dt << dkendl;

	return rImg;
}

/**
 * Computes the mean value of each foreground pixel in each channel.
 * @param data the raw signal
 * @param fgdImg a BW image 255 == foreground
 * @return cv::Mat a 1x8 (channels) vector with all mean values
 **/ 
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
