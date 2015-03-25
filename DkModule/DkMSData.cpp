/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/

#include "DkMSData.h"

#include "DkTimer.h"
#include "DkIP.h"

DkMSData::DkMSData(const std::vector<cv::Mat>& data) {

	msImgs = data;
	fixInput(msImgs);
}

void DkMSData::fixInput(std::vector<cv::Mat>& imgs) const {

	for (cv::Mat& img : imgs) {

		if (img.channels() > 1)
			cv::cvtColor(img, img, CV_RGB2GRAY);

		img = removeSensorNoise(img);

		//if (img.depth() != CV_32F)
		//	img.convertTo(img, CV_32F, 1.0f/255.0f);
	}
}

std::vector<cv::Mat> DkMSData::getImages() const {

	return msImgs;
}

cv::Mat DkMSData::getVisChannel() const {

	if (!msImgs.empty() && msImgs.size() > 1)
		return msImgs[1];

	return cv::Mat();
}

cv::Mat DkMSData::getBgChannel() const {

	if (!msImgs.empty())
		return msImgs[msImgs.size()-1];	// bg channel is the last channel

	return cv::Mat();
}

/**
 * Converts all channels to a NxC matrix.
 * Where N is the image size (rows x cols) and C
 * is the number of channels.
 * Hence, each row represents a pixel.
 * @return cv::Mat the data matrix
 **/ 
cv::Mat DkMSData::convertToSignal() const {

	if (msImgs.empty())
		return cv::Mat();

	DkTimer dt;
	cv::Mat signal(msImgs[0].rows * msImgs[0].cols, (int)msImgs.size(), msImgs[0].depth(), cv::Scalar(42));

	for (int rIdx = 0; rIdx < msImgs.size(); rIdx++) {
		imageToColumnVector(msImgs.at(rIdx)).copyTo(signal.col(rIdx));
	}

	mout << "signal converted in " << dt << dkendl;

	return signal;
}

cv::Mat DkMSData::removeSensorNoise(const cv::Mat& img) const {

	cv::Mat mImg;
	cv::medianBlur(img, mImg, 5);	// remove small speckles

	double minV, maxV;
	cv::minMaxLoc(mImg, &minV, &maxV);

	unsigned char minVC = (unsigned char)minV;
	unsigned char maxVC = (unsigned char)maxV;

	// nothing to do?
	if (minVC == 0 && maxVC == 255)
		return img.clone();

	cv::Mat resImg = img.clone();

	unsigned char mVal = (unsigned char)cv::mean(img)[0];

	// crop image values (outside the bounds)
	for (int rIdx = 0; rIdx < resImg.rows; rIdx++) {

		unsigned char* rPtr = resImg.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < resImg.cols; cIdx++) {

			if (rPtr[cIdx] > maxVC)
				rPtr[cIdx] = maxVC;
			else if (rPtr[cIdx] < minVC)
				rPtr[cIdx] = minVC;

		}
	}
	
	cv::normalize(resImg, resImg, 255.0f, 0.0f, NORM_MINMAX);

	return resImg;
}

cv::Mat DkMSData::removeBackground(const cv::Mat& img, const cv::Mat& bgImg) const {

	if (img.size() != bgImg.size()) {
		woutc << "[WARNING] in removeBackground - image size does not correspond..." << dkendl;
		return img.clone();
	}

	cv::Mat rImg(img.size(), CV_16SC1);
	//cv::absdiff(img, bgImg, rImg);
	//rImg = 255-rImg;

	// crop image values (outside the bounds)
	for (int rIdx = 0; rIdx < rImg.rows; rIdx++) {

		short* rPtr = rImg.ptr<short>(rIdx);
		const unsigned char* iPtr = img.ptr<unsigned char>(rIdx);
		const unsigned char* bPtr = bgImg.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < rImg.cols; cIdx++) {

			//rPtr[cIdx] = (iPtr[cIdx] > bPtr[cIdx]) ? iPtr[cIdx]-bPtr[cIdx] : iPtr[cIdx];
			rPtr[cIdx] = iPtr[cIdx]-bPtr[cIdx];
		}
	}

	cv::normalize(rImg, rImg, 255.0f, 0.0f, NORM_MINMAX);
	rImg.convertTo(rImg, CV_8UC1);

	return rImg;
}

cv::Mat DkMSData::estimateFgd(const cv::Mat& bwImg) const {

	cv::Mat signal = convertToSignal();
	cv::Mat bwVec = imageToColumnVector(bwImg);

	cv::Mat fgdSignal(cvRound(cv::sum(bwImg)[0]/255.0f), signal.cols, signal.depth());

	const unsigned char* bwPtr = bwVec.ptr<unsigned char>();

	int fIdx = 0;
	for (int rIdx = 0; rIdx < signal.rows; rIdx++) {

		if (bwPtr[rIdx] > 0) {
			signal.row(rIdx).copyTo(fgdSignal.row(fIdx));
			fIdx++;
		}
	}

	fgdSignal.convertTo(fgdSignal, CV_32F);
	cv::Mat lowerBound(1, signal.cols, signal.depth());
	cv::Mat upperBound(1, signal.cols, signal.depth());
	unsigned char* lPtr = lowerBound.ptr<unsigned char>();
	unsigned char* uPtr = upperBound.ptr<unsigned char>();

	float w = 1.5f;

	for (int cIdx = 0; cIdx < fgdSignal.cols; cIdx++) {

		float q25 = DkIP::statMomentMat(fgdSignal.col(cIdx), cv::Mat(), 0.25);
		float q75 = DkIP::statMomentMat(fgdSignal.col(cIdx), cv::Mat(), 0.75);
		
		lPtr[cIdx] = DkMath::cropToUChar(q25 - w*(q75-q25));
		uPtr[cIdx] = DkMath::cropToUChar(q75 + w*(q75-q25));
	
		mout << "lower bound: " << (int)lPtr[cIdx] << " upper bound: " << (int)uPtr[cIdx] << dkendl;
	}

	cv::Mat rImg = bwVec.clone();

	for (int rIdx = 0; rIdx < signal.rows; rIdx++) {

		if (bwPtr[rIdx] > 0) {

			const unsigned char* sPtr = signal.ptr<unsigned char>(rIdx);
			unsigned char* rPtr = rImg.ptr<unsigned char>(rIdx);
			for (int cIdx = 0; cIdx < signal.cols; cIdx++) {

				if (sPtr[cIdx] < lPtr[cIdx] || sPtr[cIdx] > uPtr[cIdx]) {
					*rPtr = 100;
					//mout << "sPtr is an outlier: " << (int)sPtr[cIdx] << dkendl;
					break;
				}
			}
		}
	}

	rImg = columnVectorToImage(rImg);

	return rImg;
}

cv::Mat DkMSData::imageToColumnVector(const cv::Mat& img) const {

	if (img.empty())
		return cv::Mat();

	return img.reshape(0, img.rows*img.cols);
}

Mat DkMSData::columnVectorToImage(const cv::Mat& vec) const {

	if (vec.empty() || msImgs.empty())
		return cv::Mat();

	return vec.reshape(0, msImgs[0].rows);
}





