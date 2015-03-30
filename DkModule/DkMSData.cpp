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
#include "DkSegmentation.h"
#include "DkBlobs.h"

DkMSData::DkMSData(const std::vector<cv::Mat>& data) {

	msImgs = data;
	fixInput(msImgs);
	signal = convertToSignal();
}

/**
 * Normalizes the input and converts it to grayscale (if needed).
 * @param imgs all images
 **/ 
void DkMSData::fixInput(std::vector<cv::Mat>& imgs) const {

	for (cv::Mat& img : imgs) {

		if (img.channels() > 1)
			cv::cvtColor(img, img, CV_RGB2GRAY);

		cv::normalize(img, img, 255.0f, 0.0f, NORM_MINMAX);
	}
}

/**
 * Returns all channels as vector of images.
 * @return std::vector<cv::Mat> channels.
 **/ 
std::vector<cv::Mat> DkMSData::getImages() const {

	return msImgs;
}

/**
 * Returns the visible light channel (F2)
 * @return cv::Mat returns the vis channel.
 **/ 
cv::Mat DkMSData::getVisChannel() const {

	if (!msImgs.empty() && msImgs.size() > 1)
		return msImgs[1];

	return cv::Mat();
}

/**
 * Returns the background (IR) channel.
 * @return cv::Mat the IR channel.
 **/ 
cv::Mat DkMSData::getBgChannel() const {

	if (!msImgs.empty())
		return msImgs[msImgs.size()-1];	// bg channel is the last channel

	return cv::Mat();
}

/**
 * The data signal.
 * Each channel is a column in this image.
 * @return cv::Mat the data signal.
 **/ 
cv::Mat DkMSData::getSignal() const {
	
	return signal;
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

	iout << "signal converted in " << dt << dkendl;

	return signal;
}

/**
 * Estimates the foreground from the BW image.
 * This method removes spectral outliers & small blobs
 * that are present in the BW image. It thus increases
 * the precision while decreasing the recall.
 * @param bwImg a binary image
 * @return cv::Mat the fgd image (8UC1)
 **/ 
cv::Mat DkMSData::estimateFgd(const cv::Mat& bwImg) const {

	DkTimer dt;

	unsigned char ignoreVal = 100;
	cv::Mat signal = getSignal();
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

	// define outlier bound
	for (int cIdx = 0; cIdx < fgdSignal.cols; cIdx++) {

		float q25 = DkIP::statMomentMat(fgdSignal.col(cIdx), cv::Mat(), 0.25);
		float q75 = DkIP::statMomentMat(fgdSignal.col(cIdx), cv::Mat(), 0.75);
		
		lPtr[cIdx] = DkMath::cropToUChar(q25 - w*(q75-q25));
		uPtr[cIdx] = DkMath::cropToUChar(q75 + w*(q75-q25));
	}

	cv::Mat rImg = bwVec.clone();

	// set outliers to ignoreVal
	for (int rIdx = 0; rIdx < signal.rows; rIdx++) {

		if (bwPtr[rIdx] > 0) {

			const unsigned char* sPtr = signal.ptr<unsigned char>(rIdx);
			unsigned char* rPtr = rImg.ptr<unsigned char>(rIdx);
			for (int cIdx = 0; cIdx < signal.cols; cIdx++) {

				if (sPtr[cIdx] < lPtr[cIdx] || sPtr[cIdx] > uPtr[cIdx]) {
					*rPtr = ignoreVal;
					break;
				}
			}
		}
	}

	rImg = columnVectorToImage(rImg);
	ioutc << "foreground estimation takes " << dt << dkendl;

	return rImg;
}

/**
 * Removes sensor noise (e.g. salt & pepper)
 * @param img an 8UC1 image
 * @return cv::Mat the cleaned & normalized 8UC1 image
 **/ 
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

/**
 * Removes background noise from an image.
 * @param img the 8UC1 image to be cleaned.
 * @param bgImg a background image (generally the IR image)
 * @return cv::Mat the cleaned 8UC1 image.
 **/ 
cv::Mat DkMSData::removeBackground(const cv::Mat& img, const cv::Mat& bgImg) const {

	if (img.size() != bgImg.size()) {
		woutc << "[WARNING] in removeBackground - image size does not correspond..." << dkendl;
		return img.clone();
	}

	cv::Mat rImg(img.size(), CV_16SC1);

	// crop image values (outside the bounds)
	for (int rIdx = 0; rIdx < rImg.rows; rIdx++) {

		short* rPtr = rImg.ptr<short>(rIdx);
		const unsigned char* iPtr = img.ptr<unsigned char>(rIdx);
		const unsigned char* bPtr = bgImg.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < rImg.cols; cIdx++) {

			rPtr[cIdx] = iPtr[cIdx]-bPtr[cIdx];
		}
	}

	cv::normalize(rImg, rImg, 255.0f, 0.0f, NORM_MINMAX);
	rImg.convertTo(rImg, CV_8UC1);

	return rImg;
}

/**
 * Removes small blobs that are not close to any other blobs.
 * @param bwImg a 8UC1 binary image.
 * @return cv::Mat the cleaned 8UC1 binary image.
 **/ 
cv::Mat DkMSData::removeBackgroundBlobs(const cv::Mat& bwImg) const {

	cv::Mat cleanImg = bwImg.clone();
	DkBlobs<DkAttr> blobs(cleanImg);
	blobs.calcArea();

	float sumArea = 0;
	for (int idx = 0; idx < blobs.getSize(); idx++) {
		sumArea += abs(blobs.getBlob(idx).getArea());
	}

	blobs.imgFilterArea(cvRound(sumArea/blobs.getSize()*0.75));

	DkIP::invertImg(cleanImg);
	distanceTransform(cleanImg, cleanImg, CV_DIST_L2, 3);

	cleanImg = cleanImg < 50;
	cleanImg = bwImg & cleanImg;

	return cleanImg;
}

/**
 * Converts an image to a column vector.
 * @param img an image (MxN).
 * @return cv::Mat the corresponding column vector (M*Nx1).
 **/ 
cv::Mat DkMSData::imageToColumnVector(const cv::Mat& img) const {

	if (img.empty())
		return cv::Mat();

	return img.reshape(0, img.rows*img.cols);
}

/**
 * Converts a column vector to the image.
 * NOTE: the column vector must have M*N elements.
 * where MxN is the data image size.
 * @param vec a M*N column vector.
 * @return cv::Mat the resulting MxN image.
 **/ 
Mat DkMSData::columnVectorToImage(const cv::Mat& vec) const {

	if (vec.empty() || msImgs.empty())
		return cv::Mat();

	return vec.reshape(0, msImgs[0].rows);
}





