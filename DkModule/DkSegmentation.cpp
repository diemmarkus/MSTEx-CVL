/*******************************************************************************************************
 DkSegmentation.cpp
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Florian Kleber <kleber@caa.tuwien.ac.at>
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

#include "DkSegmentation.h"

#include "DkBlobs.h"
#include "DkAttr.h"

void DkSegmentationBase::setInputG(Mat g, Mat m) {
	this->grayImg = g;
	this->mask = m;
}

void DkSegmentationBase::setInputRGB(Mat rgb, Mat m) {
	this->rgbImg = rgb;
	this->mask = m;
}

void DkSegmentationBase::setRemoveBorderSize(int erodeMaskSize) {

	if (erodeMaskSize >= 0)
		this->erodeMaskSize = erodeMaskSize;
	else {
		std::string msg = "Filter Size must be >= 0, it is: " + DkUtils::stringify(erodeMaskSize) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}


bool DkSegmentationSu::empty() {
	return (rgbImg.empty() && mask.empty() && imgs == 0 && segImg.empty());
}

void DkSegmentationSu::init() {
	//initialization segmentation params

	className = "DkSegmentationSu";
	medianFilter = true;
	strokeW = 7;	// we take a fixed stroke width for the MSTEx challenge
}

DkSegmentationSu::DkSegmentationSu() : DkSegmentationBase() {

	imgs = 0;
	init();
}

DkSegmentationSu::DkSegmentationSu(Mat rgb, Mat mask) {

	if (rgb.channels() > 1)
		this->rgbImg = rgb;//.clone();
	else
		this->grayImg = rgb;
	this->mask = mask;//.clone();
	imgs = 0;

	init();
}

DkSegmentationSu::DkSegmentationSu(DkImageSource *imgs)  : DkSegmentationBase(imgs) {

	this->rgbImg = imgs->getRgbImg();//.clone();
	this->mask = imgs->getMask();//.clone();

	init();
}

void DkSegmentationSu::checkInput() const {

	if (rgbImg.channels() != 3 && grayImg.empty()) throw DkMatException("not a 3 channel input image", __LINE__, __FILE__);
	if (grayImg.channels() != 1 && rgbImg.empty()) throw DkMatException("not a 1 channel input image", __LINE__, __FILE__);
	if (!rgbImg.empty() && rgbImg.depth() != CV_8U) throw DkMatException("not a CV_8U input image", __LINE__, __FILE__);
	if (!grayImg.empty() && grayImg.depth() != CV_8U) throw DkMatException("not a CV_8U input image", __LINE__, __FILE__);
	if (mask.empty()) throw DkMatException("empty mat", __LINE__, __FILE__);
	if (mask.type() != CV_8UC1) throw DkMatException("not a CV_8UC1 mask image", __LINE__, __FILE__);

	// check if the mask fits to the image
	if (rgbImg.size() != mask.size() && grayImg.size() != mask.size()) {
		std::string msg = "Image size does not correspond to the mask's size (" + DkVector(rgbImg.size()).toString() + " != " + DkVector(mask.size()).toString() + ")";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
}

void DkSegmentationSu::convertInputData() {
	checkInput();

	if (imgs == 0) { //either konstruktor (rgb, mask) called or initialized with setInputG(gray, mask) or setInputRgb(rgb, mask)
		if (!rgbImg.empty())
			cvtColor(rgbImg, grayImg, CV_RGB2GRAY);
		//erodedMask = mask.clone();
		erodedMask = DkIP::fastErodeImage(mask, cvRound(erodeMaskSize));
	} else {
		grayImg = imgs->getGrayImg().clone();

		if (erodeMaskSize == DK_ERODED_MASK_SIZE) {
			erodedMask = imgs->getErodedMask();//;.clone();
		} else {
			//erode mask with current filter size
			erodedMask = DkIP::fastErodeImage(imgs->getMask(), cvRound(erodeMaskSize));
		}
	}
}

Mat DkSegmentationSu::getSegmented() {

	if (segImg.empty()) {

		DkTimer dt = DkTimer();
		convertInputData();		//converts rgb to grayvalue image, and erodes the mask
		computeSu(segImg);
		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax Su image computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] localMinMax Su image computed in: " << dt << dkendl;
	}

	return segImg;
}

void DkSegmentationSu::compute() {

	convertInputData();		//converts rgb to grayvalue image, and erodes the mask
	computeSu(segImg);
}

void DkSegmentationSu::filterSegImg(int size) {

	getSegmented();

	DkBlobs<DkAttr> segFilter(segImg);
	segFilter.imgFilterArea(size);
}

cv::Mat DkSegmentationSu::filterSegImgAuto(const cv::Mat& segImg) {

	cv::Mat segImgCleaned = segImg.clone();
	DkBlobs<DkAttr> segFilter(segImgCleaned);
	segFilter.calcArea();

	float sumArea = 0;

	for (int idx = 0; idx < segFilter.getSize(); idx++) {
		sumArea += abs(segFilter.getBlob(idx).getArea());
	}

	//moutc << sumArea/segFilter.getSize()*0.5 << " fgd filter size" << dkendl;
	//moutc << sumArea << " sumArea" << dkendl;

	segFilter.imgFilterArea(cvRound(sumArea/segFilter.getSize()*0.5));

	return segImgCleaned;
}

void DkSegmentationSu::computeSu(Mat &resultSegImg) {

	contrastImg = computeContrastImg();
	binContrastImg = computeBinaryContrastImg(contrastImg);

	Mat tmp1;
	binContrastImg.convertTo(tmp1, CV_32F, 1.0f/255.0f);
	tmp1 = contrastImg.mul(tmp1);
	strokeW = getStrokeWidth(tmp1);
	tmp1.release();

	if (releaseDebug == DK_RELEASE_IMGS)
		contrastImg.release();

	// now we need a 32F image
	Mat tmp;	
	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);		//don't use image source class -> enables to set grayimg to a different value (MinMaxSatSeg)

	computeThrImg(tmp, binContrastImg, thrImg, resultSegImg);					//compute threshold image

	if (releaseDebug == DK_RELEASE_IMGS)
		binContrastImg.release();

	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition

	if (releaseDebug == DK_RELEASE_IMGS) {
		thrImg.release();
		//erodedMask.release();
	}

}

Mat DkSegmentationSu::computeContrastImg() {

	Mat tmp = Mat(grayImg.size(), CV_32FC1);

	Mat maxImg = DkIP::dilateImage(grayImg, 3, DkIP::DK_SQUARE);
	Mat minImg = DkIP::erodeImage(grayImg, 3, DkIP::DK_SQUARE);

	//speed up version of opencv style
	for(int i = 0; i < maxImg.rows; i++)
	{
		float *ptrCon = tmp.ptr<float>(i);
		unsigned char *ptrMin = minImg.ptr<unsigned char>(i);
		unsigned char *ptrMax = maxImg.ptr<unsigned char>(i);
		unsigned char *ptrMask = erodedMask.ptr<unsigned char>(i);

		for(int j = 0; j < maxImg.cols; j++, ptrCon++, ptrMin++, ptrMax++, ptrMask++) {
			*ptrCon = (*ptrMask > 0) ? contrastVal(ptrMax,ptrMin) : 0.0f;
		}
	}
	minImg.release(); // early release
	maxImg.release(); // early release;

	//DkIP::imwrite("ContrastImg181.png", tmp);

	return tmp;
}

Mat DkSegmentationSu::computeBinaryContrastImg(Mat contrast) {

	Mat histogram = DkIP::computeHist(contrast, erodedMask);
	double l = DkIP::getThreshOtsu(histogram) / 255.0;

	Mat contrastImgThr;
	threshold(contrast, contrastImgThr, l, 1.0f, CV_THRESH_BINARY);
	contrastImgThr.convertTo(contrastImgThr, CV_8U, 255, 0);

	return contrastImgThr;
}

float DkSegmentationSu::getStrokeWidth(Mat contrastImg) {
#if 1
	
	if (strokeW != -1.0f)
		return strokeW;
	
	int height = contrastImg.rows;
	int dy = 1;
	//	int cnt=0;
	float strokeWidth = 0;
	std::list<int> diffs;
	std::list<float> locInt;
	diffs.clear();
	Mat vec;

	if (height > 100) dy = height / 4;
	dy = 1;

	for(int i=0; i < height; i+= dy) {
		Mat row = contrastImg.row(i);
		computeDistHist(row, &diffs, &locInt, 0.0f);
	}

	vec.create(1, 40, CV_32FC1);
	vec = 0.0f;
	std::list<int>::iterator iter = diffs.begin();
	//std::list<float>::iterator iter2 = locInt.begin();
	float *ptr = vec.ptr<float>(0);
	int idx;
	while (iter != diffs.end()) {
		//idx = cvFloor(*iter/0.5f);
		idx = (*iter);
		//printf("distance: %i\n", *iter);
		if (idx < vec.cols)
			ptr[idx]++;//=(*iter2);

		iter++;
		//iter2++;
	}
	diffs.clear();

	//DkUtils::printMat(vec, "strokeWidth");	// diem: nervt : )

	//Mat sHist = DkIP::convolveSymmetric(vec, DkIP::get1DGauss(3.0f));
	Mat sHist = vec;
	//float *sHistPtr = sHist.ptr<float>(0);

	double sMin, sMax;
	Point pMinIdx, pMaxIdx;
	int minIdx, maxIdx;
	minMaxLoc(sHist, &sMin, &sMax, &pMinIdx, &pMaxIdx);
	minIdx = pMinIdx.x;
	maxIdx = pMaxIdx.x;

	strokeWidth = 1.0f + (float)maxIdx;  //offset since idx starts with 0
	
	if (strokeWidth < 3)
		return 3.0f;
	else
		return strokeWidth;
#else
	int height = contrastImg.rows;
	int dy = 1;
	float strokeWidth = 0;
	CDistHistList dhl(contrastImg.rows * contrastImg.cols / 2);
	CDistHistList ldhl(contrastImg.cols / 2);
	Mat vec;

	if (height > 100) dy = height / 4;
	dy = 1;
	for(int i=0; i < height; i+= dy) {
		Mat row = contrastImg.row(i);
		ldhl.Reset();
		computeDistHist(row, &dhl, &ldhl, 0.0f);
	}

	vec.create(1, 40, CV_32FC1);
	vec = 0.0f;
	float *ptr = vec.ptr<float>(0);
	int idx;

	for (int i = 0; i < dhl.getElemCount(); i++)
	{
		idx = dhl[i];
		if (idx < vec.cols)
			ptr[idx]++;//=(*iter2);
	}

	Mat sHist = vec;

	double sMin, sMax;
	Point pMinIdx, pMaxIdx;
	int minIdx, maxIdx;
	minMaxLoc(sHist, &sMin, &sMax, &pMinIdx, &pMaxIdx);
	minIdx = pMinIdx.x;
	maxIdx = pMaxIdx.x;

	strokeWidth = 1.0f + (float)maxIdx;  //offset since idx starts with 0

	if (strokeWidth < 3)
		return 3.0f;
	else
		return strokeWidth;
#endif
}

inline float DkSegmentationSu::thresholdVal(float *mean, float *std) {
	return (*mean+*std/2);
}

inline void DkSegmentationSu::calcFilterParams(int &filterS, int &Nm) {
	filterS = cvRound(strokeW);
	filterS = (filterS % 2) != 1 ? filterS+1 : filterS;
	Nm = filterS;
}

inline float DkSegmentationSu::contrastVal(unsigned char* maxVal, unsigned char * minVal) {

	return (float)(*maxVal - *minVal)/((float)(*maxVal) + (float)(*minVal) + FLT_MIN);
}

void DkSegmentationSu::computeThrImg(Mat grayImg32F, Mat binContrast, Mat &thresholdImg, Mat &thresholdContrastPxImg) {
	int filtersize, Nmin;
	
	calcFilterParams(filtersize, Nmin);
	//filtersize = cvRound(strokeW);
	//filtersize = (filtersize % 2) != 1 ? filtersize+1 : filtersize;
	//Nmin = filtersize;

	DkUtils::printDebug(DK_DEBUG_C, "kernelsize:  %i  Nmin:  %i\n", filtersize, Nmin);

	Mat contrastBin32F;
	binContrast.convertTo(contrastBin32F, CV_32FC1, 1.0f/255.0f);
	//DkIP::imwrite("contrastBin343.png", binContrast);
	// compute the mean image
	Mat meanImg = grayImg32F.mul(contrastBin32F);	// do not overwrite the gray image
	Mat stdImg = meanImg.mul(meanImg);

	Mat intContrastBinary = contrastBin32F;

	// save RAM for small filter sizes
	if (filtersize <= 7) { 

		DkUtils::printDebug(DK_DEBUG_C, "calling the new mean...\n");
		// 1-dimensional since the filter is symmetric
		Mat sumKernel = Mat(filtersize, 1, CV_32FC1);
		sumKernel = 1.0;

		//BORDER_REFLECT
		// filter y-coordinates
		filter2D(meanImg, meanImg, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
		filter2D(stdImg, stdImg, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
		filter2D(intContrastBinary, intContrastBinary, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);

		// filter x-coordinates
		sumKernel = sumKernel.t();
		filter2D(meanImg, meanImg, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
		filter2D(stdImg, stdImg, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
		filter2D(intContrastBinary, intContrastBinary, CV_32FC1, sumKernel, cv::Point(-1,-1), 0.0, BORDER_REFLECT);

	}
	else {

		// is way faster than the filter2D function for kernels > 7
		Mat intImg;
		integral(meanImg, intImg);
		meanImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DkIP::DK_BORDER_FLIP);

		// compute the standard deviation image
		integral(stdImg, intImg);
		stdImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DkIP::DK_BORDER_FLIP);
		intImg.release(); // early release

		integral(contrastBin32F, intContrastBinary);
		contrastBin32F.release();
		intContrastBinary = DkIP::convolveIntegralImage(intContrastBinary, filtersize, 0, DkIP::DK_BORDER_FLIP);
	}
	//DkIP::imwrite("meanImg343.png", meanImg, true);

	//DkIP::imwrite("meanImg343.png", meanImg, true);

	meanImg /= intContrastBinary;
	DkUtils::printDebug(DK_DEBUG_INFO, "mean: %s\n", DkUtils::getMatInfo(meanImg).c_str());

	//DkIP::imwrite("meanImgDivContrastBin348.png", meanImg);

	float *mPtr, *cPtr;
	float *stdPtr;

	for (int rIdx = 0; rIdx < stdImg.rows; rIdx++) {

		mPtr = meanImg.ptr<float>(rIdx);
		stdPtr = stdImg.ptr<float>(rIdx);
		cPtr = intContrastBinary.ptr<float>(rIdx);

		for (int cIdx = 0; cIdx < stdImg.cols; cIdx++, mPtr++, stdPtr++, cPtr++) {

			*stdPtr = (*cPtr != 0) ? *stdPtr/(*cPtr) - (*mPtr * *mPtr) : 0.0f;	// same as OpenCV 0 division
			if (*stdPtr < 0.0f) *stdPtr = 0.0f;		// sqrt throws floating point exception if stdPtr < 0

		}
	}
	
	sqrt(stdImg, stdImg);	// produces a floating point exception if < 0...

	//DkIP::imwrite("stdImg371.png", stdImg);

	Mat thrImgTmp = Mat(grayImg32F.size(), CV_32FC1);
	Mat segImgTmp = Mat(grayImg32F.size(), CV_8UC1);

	float *ptrStd, *ptrThr, *ptrSumContrast, *ptrMean;
	unsigned char *ptrSeg;

	for(int rIdx = 0; rIdx < stdImg.rows; rIdx++) {
		ptrThr = thrImgTmp.ptr<float>(rIdx);
		ptrMean = meanImg.ptr<float>(rIdx);
		ptrStd = stdImg.ptr<float>(rIdx);
		ptrSeg = segImgTmp.ptr<unsigned char>(rIdx);
		ptrSumContrast = intContrastBinary.ptr<float>(rIdx);
		
		for(int cIdx = 0; cIdx < stdImg.cols; cIdx++, ptrThr++, ptrMean++, ptrStd++, ptrSeg++, ptrSumContrast++) {
			*ptrThr = thresholdVal(ptrMean, ptrStd);
			*ptrSeg = *ptrSumContrast > Nmin ? 255 : 0;
		}
	}

	//DkIP::imwrite("thrImg388.png",thrImgTmp);
	//thresholdContrastPxImg = binContrast;
	thresholdContrastPxImg = segImgTmp;
	thresholdImg = thrImgTmp;
}

void DkSegmentationSu::release() {

	rgbImg.release();
	mask.release();
	contrastImg.release();
	binContrastImg.release();
	thrImg.release();
	erodedMask.release();
	grayImg.release();
	segImg.release();

	imgs = 0;
}

void DkSegmentationSu::clearDebugImg() {

	contrastImg.release();
	binContrastImg.release();
	thrImg.release();
	erodedMask.release();
	grayImg.release();
	//// input images...			//FK 15112010 bug, otherwise mask image is deleted before confidence is calculated
	//rgbImg.release();
	//mask.release();
}

Mat DkSegmentationSu::getThrImg() {

	if (releaseDebug == DK_RELEASE_IMGS && thrImg.empty()) {
		std::string msg = "[" + getName() + "]" + "debug images are released\n";
		DkUtils::printDebug(DK_WARNING, msg.c_str());
	}
	return thrImg;
}

Mat DkSegmentationSu::getContrastImg() {

	if (releaseDebug == DK_RELEASE_IMGS && contrastImg.empty()) {
		std::string msg = "[" + getName() + "]" + "debug images are released\n";
		DkUtils::printDebug(DK_WARNING, msg.c_str());
	}
	return contrastImg;
}

Mat DkSegmentationSu::getBinContrastImg() {
	return binContrastImg;
}

float DkSegmentationSu::getMeanContrast() {
	return -1.0f;
}

void DkSegmentationSu::saveDebugImg(Mat img, std::string path, std::string filename) {

	std::string saveName, saveFull;

	saveName = "_" + getName() + "_DKsegSu";
	//saveName = "_segDkSatFilt";
	saveFull = path + DkUtils::createFileName(filename, saveName);

	if (!segImg.empty()) {

		int rows = segImg.rows;
		int cols = segImg.cols;
		Mat grayimg;
		cvtColor(img, grayimg, CV_RGB2GRAY);

		Mat result = Mat(rows, cols*2, CV_32FC1);
		Mat a = result(Range::all(), Range(0, cols));
		grayimg.convertTo(a, CV_32F);
		Mat b = result(Range::all(), Range(cols, cols+cols));
		segImg.convertTo(b, CV_32F);

		DkIP::imwrite(saveFull.c_str(), result);
		std::string msg = "[" + getName()  + "] debug images saved...";
		DkUtils::printDebug(DK_INFO, msg.c_str());
	} else
		throw DkMatException("empty mat", __LINE__, __FILE__);
}

std::string DkSegmentationSu::toString() const {

	std::string msg = "Segmentation --------------------------\n";
	msg += "parameters:\n";
	msg += "    Erode Mask size: " + DkUtils::stringify(erodeMaskSize) + "\n";

	return msg;
}

void DkSegmentationSu::computeDistHist(Mat src, std::list<int> *maxDiffList, std::list<float> *localIntensity, float gSigma) {

	std::list<int> localMaxList;
	std::list<int>::iterator localMaxIter;

	Mat sHist;
	if (gSigma > 0)
		sHist = DkIP::convolveSymmetric(src, DkIP::get1DGauss(gSigma));
	else
		sHist = src;

	localMaxList.clear();

	float *sHistPtr = sHist.ptr<float>();

	for (int prevIdx = 0, currIdx = 1, nextIdx=2; prevIdx < sHist.cols; prevIdx++, nextIdx++, currIdx++) {

		currIdx %= sHist.cols;
		nextIdx %= sHist.cols;

		if ((sHistPtr[prevIdx] <= sHistPtr[currIdx]) && (sHistPtr[currIdx] > sHistPtr[nextIdx])) {
			localMaxList.push_back(currIdx);
			localIntensity->push_back(sHistPtr[currIdx]);
			//localIntensity->push_back(255.0f);
		}
	}
	if (localIntensity->size() >= 1)
		localIntensity->pop_back();

	if (localMaxList.empty() || localMaxList.size() <= 1)	
		return;	// at least two local maxima present?

	localMaxIter = localMaxList.begin();
	int cIdx = *localMaxIter;
	++localMaxIter;	// skip the first element

	// compute the distance between peaks
	int lIdx = -1;
	int idx = 1;
	while (localMaxIter != localMaxList.end()) {

		lIdx = cIdx;
		cIdx = *localMaxIter;

		//if (idx%2 == 0)
		maxDiffList->push_back(abs(cIdx-lIdx));
		idx++;
		//printf("distance: %i\n", cIdx-lIdx);
		++localMaxIter;
	}
}

void DkSegmentationSu::computeDistHist(Mat src, CDistHistList *pDHL, CDistHistList *pLocalDHL, float gSigma)
{
	Mat sHist;
	if (gSigma > 0)
		sHist = DkIP::convolveSymmetric(src, DkIP::get1DGauss(gSigma));
	else
		sHist = src;

	float *sHistPtr = sHist.ptr<float>();

	for (int prevIdx = 0, currIdx = 1, nextIdx=2; prevIdx < sHist.cols; prevIdx++, nextIdx++, currIdx++) {

		currIdx %= sHist.cols;
		nextIdx %= sHist.cols;

		if ((sHistPtr[prevIdx] <= sHistPtr[currIdx]) && (sHistPtr[currIdx] > sHistPtr[nextIdx])) {
			pLocalDHL->add(currIdx);
		}
	}

	if (pLocalDHL->getElemCount() <= 1)	
		return;	// at least two local maxima present?

	int localMaxIter = 0;
	int cIdx = (*pLocalDHL)[0];
	++localMaxIter;	// skip the first element


	// compute the distance between peaks
	int lIdx = -1;
	while (localMaxIter < pLocalDHL->getElemCount()) {

		lIdx = cIdx;
		cIdx = (*pLocalDHL)[localMaxIter];

		pDHL->add(abs(cIdx-lIdx));

		++localMaxIter;
	}

}


//-------------------------------------------------------------------------------------------------------------------------------------------
void DkSegmentationSuIpk::init() {
	className = "DkSegmentationSuIpk";
}

Mat DkSegmentationSuIpk::getSegmented() {

	if (segImg.empty()) {

		DkTimer dt = DkTimer();
		convertInputData();		//converts rgb to grayvalue image, and erodes the mask
		computeSuIpk(segImg);

		if (medianFilter)
			medianBlur(segImg, segImg, 3);

		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax SuIpk image computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] localMinMax SuIpk image computed in: " << dt << dkendl;

	}

	return segImg;
}

void DkSegmentationSuIpk::computeSuIpk(Mat &resultSegImg) {

	contrastImg = computeContrastImg();
	binContrastImg = computeBinaryContrastImg(contrastImg);

	if (releaseDebug == DK_RELEASE_IMGS)
		contrastImg.release();

	Mat tmp1;
	strokeW = getStrokeWidth(tmp1);

	// now we need a 32F image
	Mat tmp;	
	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);		//don't use image source class -> enables to set grayimg to a different value (MinMaxSatSeg)

	computeThrImg(tmp, binContrastImg, thrImg, resultSegImg);					//compute threshold image

	if (releaseDebug == DK_RELEASE_IMGS)
		binContrastImg.release();

	//DkIP::imwrite("resultSegImg611.png", resultSegImg);
	//DkIP::imwrite("resultThrImg612.png", tmp<=thrImg);
	//DkIP::imwrite("resultThrImg623.png", thrImg);

	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition

	//DkIP::imwrite("resultImg616.png", resultSegImg);

	if (releaseDebug == DK_RELEASE_IMGS) {
		thrImg.release();
		//erodedMask.release();
	}

}

float DkSegmentationSuIpk::contrastVal(unsigned char* maxVal, unsigned char * minVal) {

	return 2.0f*(float)(*maxVal - *minVal)/((float)(*maxVal) + (float)(*minVal) + 255.0f + FLT_MIN);
}

float DkSegmentationSuIpk::getStrokeWidth(Mat contrastImg) {

	return 4.0f;
}

void DkSegmentationSuIpk::calcFilterParams(int &filterS, int &Nm) {

	if (strokeW >= 4.5) strokeW = 3.0;
	filterS = cvRound(strokeW*10);
	if ((filterS % 2) != 1) filterS+=1;
	Nm = cvFloor(strokeW*10);
}

//-------------------------------------------------------------------------------------------------------------------------------------------

void DkSegmentationFgdIpk::init() {
	
	//initialization segmentation params
	fgdEstFilterSize = 32;
	sigmSlope = 15.0f;

	confidence = -1.0f;
	meanContrast[0] = -1.0f;
	meanContrast[1] = -1.0f;
	meanContrast[2] = -1.0f;

	stdContrast[0] = -1.0f;
	stdContrast[1] = -1.0f;
	stdContrast[2] = -1.0f;

	className = "DkSegmentationFgd";
	medianFilter = true;
}

void DkSegmentationFgdIpk::setfgdEstFilterSize(int s) {
	if (s >= 3)
		fgdEstFilterSize = s;
	else {
		std::string msg = "Filter Size must be >= 3, it is: " + DkUtils::stringify(s) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

void DkSegmentationFgdIpk::setSigmSlope(float s) {
	if (s > 1)
		sigmSlope = s;
	else {
		std::string msg = "Sigmoid Slope must be > 1, it is: " + DkUtils::stringify(s) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}

}

Mat DkSegmentationFgdIpk::computeMeanFgdEst(Mat grayImg32F) {

	Mat tmp;

	if (fgdEstFilterSize < 3) {
		tmp = Mat(grayImg32F.size(), CV_32FC1);
		tmp.setTo(1.0f);
	} else {
		Mat fgdEstImgInt = Mat(grayImg32F.rows+1, grayImg32F.cols+1, CV_64FC1);
		integral(grayImg32F, fgdEstImgInt);
		tmp = DkIP::convolveIntegralImage(fgdEstImgInt, fgdEstFilterSize, 0, DkIP::DK_BORDER_ZERO);
		fgdEstImgInt.release(); // early release

		//DkIP::mulMask(fgdEstImg, mask);	// diem: otherwise values outside the mask are mutual
		normalize(tmp, tmp, 1.0f, 0, NORM_MINMAX, -1, mask);  // note: values outside the mask remain outside [0 1]
		DkIP::invertImg(tmp);
	}

	return tmp;
}

Mat DkSegmentationFgdIpk::getFgdEstImg() {
	
	return fgdEstImg;
}

Mat DkSegmentationFgdIpk::getSegmented() {

	if (segImg.empty()) {

		DkTimer dt = DkTimer();
		convertInputData();		//converts rgb to gray value image, and erodes the mask
		computeFgd(segImg);

		if (medianFilter)
			medianBlur(segImg, segImg, 3);

		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax SegFgd image computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] localMinMax segFgd image computed in: " << dt << dkendl;
	}

	return segImg;
}

void DkSegmentationFgdIpk::computeFgd(Mat &resultSegImg) {
	int tmpDebug;
	tmpDebug = releaseDebug;
	releaseDebug = DK_KEEP_IMGS;
	computeSuIpk(resultSegImg);
	releaseDebug = tmpDebug;
	computeConfidence();

	if (releaseDebug == DK_RELEASE_IMGS) {
		contrastImg.release();
		binContrastImg.release();
	}

	// now we need a 32F image
	Mat tmp;
	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);

	weightFunction(tmp, thrImg, erodedMask);
	
	if (releaseDebug == DK_RELEASE_IMGS)
		fgdEstImg.release();

	//DkIP::imwrite("segSu744.png", resultSegImg);
	//DkIP::imwrite("segThr745.png", tmp<=thrImg);
	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition
	//DkIP::imwrite("segThr745.png", segImg);

	if (releaseDebug == DK_RELEASE_IMGS) {
		thrImg.release();
		//erodedMask.release();
	}

}


void DkSegmentationFgdIpk::weightFunction(Mat &grayI, Mat &thrI, Mat maskI) {
	
	fgdEstImg = computeMeanFgdEst(grayI);					//compute foreground estimation
	//DkIP::imwrite("fgdestImg759.png", fgdEstImg);

	Mat tmpMask;
	threshold(thrI, tmpMask, 0, 1.0, CV_THRESH_BINARY);
	Mat histogram = DkIP::computeHist(thrI, tmpMask);		//weight gray values with sigmoid function according
	tmpMask.release();

	double l = DkIP::getThreshOtsu(histogram)/255.0f;		//sigmoid slope, centered at l according text estimation
	float sigmaSlopeTmp = sigmSlope/255.0f;

	float fm[256];
	for (int i = 0; i < 256; i++)
		fm[i] = 1.0f/(1.0f + std::exp(((i/255.0f) - (float)l) * (-1.0f/(sigmaSlopeTmp))));

	for(int i = 0; i < grayI.rows; i++)
	{
		float *ptrGray = grayI.ptr<float>(i);
		float *ptrThr = thrI.ptr<float>(i);
		float *ptrFgdEst = fgdEstImg.ptr<float>(i);
		unsigned char *ptrMask = maskI.ptr<unsigned char>(i);

		for(int j = 0; j < grayI.cols; j++, ptrGray++, ptrThr++, ptrFgdEst++, ptrMask++) {
			*ptrGray = fm[cvRound(*ptrGray*255.0f)];
			*ptrThr = (*ptrMask != 0) ? *ptrThr * (*ptrFgdEst) : 0.0f;
		}
	}
}

void DkSegmentationFgdIpk::computeConfidence() {
	if (meanContrast[0] == -1.0f) {
		int n = countNonZero(binContrastImg);
		//TODO: prove if contrastImg in normalize and statmomentMat must be set to tmp?
		//Mat tmp = contrastImg.clone();
		normalize(contrastImg, contrastImg, 1, 0, NORM_MINMAX, -1, binContrastImg);
		if (n > 2000)
			meanContrast[0] = DkIP::statMomentMat(contrastImg, binContrastImg, 0.5f, 5000);
		else
			meanContrast[0] = 0.0f;
	}
}

void DkSegmentationFgdIpk::release() {
	
	DkSegmentationSuIpk::release();
	
	fgdEstImg.release();
}

void DkSegmentationFgdIpk::clearDebugImg() {

	DkSegmentationSuIpk::clearDebugImg();

	fgdEstImg.release();
}

std::string DkSegmentationFgdIpk::toString() const {

	std::string msg = "";
	msg += DkSegmentationSuIpk::toString();
	msg += "    Filter Size for the foreground estimation: " + DkUtils::stringify(fgdEstFilterSize) + "         sigmoid Slope: " + DkUtils::stringify(sigmSlope) + "\n";

	return msg;
}
//-------------------------------------------------------------------------------------------------------------------------------------------

void DkSegmentationSatIpk::init() {
	
	sensitivity = 0.995f;
	confidenceThreshold = 0.1f;
	medianFilter = true;

	className = "DkSegmentationSat";
}

void DkSegmentationSatIpk::setSensitivity(float sens) {
	if (sens <= 1.0f && sens > 0.0f)
		sensitivity = 1.0f - sens/100.0f;
	else {
		std::string msg = "Sensitivity must be ]0 1], it is: " + DkUtils::stringify(sens) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

void DkSegmentationSatIpk::setConfidenceThresh(float th) {
	if (th >= 0.0f || th <= 1.0f)
		confidenceThreshold = th;
	else {
		std::string msg = "Threshold must be [0 1], it is: " + DkUtils::stringify(th) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

Mat DkSegmentationSatIpk::getSegmented() {

	//convertInputData();
	
	//getConfidence();

	// flos code (even if diem is annotated)
	// correct, but diem wanted it that way...
	// but: now we don't iterate twice over the same image
	// könnte richtig sein - oder auch nicht!
	if (segSatImg.empty()) {
		
		DkTimer dt;
		
		//classical Segmentation needed for the removal of border pixel
		convertInputData();
		computeSat(false);		// calls getConfidence()

		if (medianFilter && !segSatImg.empty())
			medianBlur(segSatImg, segSatImg, 3);

		if (medianFilter && !segImg.empty())
			medianBlur(segImg, segImg, 3);

		//mout << "median filter: " << medianFilter << dkendl;

		// >DIR: otherwise we see each call to getConfidence (e.g. getSegmented() in 2 ms) [17.10.2012 markus]
		mout << "[DkSegmentation] getSegmented computed in: " << dt << dkendl;
	}

	if (confidence > confidenceThreshold)
		return segSatImg;
	else
		return segImg;

	

}

Mat DkSegmentationSatIpk::getFilteredSegmented(int size) {
	
	DkTimer dt = DkTimer();
	getSegmented();

	if (confidence > confidenceThreshold) {
		DkBlobs<DkAttr> segFilter(segSatImg);
		segFilter.imgFilterArea(size);

		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] getFilteredSegmented computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] getFilteredSegmented image computed in: " << dt << dkendl;

		return segSatImg;
	} else {
		DkBlobs<DkAttr> segFilter(segImg);
		segFilter.imgFilterArea(size);

		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] getFilteredSegmented computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] getFilteredSegmented image computed in: " << dt << dkendl;

		return segImg;
	}

	
}

Mat DkSegmentationSatIpk::getSegSatImg() {
	//classical Segmentation needed for the removal of border pixel
	DkTimer dt = DkTimer();

	if (!segSatImg.empty())
		segSatImg.release();
	convertInputData();

	computeSat(true);

	//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] SegSat computed in: %s\n", dt.getTotal().c_str());
	mout << "[DkSegmentation] SegSat image computed in: " << dt << dkendl;

	return segSatImg;
}

Mat DkSegmentationSatIpk::getSegSuFgdImg() {

	DkTimer dt = DkTimer();

	if (segImg.empty())
		getSegmented();

	//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] SegSuFgd computed in: %s\n", dt.getTotal().c_str());
	mout << "[DkSegmentation] segSuFgd image computed in: " << dt << dkendl;

	return segImg;
}

void DkSegmentationSatIpk::filterSatImg(int size) {
	getSegSatImg();
	//getSegmented();

	DkBlobs<DkAttr> segFilter(segSatImg);
	segFilter.imgFilterArea(size);

}

Mat DkSegmentationSatIpk::enhanceImg() {
	Mat tmp = computeSatImg(); //compute the saturation image
	tmp = enhanceSatImg(tmp); //enhance it according defined thresholds

	return tmp;
}

float DkSegmentationSatIpk::getMeanContrast() {
	return (float)meanContrast[0];
}

void DkSegmentationSatIpk::computeSat(bool computeAlways) {

	if (segImg.empty())
		computeFgd(segImg);

	if (((float)meanContrast[0] > confidenceThreshold) || computeAlways) {

		//Mat satImg = computeSatImg();		
		//satImg = enhanceSatImg(satImg);
		Mat satImg = enhanceImg();

		grayImg = satImg;					//CV_8U

		computeFgd(segSatImg);

		getConfidence();

		//to enhance the segmented black text (TODO: segImg should be filtered first)
		bitwise_or(segImg, segSatImg, segSatImg);
		//save time, but changes the input grayvalue image
		//cvtColor(rgbImg, grayImg, CV_RGB2GRAY);	//restore grayValue image, probably skip to save memory?
		//bitwise_and(segSatImg, erodedMask, segSatImg);

	} else {
		confidence = (float)meanContrast[0];
		segSatImg = segImg;
	}

	if (releaseDebug == DK_RELEASE_IMGS)
		clearDebugImg();

	grayImg.release(); //if this is commented, uncomment cvtColor above to restore grayImg

}

//calculates white balanced saturation image
Mat DkSegmentationSatIpk::computeSatImg() {

	Mat rgb32F;
	Scalar meanRgb;

	DkTimer dt;
	rgbImg.convertTo(rgb32F, CV_32F, 1.0f/255.0f,0);	//imgs doesn't contain CV_32F of rgb image -> create it

	std::vector<Mat> rgbCh;
	split(rgb32F, rgbCh);		// re-allocation -> huge memory consumption
	rgb32F.release();			// -> release

	// filter each channel
	Mat tmp;
	Mat g = DkIP::get1DGauss(0.3f);	// 3x3 gaussian
	filter2D(rgbCh[0], rgbCh[0], -1, g, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
	filter2D(rgbCh[0], rgbCh[0], -1, g.t(), cv::Point(-1,-1), 0.0, BORDER_REFLECT);

	filter2D(rgbCh[1], rgbCh[1], -1, g, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
	filter2D(rgbCh[1], rgbCh[1], -1, g.t(), cv::Point(-1,-1), 0.0, BORDER_REFLECT);

	filter2D(rgbCh[2], rgbCh[2], -1, g, cv::Point(-1,-1), 0.0, BORDER_REFLECT);
	filter2D(rgbCh[2], rgbCh[2], -1, g.t()), cv::Point(-1,-1), 0.0, BORDER_REFLECT;

	meanRgb[0] = (double)DkIP::statMomentMat(rgbCh[0], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up
	meanRgb[1] = (double)DkIP::statMomentMat(rgbCh[1], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up
	meanRgb[2] = (double)DkIP::statMomentMat(rgbCh[2], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up

	float max, min;
	Mat satImg(rgbCh[0].size(), rgbCh[0].type());

	//white balance
	for(int i = 0; i < rgbCh[0].rows; i++)
	{
		float *ptrR = rgbCh[0].ptr<float>(i);
		float *ptrG = rgbCh[1].ptr<float>(i);
		float *ptrB = rgbCh[2].ptr<float>(i);
		float *ptrS = satImg.ptr<float>(i);

		for(int j = 0; j < rgbCh[0].cols; j++) {
			ptrR[j] = ptrR[j] > (float)meanRgb[0] ? 1.0f : ptrR[j]/((float)meanRgb[0]);
			ptrG[j] = ptrG[j] > (float)meanRgb[1] ? 1.0f : ptrG[j]/((float)meanRgb[1]);
			ptrB[j] = ptrB[j] > (float)meanRgb[2] ? 1.0f : ptrB[j]/((float)meanRgb[2]);
			//saturation of IHLS color space
			max = ptrR[j] > ptrG[j] ? ptrR[j] : ptrG[j];
			max = max > ptrB[j] ? max : ptrB[j];

			min = ptrR[j] < ptrG[j] ? ptrR[j] : ptrG[j];
			min = min < ptrB[j] ? min : ptrB[j];

			ptrS[j] = (max - min);

		}
	}
	rgbCh[0].release();
	rgbCh[1].release();
	rgbCh[2].release();
	rgbCh.clear();
	mout << "[DkSegmentation] sat computation" << dt << dkendl;

	return satImg;
}

Mat DkSegmentationSatIpk::enhanceSatImg(Mat satImg) {

	Mat tmpseg;
	segImg.convertTo(tmpseg, CV_32F, 1.0f/255.0f, 0);

	double sigma = 7;
	int kSize = cvRound(cvCeil(sigma*3)*2+1);
	Mat intSeg;
	integral(tmpseg, intSeg);
	tmpseg = DkIP::convolveIntegralImage(intSeg, kSize, 0, DkIP::DK_BORDER_ZERO);
	intSeg.release();	// early release

	DkIP::invertImg(tmpseg);
	normalize(tmpseg, tmpseg, 1, 0, NORM_MINMAX);	//CHANGE test if normalization is necessary
	//DkIP::imwrite("tmpsegNeu.png", tmpseg);

	//weight of the sat image: regions segmented in the step before have a lower weight....
	//otherwise black/gray values effect the contrast enhancement in the next step
	Mat stmp = Mat(satImg.size(), CV_32FC1);
	for(int i = 0; i < satImg.rows; i++)
	{
		float *ptrS = satImg.ptr<float>(i);
		float *ptrtmpSeg = tmpseg.ptr<float>(i);
		float *ptrStmp = stmp.ptr<float>(i);

		for(int j = 0; j < satImg.cols; j++) {
			ptrStmp[j] = (ptrtmpSeg[j] * ptrS[j] * -1.0f +1.0f);
		}
	}
	satImg.release();	// early release
	tmpseg.release();	// early release

	Mat tmpResize, satresize;

	//copy smoothed sat image for background gradient normalization
	resize(stmp, tmpResize, Size(),0.1f,0.1f,INTER_LINEAR);
	//GaussianBlur(tmpResize, tmpResize, Size(), 3.0f, 3.0f);
	tmpResize = DkIP::dilateImage(tmpResize, 3, DkIP::DK_SQUARE);
	//tmpResize = DkIP::erodeImage(tmpResize, 3, DK_SQUARE);
	resize(tmpResize, satresize, stmp.size(),0,0,INTER_LINEAR);
	tmpResize.release();
	satresize.convertTo(satresize, CV_32F, 1.0f/255.0f);


	Mat grayImgTmp;
	//stretch the histogram to get a better contrast for low saturated colors
	//compute only sat image if confidence is greater than confidenceThreshold
	//if sensitivity is 1.0f the saturation image is not affected
	if (sensitivity != 1.0f) {
		Scalar bgdS, meanS, stdS;
		//Scalar bgdG, meanG, stdG, meanGCont, stdGCont;


		meanStdDev(stmp, meanS, stdS, erodedMask);

		Mat histogramSat = DkIP::computeHist(stmp, erodedMask);
		stmp.convertTo(stmp, CV_8U, 255);

		float cnt=0;
		float i=255;									//CHANGE: compatible with the old version - check why not 255 (last bin is skipped)
		float px = (float)countNonZero(erodedMask);
		float *hist = histogramSat.ptr<float>();
		while (i>0.0f && cnt/px < sensitivity) {
			cnt += hist[(int)i];
			i--;
		}

		if (i>205.0f) i=205.0f;						//BUG: < in the old version 
		float bgdThresh;

		//bgdThresh = 255.0f*((float)meanS[0] - 2.0f*(float)stdS[0]);
		bgdThresh = 255.0f*(float)meanS[0] - 2.0f*(float)stdS[0];
		if ((i-bgdThresh) > 0)
			i=0.0f;

		//printf("bgdThresh:  %f  i: %f\n", bgdThresh, i);
		//image is inverted! -> all values greater than i are assumed as outliers (0.5% of all pixels)
		// the dynamic range is changed according the median (assumed as background and i)
		//only the range "above" the background is normalized to avoid enhancing noise
		for(int r = 0; r < stmp.rows; r++)
		{
			unsigned char *ptrS = stmp.ptr<unsigned char>(r);
			for(int c = 0; c < stmp.cols; c++) {
				if (ptrS[c] < i)
					ptrS[c] = 0;
				if (ptrS[c] >= i && ptrS[c] < bgdThresh) 
					ptrS[c] = (unsigned char)(((float)ptrS[c]-i)*(bgdThresh)/(bgdThresh-i));
			}
		}
		normalize(grayImg,grayImgTmp, 255,0, NORM_MINMAX,-1,mask);	//CHANGE: ?: erodedMask in the old version, but then grayImg must be copied.
	}


	//DkIP::imwrite("stmpSatEnhBefore1054.png", stmp);

	normalize(satresize, satresize, 1.0f, 0, NORM_MINMAX,-1,mask);
	//DkIP::invertImg(satresize, mask);	//satresize = max(satResize) - satResize
	//satresize.convertTo(satresize, CV_8U, 255);
	//stmp += satresize;
	//background gradient normalization
	for(int r = 0; r < satresize.rows; r++)
	{
		unsigned char *ptrS = stmp.ptr<unsigned char>(r);
		unsigned char *ptrMask = mask.ptr<unsigned char>(r);
		float *ptrSResize = satresize.ptr<float>(r);
	
		for(int c = 0; c < satresize.cols; c++) {
			if (ptrMask[c]!=0) {
				//invert satResize and add satResize as offset to the enhanced sat image (stmp)
				// test if an overflow occure -> val is 255;
				ptrS[c] = ((1.0f-ptrSResize[c])*255.0f + (float)ptrS[c]) > 255.0f ? 255 : ptrS[c]+(unsigned char)((1.0f-ptrSResize[c])*255.0f);
			}
		}
	}

	//DkUtils::getMatInfo(satresize, "satresize");
	//DkUtils::getMatInfo(stmp, "stmp");
	//DkIP::imwrite("resizedSat1072.png", satresize);
	//DkIP::imwrite("stmpSatEnh1054.png", stmp);
	//DkIP::imwrite("grayImgNew1054.png", grayImgTmp);

	cv::min(grayImgTmp, stmp, grayImgTmp);	// diem opencv 2.1  -> bug?
	stmp.release(); // early release
	//DkIP::imwrite("grayImgEnhNew1056.png", grayImgTmp);

	return grayImgTmp;
}


float DkSegmentationSatIpk::getConfidence() {

	float fullPaper = 0.18f;

	//confidence already calculated
	if (confidence >= 0.0f)
		return confidence;

	DkTimer dt = DkTimer();

	//otherwise calculate confidence
	if (segSatImg.empty()) {
		
		//classical Segmentation needed for the removal of border pixel
		convertInputData();
		computeSat(false);

		// >DIR: otherwise we see each call to getConfidence (e.g. getSegmented() in 2 ms) [17.10.2012 markus]
		mout << "[DkSegmentation] getSegmented computed in: " << dt << dkendl;
	}

	segmentedPx = countNonZero(segSatImg);
	maskPx = countNonZero(mask);

	if (maskPx == 0) {
		confidence = 0.0f;

		if (mask.empty()) {
			DkUtils::printDebug(DK_WARNING, "[DkSegmentation] mask image is empty, where it should not be (getConfidence)\n");
		}

	} else {
		float segRatio = (float)segmentedPx / (float)maskPx;

		segRatio = segRatio-fullPaper < 0 ? 1.0f : 1.0f - (segRatio-fullPaper)/(1.0f-fullPaper);

		//if (segRatio != 1.0f) {
		//	printf("---------------------------\n");
		//	printf("segRatio != 1.0f\n");
		//	printf("---------------------------\n");
		//}
		confidence = segRatio * (float)meanContrast[0];
	}

	return confidence;
}

void DkSegmentationSatIpk::saveDebugImg(Mat img, std::string path, std::string filename) {
	std::string saveName, saveFull;

	getConfidence();

	saveName = "_" + getName() + "_DkSegImg";
	//saveName = "_segDkSatFilt";
	saveFull = path + DkUtils::createFileName(filename, saveName);

	int rows = segImg.rows;
	int cols = segImg.cols;
	Mat grayimg;
	cvtColor(img, grayimg, CV_RGB2GRAY);

	Mat result = Mat(rows, cols*3, CV_32FC1);
	Mat a = result(Range::all(), Range(0, cols));
	grayimg.convertTo(a, CV_32F);
	Mat b = result(Range::all(), Range(cols, cols+cols));
	//segImg*=255;
	segImg.convertTo(b, CV_32F);
	Mat c = result(Range::all(), Range(cols+cols, cols+cols+cols));
	//segSatImg*=255;
	segSatImg.convertTo(c, CV_32F);

	std::string txt = "Confidence: " + DkUtils::stringify(confidence*100);
	int baseline;
	float xW = (float)cols;
	float yH = 0.25f*(float)rows;
	int scale = 0;
	Size s;

	do {
		scale++;
		s = getTextSize(txt, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, 5, &baseline);
	} while((s.width < xW) && (s.height < yH));
	if (scale > 1) scale--;
	s = getTextSize(txt, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, 5, &baseline);

	Point tC = Point(0,s.height);
	putText(result, txt, tC, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, Scalar(255, 255, 255), 5, CV_AA);
	putText(result, txt, tC, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, Scalar(0, 0, 0), 1, CV_AA);

	if (!segImg.empty()) {
		//printf("..saving image: %s\n", saveSeg.c_str());
		//DkIP::imwrite(saveFull.c_str(), img);
		DkIP::imwrite(saveFull.c_str(), result);
		std::string msg = "[" + getName()  + "] debug images saved...";
		DkUtils::printDebug(DK_INFO, msg.c_str());
		//printf("done...\n", saveSeg.c_str());

	} else
		throw DkMatException("empty mat", __LINE__, __FILE__);
}

std::string DkSegmentationSatIpk::toString() const {

	std::string msg = "";
	msg += DkSegmentationFgdIpk::toString();
	msg += "    confidence threshold: " + DkUtils::stringify(confidenceThreshold) + "         sensitivity: " + DkUtils::stringify(sensitivity) + "\n";

	return msg;
}

void DkSegmentationSatIpk::release() {

	DkSegmentationFgdIpk::release();

	segSatImg.release();
}
//-------------------------------------------------------------------------------------------------------------------------------------------

Mat DkSegmentationDibcoTest::getSegmented() {
	if (segImg.empty()) {

		DkTimer dt = DkTimer();
		convertInputData();		//converts rgb to grayvalue image, and erodes the mask
		computeSuDibcoTest(segImg);
		//DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax Su image computed in: %s\n", dt.getTotal().c_str());
		mout << "[DkSegmentation] localMinMaxSu image computed in: " << dt << dkendl;

	}

	return segImg;
}

float DkSegmentationDibcoTest::getStrokeWidth(Mat contrastImg) {

	return 4.0f;
}

void DkSegmentationDibcoTest::calcFilterParams(int &filterS, int &Nm) {

	//if (strokeW >= 4.5) strokeW = 3.0;
	//filterS = cvRound(strokeW*10);
	//if ((filterS % 2) != 1) filterS+=1;
	//Nm = cvFloor(strokeW*10);

	filterS = cvRound(strokeW);
	filterS = (filterS % 2) != 1 ? filterS+1 : filterS;
	//Nm = filterS;
	Nm = 0;
}

float DkSegmentationDibcoTest::thresholdVal(float *mean, float *std) {
	//return (*mean+*std/2);
	float thr, scal;
	scal = 0.5f;
	thr = (*mean) + (*std)*scal;
	return thr;
}

void DkSegmentationDibcoTest::computeSuDibcoTest(Mat &resultSegImg) {

	contrastImg = computeContrastImg();
	binContrastImg = computeBinaryContrastImg(contrastImg);

	if (releaseDebug == DK_RELEASE_IMGS)
		contrastImg.release();

	Mat tmp1;
	strokeW = getStrokeWidth(tmp1);

	// now we need a 32F image
	Mat tmp;	
	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);		//don't use image source class -> enables to set grayimg to a different value (MinMaxSatSeg)


	//tmp = DkIP::erodeImage(tmp, 5, DK_SQUARE);
	computeThrImg(tmp, binContrastImg, thrImg, resultSegImg);					//compute threshold image

	//DkIP::imwrite("resultSegImg611.png", resultSegImg);
	//DkIP::imwrite("resultThrImg612.png", tmp<=thrImg);
	DkIP::imwrite("resultThrImg623.png", thrImg);

	Mat tmpWhite;
	resultSegImg.convertTo(tmpWhite, CV_32FC1, 1.0f/255.0f);
	DkIP::invertImg(tmpWhite);
//	thrImg += (1.0f-tmpWhite);
	thrImg += tmpWhite;
	DkIP::invertImg(tmpWhite);
	thrImg = DkIP::erodeImage(thrImg, 3, DkIP::DK_SQUARE);
	thrImg = thrImg.mul(tmpWhite);

	//DkIP::imwrite("tmpwhite1334.png", tmpWhite);
	DkIP::imwrite("resultThrImg1334.png", thrImg);

	if (releaseDebug == DK_RELEASE_IMGS)
		binContrastImg.release();

	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition

	//DkIP::imwrite("resultImg616.png", resultSegImg);

	if (releaseDebug == DK_RELEASE_IMGS) {
		thrImg.release();
		//erodedMask.release();
	}
}


