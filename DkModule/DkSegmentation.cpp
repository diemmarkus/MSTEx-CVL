/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/


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
	strokeW = 11;	// we take a fixed stroke width for the MSTEx challenge
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

	DkIP::imwrite("sImg.png", segImg);

	float sumArea = 0;

	for (int idx = 0; idx < segFilter.getSize(); idx++) {
		sumArea += abs(segFilter.getBlob(idx).getArea());
	}

	moutc << sumArea/segFilter.getSize()*0.5 << " fgd filter size" << dkendl;
	moutc << sumArea << " sumArea" << dkendl;

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

		// filter y-coordinates
		filter2D(meanImg, meanImg, CV_32FC1, sumKernel);
		filter2D(stdImg, stdImg, CV_32FC1, sumKernel);
		filter2D(intContrastBinary, intContrastBinary, CV_32FC1, sumKernel);

		// filter x-coordinates
		sumKernel = sumKernel.t();
		filter2D(meanImg, meanImg, CV_32FC1, sumKernel);
		filter2D(stdImg, stdImg, CV_32FC1, sumKernel);
		filter2D(intContrastBinary, intContrastBinary, CV_32FC1, sumKernel);

	}
	else {

		// is way faster than the filter2D function for kernels > 7
		Mat intImg;
		integral(meanImg, intImg);
		meanImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DkIP::DK_BORDER_ZERO);

		// compute the standard deviation image
		integral(stdImg, intImg);
		stdImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DkIP::DK_BORDER_ZERO);
		intImg.release(); // early release

		integral(contrastBin32F, intContrastBinary);
		contrastBin32F.release();
		intContrastBinary = DkIP::convolveIntegralImage(intContrastBinary, filtersize, 0, DkIP::DK_BORDER_ZERO);
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
	filter2D(rgbCh[0], rgbCh[0], -1, g);
	filter2D(rgbCh[0], rgbCh[0], -1, g.t());

	filter2D(rgbCh[1], rgbCh[1], -1, g);
	filter2D(rgbCh[1], rgbCh[1], -1, g.t());

	filter2D(rgbCh[2], rgbCh[2], -1, g);
	filter2D(rgbCh[2], rgbCh[2], -1, g.t());

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




//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
// old class
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//bool DkSegmentation::empty() {
//
//	return (rgbImg.empty() && mask.empty() && imgs == 0 && segImg.empty() && segSatImg.empty());
//
//}
//
//void DkSegmentation::setfgdEstFilterSize(int s) {
//	if (s >= 3)
//		fgdEstFilterSize = s;
//	else {
//		std::string msg = "Filter Size must be >= 3, it is: " + DkUtils::stringify(s) + "\n";
//		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
//	}
//}
//
//void DkSegmentation::setSensitivity(float sens) {
//	if (sens <= 1.0f && sens > 0.0f)
//		sensitivity = 1.0f - sens/100.0f;
//	else {
//		std::string msg = "Sensitivity must be ]0 1], it is: " + DkUtils::stringify(sens) + "\n";
//		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
//	}
//}
//
//void DkSegmentation::setConfidenceThresh(float th) {
//	if (th >= 0.0f || th <= 1.0f)
//		confidenceThreshold = th;
//	else {
//		std::string msg = "Threshold must be [0 1], it is: " + DkUtils::stringify(th) + "\n";
//		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
//	}
//}
//
//void DkSegmentation::setRemoveBorderSize(int erodeMaskSize) {
//
//	if (erodeMaskSize >= 0)
//		this->erodeMaskSize = erodeMaskSize;
//	else {
//		std::string msg = "Filter Size must be >= 0, it is: " + DkUtils::stringify(erodeMaskSize) + "\n";
//		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
//	}
//
//}
//
//void DkSegmentation::setSigmSlope(float s) {
//	if (s > 1)
//		sigmSlope = s;
//	else {
//		std::string msg = "Sigmoid Slope must be > 1, it is: " + DkUtils::stringify(s) + "\n";
//		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
//	}
//
//}
//
//void DkSegmentation::init() {
//
//	//initialization segmentation params
//	fgdEstFilterSize = 32;
//	sigmSlope = 15.0f;
//
//	sensitivity = 0.995f;
//	//sensitivity = 1.0f;
//	confidenceThreshold = 0.1f;
//
//	confidence = -1.0f;
//	meanContrast[0] = -1.0f;
//	meanContrast[1] = -1.0f;
//	meanContrast[2] = -1.0f;
//
//	erodeMaskSize = DK_ERODED_MASK_SIZE;
//
//	className = "DkSegmentation";
//	modeS = DK_IPK;
//}
//
//void DkSegmentation::convertInputData() {
//	checkInput();
//
//	if (imgs == 0) {
//		cvtColor(rgbImg, grayImg, CV_RGB2GRAY);
//		//erodedMask = mask.clone();
//		erodedMask = DkIP::fastErodeImage(mask, cvRound(erodeMaskSize));
//	} else {
//		grayImg = imgs->getGrayImg().clone();
//
//		if (erodeMaskSize == DK_ERODED_MASK_SIZE) {
//			//erodedMask = mask.clone();
//			erodedMask = imgs->getErodedMask().clone();
//		} else {	//erode mask with current filter size
//			erodedMask = DkIP::fastErodeImage(imgs->getMask(), cvRound(erodeMaskSize));
//			//erodedMask = mask.clone();
//		}
//	}
//}
//
//DkSegmentation::DkSegmentation(){
//
//	imgs = 0;
//	init();
//}
//
//DkSegmentation::DkSegmentation(Mat rgb, Mat mask) {
//
//	this->rgbImg = rgb;
//	this->mask = mask;
//	imgs = 0;
//
//	init();
//}
//
//DkSegmentation::DkSegmentation(DkImageSource *imgs)  : DkModule(imgs) {
//
//	rgbImg = imgs->getRgbImg();
//	mask = imgs->getMask();
//
//	init();
//}
//
//void DkSegmentation::checkInput() {
//
//	if (rgbImg.empty()) throw DkMatException("empty mat", __LINE__, __FILE__);
//	if (rgbImg.channels() != 3) throw DkMatException("not a 3 channel input image", __LINE__, __FILE__);
//	if (rgbImg.depth() != CV_8U) throw DkMatException("not a CV_8U input image", __LINE__, __FILE__);
//	if (mask.empty()) throw DkMatException("empty mat", __LINE__, __FILE__);
//	if (mask.depth() != CV_8U) throw DkMatException("not a CV_8U input image", __LINE__, __FILE__);
//
//	// check if the mask fits to the image
//	if (rgbImg.size() != mask.size()) {
//		std::string msg = "Image size does not correspond to the mask's size (" + DkVector(rgbImg.size()).toString() + " != " + DkVector(mask.size()).toString() + ")";
//		throw DkMatException(msg, __LINE__, __FILE__);
//	}
//}
//
//Mat DkSegmentation::getSegmentedLocalMinMaxSu() {
//
//	if (segImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		checkInput();
//		convertInputData();		//converts rgb to grayvalue image, and erodes the mask
//		computeLocalMinMaxSu(segImg);
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax Su image computed in: %s\n", dt.getTotal().c_str());
//
//	}
//
//	return segImg;
//}
//
//
//Mat DkSegmentation::getSegmentedLocalMinMax() {
//
//	if (segImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		checkInput();
//		convertInputData();		//converts rgb to grayvalue image, and erodes the mask
//		computeLocalMinMaxSeg(segImg);
//
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] localMinMax SegFgd image computed in: %s\n", dt.getTotal().c_str());
//	}
//
//	return segImg;
//}
//
//Mat DkSegmentation::getSegmentedLocalMinMaxSat() {
//	//classical Segmentation needed for the removal of border pixel
//
//	DkTimer dt = DkTimer();
//
//	segSatImg.release();
//	checkInput();
//	convertInputData();
//
//	computeLocalMinMaxSatSeg(true);
//
//	DkUtils::printDebug(DK_MODULE, "[DkSegmentation] SegSat computed in: %s\n", dt.getTotal().c_str());
//
//	return segSatImg;
//}
//
//Mat DkSegmentation::getSegmentedScaleSu() {
//	//classical Segmentation needed for the removal of border pixel
//	modeS = DK_SEGSU;
//	if (scaleSegImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		scaleSegImg = computeLocalMinMaxScale(modeS);
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] scale SegSu computed in: %s\n", dt.getTotal().c_str());
//	}
//
//	return scaleSegImg;
//}
//
//Mat DkSegmentation::getSegmentedScaleSuFgd() {
//	modeS = DK_SEGSUFGD;
//	if (scaleSegImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		scaleSegImg = computeLocalMinMaxScale(modeS);
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] scale SegSu computed in: %s\n", dt.getTotal().c_str());
//	}
//
//	return scaleSegImg;
//}
//
//Mat DkSegmentation::getSegmentedScaleFgd() {
//	//classical Segmentation needed for the removal of border pixel
//	modeS = DK_SEGFGD;
//	if (scaleSegImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		scaleSegImg = computeLocalMinMaxScale(modeS);
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] scale SegFgd computed in: %s\n", dt.getTotal().c_str());
//	}
//
//	return scaleSegImg;
//}
//
//Mat DkSegmentation::getSegmentedScaleSat() {
//	//classical Segmentation needed for the removal of border pixel
//	modeS = DK_SEGSAT;
//	if (scaleSatImg.empty()) {
//
//		DkTimer dt = DkTimer();
//		scaleSatImg = computeLocalMinMaxScale(modeS);
//		DkUtils::printDebug(DK_MODULE, "[DkSegmentation] scale SegSat computed in: %s\n", dt.getTotal().c_str());
//	}
//
//	return scaleSatImg;
//}
//
//
//
//
//Mat DkSegmentation::getSegmented() {
//
//	getConfidence();
//
//	if (confidence > confidenceThreshold)
//		return segSatImg;
//	else
//		return segImg;
//}
//
//void DkSegmentation::filterMinMax(int size) {
//
//	getSegmentedLocalMinMax();
//
//	DkBlobs<DkAttr> segFilter(segImg);
//	segFilter.imgFilterArea(size);
//
//}
//
//void DkSegmentation::filterMinMaxSat(int size) {
//
//	getSegmentedLocalMinMaxSat();
//
//	DkBlobs<DkAttr> segFilter(segSatImg);
//	segFilter.imgFilterArea(size);
//}
//
//
//void DkSegmentation::filterMinMaxScale(int size) {
//
//	getSegmentedScaleFgd();
//
//	DkBlobs<DkAttr> segFilter(scaleSegImg);
//	segFilter.imgFilterArea(size);
//}
//
//
//void DkSegmentation::filterMinMaxScaleSat(int size) {
//
//	getSegmentedScaleSat();
//
//	DkBlobs<DkAttr> segFilter(scaleSatImg);
//	segFilter.imgFilterArea(size);
//}
//
////The original Su, Lu, Tan version
//void DkSegmentation::computeLocalMinMaxSu(Mat &resultSegImg) {
//
//	contrastImg = computeContrastImg();
//	//DkIP::imwrite("contrastImg1452.png", contrastImg);
//	binContrastImg = computeBinaryContrastImg(contrastImg);
//	//DkIP::imwrite("binContrast1454.png", binContrastImg, true);
//	if (modeS==DK_IPK) {
//		////old ipk version:
//		//strokeW = getStrokeWidth(contrastImg);
//		////strokewidth version
//		//Mat tmp1;
//		//binContrastImg.convertTo(tmp1, CV_32F, 1.0f/255.0f);
//		//tmp1 = contrastImg.mul(tmp1);
//		//strokeW = getStrokeWidth(tmp1);
//		//tmp1.release();
//		////test->constant strokewidth
//		strokeW = 4.0f;
//	} else if (modeS==DK_SEGSU) {
//		Mat tmp1;
//		binContrastImg.convertTo(tmp1, CV_32F, 1.0f/255.0f);
//		tmp1 = contrastImg.mul(tmp1);
//		strokeW = getStrokeWidth(tmp1);
//		tmp1.release();
//	} else {
//		strokeW = 5.0f;
//	}
//
//	// now we need a 32F image
//	Mat tmp;	
//	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);		//don't use image source class -> enables to set grayimg to a different value (MinMaxSatSeg)
//
//	computeThrImg(tmp, binContrastImg, thrImg, resultSegImg);					//compute threshold image
//	//DkIP::imwrite("thrImg1484.png", thrImg);
//	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition
//	//bitwise_and(resultSegImg, erodedMask, resultSegImg);		//combine with Nmin condition
//}
//
//void DkSegmentation::computeLocalMinMaxSeg(Mat &resultSegImg) {
//
//	computeLocalMinMaxSu(resultSegImg);
//
//	// now we need a 32F image
//	Mat tmp;
//	grayImg.convertTo(tmp, CV_32F, 1.0f/255.0f);
//
//	fgdEstImg = computeMeanFgdEst(tmp);					//compute foreground estimation
//
//	Mat tmpMask;
//	threshold(thrImg, tmpMask, 0, 1.0, CV_THRESH_BINARY);
//	Mat histogram = DkIP::computeHist(thrImg, tmpMask);		//weight gray values with sigmoid function according
//	tmpMask.release();
//
//	double l = DkIP::getThreshOtsu(histogram, 0.0007)/255.0f;		//sigmoid slope, centered at l according text estimation
//	//sigmSlope = 15.0f;
//	float sigmaSlopeTmp = sigmSlope/255.0f;
//
//	float fm[256];
//	for (int i = 0; i < 256; i++)
//		fm[i] = 1.0f/(1.0f + std::exp(((i/255.0f) - (float)l) * (-1.0f/(sigmaSlopeTmp))));
//
//	for(int i = 0; i < tmp.rows; i++)
//	{
//		float *ptrGray = tmp.ptr<float>(i);
//		float *ptrThr = thrImg.ptr<float>(i);
//		float *ptrFgdEst = fgdEstImg.ptr<float>(i);
//		unsigned char *ptrMask = erodedMask.ptr<unsigned char>(i);
//
//		for(int j = 0; j < tmp.cols; j++, ptrGray++, ptrThr++, ptrFgdEst++, ptrMask++) {
//			*ptrGray = fm[cvRound(*ptrGray*255.0f)];
//			*ptrThr = (*ptrMask != 0) ? *ptrThr * (*ptrFgdEst) : 0.0f;
//		}
//	}
//
//	bitwise_and(resultSegImg, tmp <= (thrImg), resultSegImg);		//combine with Nmin condition
//
//	if (releaseDebug == DK_RELEASE_IMGS) {
//		fgdEstImg.release();
//		thrImg.release();
//		contrastImg.release();
//		binContrastImg.release();
//	}
//
//}
//
//void DkSegmentation::computeLocalMinMaxSatSeg(bool computeAlways) {
//
//	if (segImg.empty())
//		computeLocalMinMaxSeg(segImg);
//
//	if (((float)meanContrast[0] > confidenceThreshold) || computeAlways) {
//
//		Mat satImg = computeSatImg();		//compute the saturation image
//		satImg = enhanceSatImg(satImg);		//enhance it according defined thresholds
//
//		grayImg = satImg;					//CV_8U
//
//		computeLocalMinMaxSeg(segSatImg);
//
//		getConfidence();
//													//save time, but changes the input grayvalue image
//		//cvtColor(rgbImg, grayImg, CV_RGB2GRAY);	//restore grayValue image, probably skip to save memory?
//		//bitwise_and(segSatImg, erodedMask, segSatImg);
//	
//	} else {
//		confidence = (float)meanContrast[0];
//		segSatImg = segImg;
//	}
//
//	if (releaseDebug == DK_RELEASE_IMGS)
//		clearDebugImg();
//
//	grayImg.release(); //if this is commented, uncomment cvtColor above to restore grayImg
//
//}
//
//
//Mat DkSegmentation::computeLocalMinMaxScale(int mode) {
//
//	checkInput();
//	convertInputData();
//	releaseDebug = DK_KEEP_IMGS;
//	//DkTimer dt;
//
//	DkDogDetector dogM;
//
//	if (mode == DK_SEGSAT) {
//		computeLocalMinMaxSeg(segImg);						//needed for meanContrast and SegImg
//		//if SatSeg is decided, calc the enhanced sat image and set the grayimg
//		if ((float)meanContrast[0] > confidenceThreshold) {
//			Mat satImg = computeSatImg();		//compute the saturation image
//			satImg = enhanceSatImg(satImg);		//enhance it according defined thresholds
//			grayImg = satImg;
//			segImg.release();
//		}
//	}
//
//	Mat tmpGrayImg = grayImg.clone();
//	grayImg.convertTo(grayImg, CV_32F);
//
//	dogM = DkDogDetector(grayImg, mask);
//	//dogM.o = 4;												//TODO: determine (state a reason) # octaves; #scales
//	Mat tmpFgd = Mat();
//	Mat tmpImg = Mat();
//	Mat rgbTmp = rgbImg.clone();
//	int tmpFgdEstFilterSize = fgdEstFilterSize;
//	Mat segTmp;
//	vector<vector<Mat> > scales = dogM.getScaleSpace();
//	double minVal, maxVal;
//	minMaxLoc(mask, &minVal, &maxVal);
//	vector<Mat> scaleMasks;
//	if (minVal != maxVal)
//		scaleMasks = dogM.getScaleSpaceMasks();
//
//	//for (int oIdx = (int)scales.size()-1; oIdx >= 0; oIdx--) {
//	for (int oIdx = (int)scales.size()-1; oIdx >= 0; oIdx--) {			
//
//		vector<Mat> dogO = scales[oIdx];
//		//printf("sidx: %i", (int)dogO.size()-1);
//		Mat eMask;
//		if (minVal != maxVal) 
//			eMask = scaleMasks[oIdx];
//		else {
//			eMask = Mat(dogO[0].size(), CV_8UC1);
//			resize(mask, eMask, eMask.size(), 0, 0, INTER_NEAREST);
//		}
//
//		for (int sIdx = (int)dogO.size()-1; sIdx >= 0; sIdx--) {
//
//			////old version without masks of DkDogDetector
//			//Mat eMask = Mat(dogO[sIdx].size(), CV_8UC1);
//			//resize(erodedMask, eMask, eMask.size(), 0, 0, INTER_NEAREST);
//
//			Mat gImg = dogO[sIdx];
//			normalize(gImg, gImg, 0.0f, 1.0f, NORM_MINMAX, -1);
//
//			float scale = (float)gImg.rows / (float)tmpGrayImg.rows;
//			fgdEstFilterSize = (int)(scale * (float)tmpFgdEstFilterSize);
//
//			gImg.convertTo(gImg, CV_8UC1, 255);
//			segTmp = computeThreshScale(gImg, eMask, segTmp, mode);
//
//			//for mode SEGSAT the internal memory must be cleared,
//			//otherwise in computeLocalMinMaxSat the segImg is referenced (wrong size)
//
//			Mat tmpMask;
//			eMask.convertTo(tmpMask, CV_32F, 1.0f/255.0f);
//			Mat tmpG = dogO[sIdx];
//			normalize(tmpG, tmpG, 0.0f, 1.0f, NORM_MINMAX, -1);
//			tmpG = tmpG.mul(tmpMask);
//			gImg = tmpG;
//
//			if (oIdx == 0 && sIdx == 3 && mode != DK_SEGSU) {
//				tmpFgd = segTmp.clone();
//				tmpImg = dogO[sIdx].clone();
//			}
//			//DkUtils::getMatInfo(segTmp, "segTmp");
//			//DkIP::imwrite("dogO.png", gImg);
//			//DkIP::imwrite("segImg.png", segTmp);
//			//printf("waiting for input....\n");
//			//printf("oIdx: %i sIdx: %i\n", oIdx, sIdx);
//			//waitKey();
//			//DkUtils::printDebug(DK_MODULE, "scale %i computed in %s\n", sIdx, dt.getIvl().c_str());
//
//			contrastImg.release();									//contrast image binary
//			fgdEstImg.release();										//foreground estimation image
//			thrImg.release();											//the threshold image
//		}
//
//		//DkUtils::printDebug(DK_MODULE, "octave %i computed in %s\n\n", oIdx, dt.getIvl().c_str());
//	}
//
//	grayImg = tmpGrayImg;
//	fgdEstFilterSize = tmpFgdEstFilterSize;
//	
//	//foreground estimation
//	if (mode!=DK_SEGSU) {
//		normalize(tmpImg, tmpImg, 1.0f, 0.0f, NORM_MINMAX);
//	//	grayImg.convertTo(grayImg, CV_32F, 1.0f/255.0f);
//	//	normalize(grayImg, grayImg, 1.0f, 0.0f, NORM_MINMAX);
//	//	grayImg.convertTo(grayImg, CV_8U, 255);
//	}
//
//	//segTmp = computeThreshScale(grayImg, erodedMask, segTmp, DK_SEGFGD);
//	segTmp = computeThreshScale(grayImg, erodedMask, segTmp, mode, tmpImg, tmpFgd);
//	//DkIP::imwrite("dogO.png", grayImg);
//	//DkIP::imwrite("segImg.png", segTmp);
//	//printf("waiting for input....\n");
//	//waitKey();
//
//
//	if (releaseDebug == DK_RELEASE_IMGS) {
//		fgdEstImg.release();
//		thrImg.release();
//		contrastImg.release();
//	}
//
//	return segTmp;
//}
//
////calculates white balanced saturation image
//Mat DkSegmentation::computeSatImg() {
//
//	Mat rgb32F;
//	Scalar meanRgb;
//
//	rgbImg.convertTo(rgb32F, CV_32F, 1.0f/255.0f,0);	//imgs doesn't contain CV_32F of rgb image -> create it
//
//	std::vector<Mat> rgbCh;
//	split(rgb32F, rgbCh);		// re-allocation -> huge memory consumption
//	rgb32F.release();			// -> release
//
//	// filter each channel
//	Mat tmp;
//	Mat g = DkIP::get1DGauss(0.3f);	// 3x3 gaussian
//	filter2D(rgbCh[0], rgbCh[0], -1, g);
//	filter2D(rgbCh[0], rgbCh[0], -1, g.t());
//
//	filter2D(rgbCh[1], rgbCh[1], -1, g);
//	filter2D(rgbCh[1], rgbCh[1], -1, g.t());
//
//	filter2D(rgbCh[2], rgbCh[2], -1, g);
//	filter2D(rgbCh[2], rgbCh[2], -1, g.t());
//
//	meanRgb[0] = (double)DkIP::statMomentMat(rgbCh[0], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up
//	meanRgb[1] = (double)DkIP::statMomentMat(rgbCh[1], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up
//	meanRgb[2] = (double)DkIP::statMomentMat(rgbCh[2], mask, 0.5f, 1000) + (double)FLT_MIN;		// edit: diem, max 1000 samples set -> speed-up
//
//	float max, min;
//	Mat satImg(rgbCh[0].size(), rgbCh[0].type());
//
//	//white balance
//	for(int i = 0; i < rgbCh[0].rows; i++)
//	{
//		float *ptrR = rgbCh[0].ptr<float>(i);
//		float *ptrG = rgbCh[1].ptr<float>(i);
//		float *ptrB = rgbCh[2].ptr<float>(i);
//		float *ptrS = satImg.ptr<float>(i);
//
//		for(int j = 0; j < rgbCh[0].cols; j++) {
//			ptrR[j] = ptrR[j] > (float)meanRgb[0] ? 1.0f : ptrR[j]/((float)meanRgb[0]);
//			ptrG[j] = ptrG[j] > (float)meanRgb[1] ? 1.0f : ptrG[j]/((float)meanRgb[1]);
//			ptrB[j] = ptrB[j] > (float)meanRgb[2] ? 1.0f : ptrB[j]/((float)meanRgb[2]);
//			//saturation of IHLS color space
//			max = ptrR[j] > ptrG[j] ? ptrR[j] : ptrG[j];
//			max = max > ptrB[j] ? max : ptrB[j];
//
//			min = ptrR[j] < ptrG[j] ? ptrR[j] : ptrG[j];
//			min = min < ptrB[j] ? min : ptrB[j];
//
//			ptrS[j] = (max - min);
//
//		}
//	}
//	rgbCh[0].release();
//	rgbCh[1].release();
//	rgbCh[2].release();
//	rgbCh.clear();
//
//	return satImg;
//}
//
//Mat DkSegmentation::enhanceSatImg(Mat satImg) {
//
//	Mat tmpseg;
//	segImg.convertTo(tmpseg, CV_32F, 1.0f/255.0f, 0);
//
//	double sigma = 7;
//	int kSize = cvRound(cvCeil(sigma*3)*2+1);
//	Mat intSeg;
//	integral(tmpseg, intSeg);
//	tmpseg = DkIP::convolveIntegralImage(intSeg, kSize, 0, DK_BORDER_ZERO);
//	intSeg.release();	// early release
//
//	DkIP::invertImg(tmpseg);
//	normalize(tmpseg, tmpseg, 1, 0, NORM_MINMAX);	//CHANGE test if normalization is necessary
//
//	//weight of the sat image: regions segmented in the step before have a lower weight....
//	//otherwise black/gray values effect the contrast enhancement in the next step
//	Mat stmp = Mat(satImg.size(), CV_32FC1);
//	for(int i = 0; i < satImg.rows; i++)
//	{
//		float *ptrS = satImg.ptr<float>(i);
//		float *ptrtmpSeg = tmpseg.ptr<float>(i);
//		float *ptrStmp = stmp.ptr<float>(i);
//
//		for(int j = 0; j < satImg.cols; j++) {
//			ptrStmp[j] = (ptrtmpSeg[j] * ptrS[j] * -1.0f +1.0f);
//		}
//	}
//	satImg.release();	// early release
//	tmpseg.release();	// early release
//	//DkIP::imwrite("satImgNeu.png", stmp);
//
//	Mat grayImgTmp;
//	//stretch the histogram to get a better contrast for low saturated colors
//	//compute only sat image if confidence is greater than confidenceThreshold
//	//if sensitivity is 1.0f the saturation image is not affected
//	if (sensitivity != 1.0f) {
//		Scalar bgdS, meanS, stdS;
//		//Scalar bgdG, meanG, stdG, meanGCont, stdGCont;
//
//		meanStdDev(stmp, meanS, stdS, erodedMask);
//
//		Mat histogramSat = DkIP::computeHist(stmp, erodedMask);
//		stmp.convertTo(stmp, CV_8U, 255);
//
//		float cnt=0;
//		float i=254;									//CHANGE: compatible with the old version - check why not 255 (last bin is skipped)
//		float px = (float)countNonZero(erodedMask);
//		float *hist = histogramSat.ptr<float>();
//		while (i>0.0f && cnt/px < sensitivity) {
//			cnt += hist[(int)i];
//			i--;
//		}
//
//		if (i<205.0f) i=205.0f;
//		float bgdThresh;
//
//		bgdThresh = 255.0f*(float)meanS[0] - 2.0f*(float)stdS[0];
//		if ((i-bgdThresh) > 0)
//			i=0.0f;
//	
//		//image is inverted! -> all values greater than i are assumed as outliers (0.5% of all pixels)
//		// the dynamic range is changed according the median (assumed as background and i)
//		//only the range "above" the background is normalized to avoid enhancing noise
//		for(int r = 0; r < stmp.rows; r++)
//		{
//			unsigned char *ptrS = stmp.ptr<unsigned char>(r);
//			for(int c = 0; c < stmp.cols; c++) {
//				if (ptrS[c] < i)
//					ptrS[c] = 0;
//				if (ptrS[c] >= i && ptrS[c] < bgdThresh) 
//					ptrS[c] = (unsigned char)(((float)ptrS[c]-i)*(bgdThresh)/(bgdThresh-i));
//			}
//		}
//		normalize(grayImg,grayImgTmp, 255,0, NORM_MINMAX,-1,mask);	//CHANGE: ?: erodedMask in the old version, but then grayImg must be copied.
//	}
//
//	cv::min(grayImgTmp, stmp, grayImgTmp);	// diem opencv 2.1  -> bug?
//	stmp.release(); // early release
//
//	return grayImgTmp;
//}
//
//
//Mat DkSegmentation::computeThreshScale(Mat grayImgTmp, Mat eMask, Mat parentImg, int mode, Mat weightImg, Mat segFgd) {
//
//
//	grayImg = grayImgTmp.clone();
//	mask = eMask;
//	erodedMask = eMask;
//
//	Mat segTmpScale;
//
//	if (mode == DK_SEGSU || mode == DK_SEGSUFGD) {
//		computeLocalMinMaxSu(segTmpScale);
//	} else if (mode == DK_SEGFGD) {
//		computeLocalMinMaxSeg(segTmpScale);
//	} else if (mode == DK_SEGSAT) {
//		computeLocalMinMaxSeg(segTmpScale);
//	} else {
//		printf("[DkSegmentation] wrong mode!!!\n");
//		return Mat();
//	}
//
//	
//	bool sameSize = (parentImg.size() == grayImg.size()) ? true : false;
//	int pIdx = 0;
//
//	Mat thrImgParent;
//	//float fm[256];
//	//double l;
//	//for (int i = 0; i < 256; i++) fm[i] = 1.0f;
//
//	if (!parentImg.empty()) {
//		thrImgParent = computeThrImgScale(thrImg, parentImg, binContrastImg);
//
//		//Mat tmpMask;
//		//threshold(thrImgParent, tmpMask, 0, 1.0, CV_THRESH_BINARY);
//		//Mat histogram = DkIP::computeHist(thrImgParent, tmpMask);		//weight gray values with sigmoid function according
//		//DkIP::imwrite("tmpMask682.png", tmpMask);
//		//tmpMask.release();
//
//		//l = DkIP::getThreshOtsu(histogram, 0.0007)/255.0f;		//sigmoid slope, centered at l according text estimation
//		//float sigmaSlopeTmp = sigmSlope/255.0f;
//
//		//for (int i = 0; i < 256; i++)
//		//	fm[i] = 1.0f/(1.0f + std::exp(((i/255.0f) - (float)l) * (-1.0f/(sigmaSlopeTmp))));
//	}
//
//// 	printf("-----------------------before mega loop\n");
//// 	DkUtils::getMatInfo(grayImg, "grayImg");
//// 	DkUtils::getMatInfo(thrImg, "thrImg");
//// 	DkUtils::getMatInfo(fgdEstImg, "fgdEstImg");
//// 	DkUtils::getMatInfo(eMask, "eMask");
//// 	DkUtils::getMatInfo(segTmpScale, "segScaleImg");
//// 	DkUtils::getMatInfo(parentImg, "parentImg");
//// 	DkUtils::getMatInfo(thrImgParent, "thrImgParent");
//	//if (!thrImg.empty()) DkIP::imwrite("thrImg654.png", thrImg);
//	//if (!thrImgParent.empty()) DkIP::imwrite("thrImgParent598.png", thrImgParent);
//	//if (!segTmpScale.empty()) DkIP::imwrite("segTmpScale598.png", segTmpScale);
//	//DkIP::imwrite("rgbImg600.png", rgbImg);
//	//DkIP::imwrite("fgdEstImg601.png", fgdEstImg);
//	//DkIP::imwrite("grayImg602.png", grayImg);
//	//if (!thrImgParent.empty() && !fgdEstImg.empty()) DkIP::imwrite("weightImg683.png", fgdEstImg.mul(thrImgParent));
//	//if (!parentImg.empty())	DkIP::imwrite("parentPtr602.png", parentImg);
//// 	printf("-----------------------before mega loop\n");
//
//	grayImg.convertTo(grayImg, CV_32F, 1.0f/255.0f);
//
//	float *ptrFgdEst = 0;
//	float *ptrWeightImg = 0;
//	unsigned char *ptrSegFgd = 0;
//
//	for(int i = 0; i < grayImg.rows; i++) {
//		float *ptrGray = grayImg.ptr<float>(i);
//		float *ptrThr = thrImg.ptr<float>(i);
//		if (!fgdEstImg.empty())						//only for SEGFGD or SEGSAT
//			ptrFgdEst = fgdEstImg.ptr<float>(i);
//		if (!weightImg.empty())
//			ptrWeightImg = weightImg.ptr<float>(i);
//		if (!segFgd.empty())
//			ptrSegFgd = segFgd.ptr<unsigned char>(i);
//		unsigned char *ptrMask = eMask.ptr<unsigned char>(i);
//		unsigned char *ptrSeg = segTmpScale.ptr<unsigned char>(i);
//
//		pIdx = (sameSize) ? i : DkMath::halfInt(i);
//
//		unsigned char *ptrParent = (parentImg.empty() || pIdx >= parentImg.rows) ? 0 : parentImg.ptr<unsigned char>(pIdx);
//		float *ptrThrParent = (parentImg.empty()) ? 0 : thrImgParent.ptr<float>(i);
//
//		for(int j = 0; j < grayImg.cols; j++, ptrSeg++, ptrGray++, ptrThr++, ptrFgdEst++, ptrMask++, ptrThrParent++) {
//
//			if (ptrParent!=0 && ptrThrParent!=0 && (*ptrParent)!=0 && (*ptrSeg)==0) {//parent exists, pixel in parent segmented, but not in current image
//				if (mode == DK_SEGSU || mode == DK_SEGSUFGD)
//					*ptrSeg = (*ptrGray <= (*ptrThrParent)) ? 255 : 0;
//				else if (ptrFgdEst!=0 && ptrThrParent!=0) {
//					*ptrSeg = (*ptrGray <= (*ptrFgdEst)*(*ptrThrParent)) ? 255 : 0;
//				}
//			}
//			if (mode!=DK_SEGSU && ptrWeightImg!=0 && ptrSegFgd!=0 && (*ptrSegFgd)==0 && (*ptrSeg)!=0) {//weight/segFgd image exists, pixel in foreground image not segmented, but in current image
//
//				*ptrSeg = (*ptrGray) <= (1.0f - *ptrWeightImg)*(*ptrThr) ? 255 : 0;
//			}
//
//
//			if (ptrWeightImg!=0) ptrWeightImg++;
//			if (ptrSegFgd!=0) ptrSegFgd++;
//
//			if (ptrParent != 0 && sameSize)
//				ptrParent++;
//			else if (ptrParent != 0 && j % 2 == 1)
//				ptrParent++;
//
//		}
//	}
//
//	bitwise_and(segTmpScale, mask, segTmpScale);
//
//	return segTmpScale.clone();
//	
//}
//
//Mat DkSegmentation::computeThrImgScale(Mat thrImg, Mat parentImg, Mat binThrImg) {
//	// parent image
//	//				-> bei gleicher größe konturen von parentimg in aktuelles schwellwertbild zeichnen -> mittlerer Schwellwert
//	//				-> bei anderer oktave größe anpassen...
//	Mat tmpParent;
//	Mat thrScale;
//	Mat binThrImgScale;
//	Mat parentImgScale;
//	vector<vector<Point> > contours;		/**< vector with all blob contours  **/
//	vector<Vec4i> hierarchy;			/**< hierarchy vector of all blobs (see findContours)  **/
//	DkBox bb;					/**< bounding box **/
//	DkVector offset;
//	Mat maskBox;
//	Mat thrBox;
//
//	if (parentImg.size() != thrImg.size()) {
//		//resize(parentImg, parentImgScale, thrImg.size(),0,0,INTER_NEAREST);
//		//parentImgScale.convertTo(tmpParent, CV_8U, 255);
//
//		resize(thrImg, thrScale, parentImg.size(),0,0,INTER_NEAREST);
//		resize(binThrImg, binThrImgScale, parentImg.size(),0,0,INTER_NEAREST);
//	} else { 
//		//parentImgScale = parentImg.clone();
//		//parentImgScale.convertTo(tmpParent, CV_8U, 255);
//		thrScale = thrImg.clone();
//		binThrImgScale = binThrImg.clone();
//	}
//	parentImg.convertTo(tmpParent, CV_8U, 255);
//
//	//thrScale = thrImg.clone();
//	//binThrImgScale = binThrImg.clone();
//
//	//if (!tmpParent.empty()) DkIP::imwrite("parentImg819.png", tmpParent);
//	findContours(tmpParent, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//
//	//if (!thrScale.empty()) DkIP::imwrite("thrImg766.png", thrScale);
//	//if (!binThrImgScale.empty()) DkIP::imwrite("binthrImg834.png", binThrImgScale);
//	//if (!tmpParent.empty()) DkIP::imwrite("parentImg825.png", tmpParent);
//	//if (!binThrImgScale.empty()) DkIP::imwrite("binthrImg819.png", binThrImgScale);
//
//	Scalar thr;
//	//DkUtils::getMatInfo(tmpParent, "tmpparent");
//	//DkUtils::getMatInfo(thrScale, "thrScale");
//	//DkUtils::printDebug(DK_MODULE, "contours size: %i\n", (int)contours.size());
//
//	int idx = 0;
//	//int tmp = 0;
//	for( ; idx >= 0 && !hierarchy.empty(); idx = hierarchy[idx][0]) {
//		
//		//tmp = idx;
//
//		if (contours.empty() || contours[idx].empty()) {
//			continue;
//		}
//
//		bb = DkBox(boundingRect(Mat(contours[idx])));
//		offset = bb.uc * -1.0f;
//
//		if (modeS==DK_SEGSU) {
//			maskBox.create(cvRound(bb.size.height), cvRound(bb.size.width), CV_32FC1);
//			maskBox.setTo(0);
//			//old version
//			//drawContours(maskBox, contours, idx, Scalar(1.0f,1.0f,1.0f), 1, 8, hierarchy, 1, offset.getCvPoint());
//			//draw only outer (parent contour and not the nested contours for determining the threshold
//			//TODO: testing
//			drawContours(maskBox, contours, idx, Scalar(1.0f,1.0f,1.0f), 1, 8, hierarchy, 0, offset.getCvPoint());
//			thrBox = thrScale(bb.getCvRect()).clone();
//			thrBox = thrBox.mul(maskBox);
//			maskBox.convertTo(maskBox, CV_8U, 255);
//			Scalar mean, std;
//			meanStdDev(thrBox, mean, std, maskBox);
//			thr = mean+std;
//
//			drawContours(thrScale, contours, idx, thr, CV_FILLED, 8, hierarchy, 0);//, offset.getCvPoint());
//
//		} else {
//			Mat highContrastPx = binThrImgScale(bb.getCvRect()).clone();
//			highContrastPx.convertTo(highContrastPx, CV_32F, 1.0f/255.0f);
//			int nrhighContrastPx = countNonZero(highContrastPx);
//
//			maskBox = highContrastPx;
//			////old version
//			////drawContours(maskBox, contours, idx, Scalar(1.0f,1.0f,1.0f), 1, 8, hierarchy, 1, offset.getCvPoint());
//			////draw only outer (parent contour and not the nested contours for determining the threshold
//			////TODO: testing
//			//drawContours(maskBox, contours, idx, Scalar(1.0f,1.0f,1.0f), 1, 8, hierarchy, 0, offset.getCvPoint());
//
//			thrBox = thrScale(bb.getCvRect()).clone();
//			thrBox = thrBox.mul(maskBox);
//			Mat tmpThrThr;
//			threshold(thrBox, tmpThrThr, 0, 1.0f, CV_THRESH_BINARY);
//			maskBox = maskBox.mul(tmpThrThr);
//			maskBox.convertTo(maskBox, CV_8U, 255);
//			Scalar mean, std;
//			meanStdDev(thrBox, mean, std, maskBox);
//			if (nrhighContrastPx != 0)
//				thr = mean;//mean+std;
//
//			//thr[0] = mean[0]-std[0]*0.25f;
//			//thr = mean(thrBox, maskBox);
//			//old version with nested contours...
//			//drawContours(thrScale, contours, idx, thr, CV_FILLED, 8, hierarchy, 1);//, offset.getCvPoint());
//			drawContours(thrScale, contours, idx, thr, CV_FILLED, 8, hierarchy, 0);//, offset.getCvPoint());
//		}
//	}
//
//	//DkUtils::printDebug(DK_MODULE, "idx: %i\n", tmp);
//	
//
//	//if (!thrScale.empty()) DkIP::imwrite("thrImg862.png", thrScale);
//	if (parentImg.size() != thrImg.size()) {
//		Mat tmp;
//		resize(thrScale, tmp, thrImg.size(),0,0,INTER_NEAREST);
//		thrScale = tmp;
//	}
//
//
//	return thrScale;
//}
//
//
//
//Mat DkSegmentation::computeContrastImg() {
//
//	Mat tmp = Mat(grayImg.size(), CV_32FC1);
//
//	Mat maxImg = DkIP::dilateImage(grayImg, 3, DK_SQUARE);
//	Mat minImg = DkIP::erodeImage(grayImg, 3, DK_SQUARE);
//
//	//speed up version of opencv style
//	for(int i = 0; i < maxImg.rows; i++)
//	{
//		float *ptrCon = tmp.ptr<float>(i);
//		unsigned char *ptrMin = minImg.ptr<unsigned char>(i);
//		unsigned char *ptrMax = maxImg.ptr<unsigned char>(i);
//		unsigned char *ptrMask = erodedMask.ptr<unsigned char>(i);
//
//		for(int j = 0; j < maxImg.cols; j++, ptrCon++, ptrMin++, ptrMax++, ptrMask++) {
//			*ptrCon = (*ptrMask > 0) ? 2.0f*(float)(*ptrMax - *ptrMin)/((float)(*ptrMax) + (float)(*ptrMin) + 255.0f + FLT_MIN) : 0.0f;
//			//-----------------original chew lim tan ----------------------------------------------------------------------
//			//*ptrCon = (*ptrMask > 0) ? (float)(*ptrMax - *ptrMin)/((float)(*ptrMax) + (float)(*ptrMin) + FLT_MIN) : 0.0f;
//			//-----------------original chew lim tan ----------------------------------------------------------------------
//		}
//	}
//	minImg.release(); // early release
//	maxImg.release(); // early release;
//
//	return tmp;
//}
//
//Mat DkSegmentation::computeBinaryContrastImg(Mat contrast) {
//
//	Mat histogram = DkIP::computeHist(contrast, erodedMask);
//	//double l = DkIP::getThreshOtsu(histogram, 0.0007) / 255.0;
//	double l = DkIP::getThreshOtsu(histogram) / 255.0;
//
//	Mat contrastImgThr;
//	threshold(contrast, contrastImgThr, l, 1.0f, CV_THRESH_BINARY);
//	contrastImgThr.convertTo(contrastImgThr, CV_8U, 255, 0);
//
//	if (meanContrast[0] == -1.0f) {
//		int n = countNonZero(contrastImgThr);
//		normalize(contrast, contrast, 1, 0, NORM_MINMAX, -1, contrastImgThr);
//		if (n > 2000)
//			meanContrast[0] = DkIP::statMomentMat(contrastImg, contrastImgThr, 0.5f, 5000);
//		else
//			meanContrast[0] = 0.0f;
//	}
//
//	return contrastImgThr;
//}
//
//
//void DkSegmentation::computeFgdEst(Mat grayImg, Mat mask) {
//
//	normalize(grayImg, fgdEstImg, 1, 0, NORM_MINMAX, -1, mask);
//	fgdEstImg = DkIP::erodeImage(fgdEstImg, 7, DK_SQUARE);
//	fgdEstImg.convertTo(fgdEstImg, CV_32F);
//	
//	normalize(fgdEstImg, fgdEstImg, 1, 0, NORM_MINMAX, -1, mask);
//	
//	DkIP::invertImg(fgdEstImg);
//	DkIP::imwrite("fgdEst813.png", fgdEstImg);
//	
//}
//
//Mat DkSegmentation::computeMeanFgdEst(Mat grayImg32F) {
//
//	Mat tmp;
//
//	if (fgdEstFilterSize < 3) {
//		tmp = Mat(grayImg32F.size(), CV_32FC1);
//		tmp.setTo(1.0f);
//	} else {
//		Mat fgdEstImgInt = Mat(grayImg32F.rows+1, grayImg32F.cols+1, CV_64FC1);
//		integral(grayImg32F, fgdEstImgInt);
//		tmp = DkIP::convolveIntegralImage(fgdEstImgInt, fgdEstFilterSize, 0, DK_BORDER_ZERO);
//		fgdEstImgInt.release(); // early release
//
//		//DkIP::mulMask(fgdEstImg, mask);	// diem: otherwise values outside the mask are mutual
//		normalize(tmp, tmp, 1.0f, 0, NORM_MINMAX, -1, mask);  // note: values outside the mask remain outside [0 1]
//		DkIP::invertImg(tmp);
//	}
//
//	//DkIP::imwrite("fgdEst828.png", tmp);
//	return tmp;
//}
//
//void DkSegmentation::computeThrImg(Mat grayImg32F, Mat binContrast, Mat &thresholdImg, Mat &thresholdContrastPxImg) {
//	int filtersize, Nmin;
//
//	if (modeS==DK_IPK || modeS==DK_SEGSU) {
//		if (strokeW >= 4.5) strokeW = 3.0;
//		filtersize = cvRound(strokeW*10);		//*10       //40
//		if ((filtersize % 2) != 1) filtersize+=1;
//		Nmin = cvFloor(strokeW*10);
//	} else {
//		//-----------------original chew lim tan ------------------------------------------
//		filtersize = cvRound(strokeW);
//		filtersize = (filtersize % 2) != 1 ? filtersize+1 : filtersize;
//		Nmin = filtersize;
//		//-----------------original chew lim tan ------------------------------------------
//	}
//
//	DkUtils::printDebug(DK_DEBUG_C, "kernelsize:  %i  Nmin:  %i\n", filtersize, Nmin);
//
//	Mat contrastBin32F;
//	binContrast.convertTo(contrastBin32F, CV_32FC1, 1.0f/255.0f);
//	// compute the mean image
//	Mat meanImg = grayImg32F.mul(contrastBin32F);	// do not overwrite the gray image
//	Mat stdImg = meanImg.mul(meanImg);
//
//	Mat intImg;
//	integral(meanImg, intImg);
//	meanImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DK_BORDER_ZERO);
//
//	DkUtils::printDebug(DK_DEBUG_B, "intGray: %s\n", DkUtils::getMatInfo(meanImg).c_str());
//	DkUtils::printDebug(DK_DEBUG_B, "grayImgInt: %s\n", DkUtils::getMatInfo(intImg).c_str());
//
//	// compute the standard deviation image
//	integral(stdImg, intImg);
//	stdImg = DkIP::convolveIntegralImage(intImg, filtersize, 0, DK_BORDER_ZERO);
//	intImg.release(); // early release
//
//	Mat intContrastBinary;
//	integral(contrastBin32F, intContrastBinary);
//	contrastBin32F.release();
//	intContrastBinary = DkIP::convolveIntegralImage(intContrastBinary, filtersize, 0, DK_BORDER_ZERO);
//	
//	meanImg /= intContrastBinary;
//	DkUtils::printDebug(DK_DEBUG_B, "mean: %s\n", DkUtils::getMatInfo(meanImg).c_str());
//
//	float *mPtr, *cPtr;
//	float *stdPtr;
//
//	for (int rIdx = 0; rIdx < stdImg.rows; rIdx++) {
//
//		mPtr = meanImg.ptr<float>(rIdx);
//		stdPtr = stdImg.ptr<float>(rIdx);
//		cPtr = intContrastBinary.ptr<float>(rIdx);
//
//		for (int cIdx = 0; cIdx < stdImg.cols; cIdx++, mPtr++, stdPtr++, cPtr++) {
//
//			*stdPtr = (*cPtr != 0) ? *stdPtr/(*cPtr) - (*mPtr * *mPtr) : 0.0f;	// same as OpenCV 0 division
//			if (*stdPtr < 0.0f) *stdPtr = 0.0f;		// sqrt throws floating point exception if stdPtr < 0
//
//		}
//	}
//	
//	sqrt(stdImg, stdImg);	// produces a floating point exception if < 0...
//
// 	Mat thrImgTmp = Mat(grayImg32F.size(), CV_32FC1);
//	Mat segImgTmp = Mat(grayImg32F.size(), CV_8UC1);
//
//	float *ptrStd, *ptrThr, *ptrSumContrast, *ptrMean;
//	unsigned char *ptrSeg;
//
//	//DkIP::getPixelInfo(intContrastBinary, 205,34, "sumContrast");
//
//	for(int rIdx = 0; rIdx < stdImg.rows; rIdx++) {
//		ptrThr = thrImgTmp.ptr<float>(rIdx);
// 		ptrMean = meanImg.ptr<float>(rIdx);
// 		ptrStd = stdImg.ptr<float>(rIdx);
//		ptrSeg = segImgTmp.ptr<unsigned char>(rIdx);
//		ptrSumContrast = intContrastBinary.ptr<float>(rIdx);
// 		
//		for(int cIdx = 0; cIdx < stdImg.cols; cIdx++, ptrThr++, ptrMean++, ptrStd++, ptrSeg++, ptrSumContrast++) {
//			//uncomment for dibco
//			//if (modeS==DK_IPK)
//				*ptrThr = *ptrMean + *ptrStd / 2;
//			//else {
//			//	*ptrThr = *ptrMean;
//			//}
//			*ptrSeg = *ptrSumContrast > Nmin ? 255 : 0;
// 		}
// 	}
//
//	thresholdContrastPxImg = segImgTmp;
//	thresholdImg = thrImgTmp;
//
//}
//
//
//float DkSegmentation::getStrokeWidth(Mat contrastImg) {
//#if 1
//	int height = contrastImg.rows;
//	int dy = 1;
////	int cnt=0;
//	float strokeWidth = 0;
//	std::list<int> diffs;
//	std::list<float> locInt;
//	diffs.clear();
//	Mat vec;
//
//	if (height > 100) dy = height / 4;
//	dy = 1;
//
//	for(int i=0; i < height; i+= dy) {
//		Mat row = contrastImg.row(i);
//		computeDistHist(row, &diffs, &locInt, 0.0f);
//	}
//
//	vec.create(1, 40, CV_32FC1);
//	vec = 0.0f;
//	std::list<int>::iterator iter = diffs.begin();
//	//std::list<float>::iterator iter2 = locInt.begin();
//	float *ptr = vec.ptr<float>(0);
//	int idx;
//	while (iter != diffs.end()) {
//		//idx = cvFloor(*iter/0.5f);
//		idx = (*iter);
//		//printf("distance: %i\n", *iter);
//		if (idx < vec.cols)
//			ptr[idx]++;//=(*iter2);
//
//		iter++;
//		//iter2++;
//	}
//	diffs.clear();
//
//	//DkUtils::printMat(vec, "strokeWidth");	// diem: nervt : )
//
//	//Mat sHist = DkIP::convolveSymmetric(vec, DkIP::get1DGauss(3.0f));
//	Mat sHist = vec;
//	//float *sHistPtr = sHist.ptr<float>(0);
//
//	double sMin, sMax;
//	Point pMinIdx, pMaxIdx;
//	int minIdx, maxIdx;
//	minMaxLoc(sHist, &sMin, &sMax, &pMinIdx, &pMaxIdx);
//	minIdx = pMinIdx.x;
//	maxIdx = pMaxIdx.x;
//
//	strokeWidth = 1.0f + (float)maxIdx;  //offset since idx starts with 0
//
//	if (strokeWidth < 3)
//		return 3.0f;
//	else
//		return strokeWidth;
//#else
//	int height = contrastImg.rows;
//	int dy = 1;
//	float strokeWidth = 0;
//	CDistHistList dhl(contrastImg.rows * contrastImg.cols / 2);
//	CDistHistList ldhl(contrastImg.cols / 2);
//	Mat vec;
//
//	if (height > 100) dy = height / 4;
//	dy = 1;
//	for(int i=0; i < height; i+= dy) {
//		Mat row = contrastImg.row(i);
//		ldhl.Reset();
//		computeDistHist(row, &dhl, &ldhl, 0.0f);
//	}
//
//	vec.create(1, 40, CV_32FC1);
//	vec = 0.0f;
//	float *ptr = vec.ptr<float>(0);
//	int idx;
//
//	for (int i = 0; i < dhl.getElemCount(); i++)
//	{
//		idx = dhl[i];
//		if (idx < vec.cols)
//			ptr[idx]++;//=(*iter2);
//	}
//
//	Mat sHist = vec;
//
//	double sMin, sMax;
//	Point pMinIdx, pMaxIdx;
//	int minIdx, maxIdx;
//	minMaxLoc(sHist, &sMin, &sMax, &pMinIdx, &pMaxIdx);
//	minIdx = pMinIdx.x;
//	maxIdx = pMaxIdx.x;
//
//	strokeWidth = 1.0f + (float)maxIdx;  //offset since idx starts with 0
//
//	if (strokeWidth < 3)
//		return 3.0f;
//	else
//		return strokeWidth;
//#endif
//}
//
//void DkSegmentation::computeDistHist(Mat src, std::list<int> *maxDiffList, std::list<float> *localIntensity, float gSigma) {
//
//	std::list<int> localMaxList;
//	std::list<int>::iterator localMaxIter;
//
//	Mat sHist;
//	if (gSigma > 0)
//		sHist = DkIP::convolveSymmetric(src, DkIP::get1DGauss(gSigma));
//	else
//		sHist = src;
//
//	localMaxList.clear();
//
//	float *sHistPtr = sHist.ptr<float>();
//
//	for (int prevIdx = 0, currIdx = 1, nextIdx=2; prevIdx < sHist.cols; prevIdx++, nextIdx++, currIdx++) {
//
//		currIdx %= sHist.cols;
//		nextIdx %= sHist.cols;
//
//		if ((sHistPtr[prevIdx] <= sHistPtr[currIdx]) && (sHistPtr[currIdx] > sHistPtr[nextIdx])) {
//			localMaxList.push_back(currIdx);
//			localIntensity->push_back(sHistPtr[currIdx]);
//			//localIntensity->push_back(255.0f);
//		}
//	}
//	if (localIntensity->size() >= 1)
//		localIntensity->pop_back();
//
//	if (localMaxList.empty() || localMaxList.size() <= 1)	
//	return;	// at least two local maxima present?
//
//	localMaxIter = localMaxList.begin();
//	int cIdx = *localMaxIter;
//	++localMaxIter;	// skip the first element
//
//	// compute the distance between peaks
//	int lIdx = -1;
//	int idx = 1;
//	while (localMaxIter != localMaxList.end()) {
//
//		lIdx = cIdx;
//		cIdx = *localMaxIter;
//
//		//if (idx%2 == 0)
//			maxDiffList->push_back(abs(cIdx-lIdx));
//		idx++;
//		//printf("distance: %i\n", cIdx-lIdx);
//		++localMaxIter;
//	}
//}
//
//void DkSegmentation::computeDistHist(Mat src, CDistHistList *pDHL, CDistHistList *pLocalDHL, float gSigma)
//{
//	Mat sHist;
//	if (gSigma > 0)
//		sHist = DkIP::convolveSymmetric(src, DkIP::get1DGauss(gSigma));
//	else
//		sHist = src;
//
//	float *sHistPtr = sHist.ptr<float>();
//
//	for (int prevIdx = 0, currIdx = 1, nextIdx=2; prevIdx < sHist.cols; prevIdx++, nextIdx++, currIdx++) {
//
//		currIdx %= sHist.cols;
//		nextIdx %= sHist.cols;
//
//		if ((sHistPtr[prevIdx] <= sHistPtr[currIdx]) && (sHistPtr[currIdx] > sHistPtr[nextIdx])) {
//			pLocalDHL->add(currIdx);
//		}
//	}
//
//	if (pLocalDHL->getElemCount() <= 1)	
//		return;	// at least two local maxima present?
//
//	int localMaxIter = 0;
//	int cIdx = (*pLocalDHL)[0];
//	++localMaxIter;	// skip the first element
//
//
//	// compute the distance between peaks
//	int lIdx = -1;
//	while (localMaxIter < pLocalDHL->getElemCount()) {
//
//		lIdx = cIdx;
//		cIdx = (*pLocalDHL)[localMaxIter];
//
//		pDHL->add(abs(cIdx-lIdx));
//
//		++localMaxIter;
//	}
//
//}
//
//void DkSegmentation::saveDebugImg(Mat img, std::string path, std::string filename) {
//	
//	std::string saveName, saveFull;
//
//	getConfidence();
//
//	saveName = "_" + getName() + "_DKsegImgNew";
//	//saveName = "_segDkSatFilt";
//	saveFull = path + DkUtils::createFileName(filename, saveName);
//
//	int rows = segImg.rows;
//	int cols = segImg.cols;
//	Mat grayimg;
//	cvtColor(img, grayimg, CV_RGB2GRAY);
//
//	Mat result = Mat(rows, cols*3, CV_32FC1);
//	Mat a = result(Range::all(), Range(0, cols));
//	grayimg.convertTo(a, CV_32F);
//	Mat b = result(Range::all(), Range(cols, cols+cols));
//	//segImg*=255;
//	segImg.convertTo(b, CV_32F);
//	Mat c = result(Range::all(), Range(cols+cols, cols+cols+cols));
//	//segSatImg*=255;
//	segSatImg.convertTo(c, CV_32F);
//
//	std::string txt = "Confidence: " + DkUtils::stringify(confidence*100);
//	int baseline;
//	float xW = (float)cols;
//	float yH = 0.25f*(float)rows;
//	int scale = 0;
//	Size s;
//	
//	do {
//		scale++;
//		s = getTextSize(txt, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, 5, &baseline);
//	} while((s.width < xW) && (s.height < yH));
//	if (scale > 1) scale--;
//	s = getTextSize(txt, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, 5, &baseline);
//
//	Point tC = Point(0,s.height);
//	putText(result, txt, tC, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, Scalar(255, 255, 255), 5, CV_AA);
//	putText(result, txt, tC, CV_FONT_HERSHEY_COMPLEX_SMALL, scale, Scalar(0, 0, 0), 1, CV_AA);
//
//	if (!segImg.empty()) {
//		//printf("..saving image: %s\n", saveSeg.c_str());
//		//DkIP::imwrite(saveFull.c_str(), img);
//		DkIP::imwrite(saveFull.c_str(), result);
//		std::string msg = "[" + getName()  + "] debug images saved...";
//		DkUtils::printDebug(DK_DEBUG_A, msg.c_str());
//		//printf("done...\n", saveSeg.c_str());
//
//	} else
//		throw DkMatException("empty mat", __LINE__, __FILE__);
//
//}
//
//Mat DkSegmentation::getThrImg() {
//
//	if (releaseDebug == DK_RELEASE_IMGS && thrImg.empty()) {
//		std::string msg = "[" + getName() + "]" + "debug images are released\n";
//		DkUtils::printDebug(DK_WARNING, msg.c_str());
//	}
//	return thrImg;
//}
//
//Mat DkSegmentation::getContrastImg() {
//	
//	if (releaseDebug == DK_RELEASE_IMGS && contrastImg.empty()) {
//		std::string msg = "[" + getName() + "]" + "debug images are released\n";
//		DkUtils::printDebug(DK_WARNING, msg.c_str());
//	}
//	return contrastImg;
//}
//
//Mat DkSegmentation::getFgdEstImg() {
//
//	if (releaseDebug == DK_RELEASE_IMGS && fgdEstImg.empty()) {
//		std::string msg = "[" + getName() + "]" + "debug images are released\n";
//		DkUtils::printDebug(DK_WARNING, msg.c_str());
//	}
//	return fgdEstImg;
//}
//
//Mat DkSegmentation::getErodedMask() {
//
//	if (releaseDebug == DK_RELEASE_IMGS && erodedMask.empty()) {
//		std::string msg = "[" + getName() + "]" + "debug images are released\n";
//		DkUtils::printDebug(DK_WARNING, msg.c_str());
//	}
//	return erodedMask;
//}
//
//float DkSegmentation::getConfidence() {
//
//	float fullPaper = 0.18f;
//
//	//confidence already calculated
//	if (confidence >= 0.0f)
//		return confidence;
//
//	//otherwise calculate confidence
//	if (segSatImg.empty()) {
//		//classical Segmentation needed for the removal of border pixel
//		checkInput();
//		convertInputData();
//
//		computeLocalMinMaxSatSeg(false);
//	}
//
//	segmentedPx = countNonZero(segSatImg);
//	maskPx = countNonZero(mask);
//
//	if (maskPx == 0) {
//		confidence = 0.0f;
//
//		if (mask.empty()) {
//			DkUtils::printDebug(DK_WARNING, "[DkSegmentation] mask image is empty, where it should not be (getConfidence)\n");
//		}
//
//	} else {
//		float segRatio = (float)segmentedPx / (float)maskPx;
//
//		segRatio = segRatio-fullPaper < 0 ? 1.0f : 1.0f - (segRatio-fullPaper)/(1.0f-fullPaper);
//
//		if (segRatio != 1.0f) {
//			printf("---------------------------\n");
//			printf("segRatio != 1.0f\n");
//			printf("---------------------------\n");
//		}
//		confidence = segRatio * (float)meanContrast[0];
//	}
//
//	//printf("confidence:  %f\n", confidence);
//
//	return confidence;
//}
//
//
//std::string DkSegmentation::toString() {
//
//	std::string msg = "Segmentation --------------------------\n";
//	msg += "parameters:\n";
//	msg += "    Filter Size for the foreground estimation: " + DkUtils::stringify(fgdEstFilterSize) + "         sigmoid Slope: " + DkUtils::stringify(sigmSlope) + "\n";
//
//	return msg;
//}
//
//void DkSegmentation::release() {
//
//	rgbImg.release();
//	mask.release();
//	contrastImg.release();
//	binContrastImg.release();
//	fgdEstImg.release();
//	thrImg.release();
//	erodedMask.release();
//	grayImg.release();
//
//	imgs = 0;
//}
//
//void DkSegmentation::clearDebugImg() {
//
//	contrastImg.release();
//	binContrastImg.release();
//	fgdEstImg.release();
//	thrImg.release();
//	erodedMask.release();
//	grayImg.release();
//	//// input images...			//FK 15112010 bug, otherwise mask image is deleted before confidence is calculated
//	//rgbImg.release();
//	//mask.release();
//}

