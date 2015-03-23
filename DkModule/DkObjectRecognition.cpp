/**************************************************
 * 	DkObjectRecognition.cpp
 *
 *	Created on:	12.08.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkObjectRecognition.h"
#include "DkMachineLearning.h"
#include "DkFeatureExtraction.h"
#include "DkQuantization.h"
#include "DkSegmentation.h"
#include "opencv2/video/tracking.hpp"

// DkInputParameter --------------------------------------------------------------------
DkInputParameter::DkInputParameter() :	detectorMinArea(7000), detectorMaxArea(200000), 
										detectorMaxSide(0), filterDuplicates(true),
										rescaleArea(1.0f), detectCircles(true) {
}

// ProductInfo --------------------------------------------------------------------
/**
 * Default empty constructor.
 **/ 
DkProductInfo::DkProductInfo() {}

/**
 * Default constructor.
 * @param window a computed sliding window.
 **/ 
DkProductInfo::DkProductInfo(const DkSlidingWindow& window) {
	this->window = window;
}


/**
 * Serializes transportable member vars to a byte array.
 * NOTE: you have to take care of this buffer (e.g. delete it)
 * @param buffer pointer to a char buffer which will be filled with the values
 * @param length (output) the size of the buffer in bytes
 **/ 
void DkProductInfo::getStorageBuffer(char** buffer, size_t& length) const {

	window.getStorageBuffer(buffer, length);

}

/**
 * Initializes an instance with a serialized buffer (@see getStorageBuffer).
 * NOTE: you must not change any bit of the buffer.
 * @param buffer the buffer with all important member vars.
 **/ 
void DkProductInfo::setStorageBuffer(const char* buffer) {

	window.setStorageBuffer(buffer);
}

/**
 * Returns true if the object was created with the empty constructor
 * or if no descriptors were assigned to the sliding window.
 * @return bool true if the object is empty.
 **/ 
bool DkProductInfo::isEmpty() const {
	return window.isEmpty();
}

/**
 * Returns the object's label.
 * The labels can be mapped using DkLabelManager
 * @return int the label index.
 **/ 
int DkProductInfo::getLabel() const {
	return window.getLabel();
}

/**
 * Returns the object's bounding box.
 * The coordinates are stored in pixel.
 * The coordinate origin is in the upper left corner of an image.
 * @return DkBox the bounding box
 **/ 
DkBox DkProductInfo::getRectangle() const {
	return window.getWindow();
}

/**
 * Returns the object's bounding box.
 * The coordinates are stored in pixel.
 * The coordinate origin is in the upper left corner of an image.
 * @param x the x coordinate of the upper left point (in pixel)
 * @param y the y coordinate of the upper left point (in pixel)
 * @param width the BB's width (in pixel)
 * @param height the BB's height (in pixel)
 **/ 
void DkProductInfo::getRectangle(float& x, float& y, float& width, float& height) const {
	
	DkBox b = window.getWindow();
	x = b.uc.x;
	y = b.uc.y;
	width = b.getWidthF();
	height = b.getHeightF();
}

/**
 * The 'probability' of the class decision between [0 1].
 * @return float the objects classification probability.
 **/ 
float DkProductInfo::getProbability() const {
	return window.getProbability();
}

/**
 * Returns the exact polygon of a product.
 * The polygon points are sorted. Hence, xCoords[i] xCoords[i+/-1] should
 * be connected.
 * NOTE xCoords and yCoords are empty if the products
 * are localized using DETECT_VIOLA.
 * @param xCoords a 4 element vector with the polygon's x coordinates
 * @param yCoords a 4 element vector with the polygon's y coordinates
 **/ 
void DkProductInfo::getPolygon(std::vector<float>& xCoords, std::vector<float>& yCoords) const {

	std::vector<DkVector> pts = window.getPolyRect().getCorners();

	for (size_t idx = 0; idx < pts.size(); idx++) {

		xCoords.push_back(pts[idx].x);
		yCoords.push_back(pts[idx].y);
	}
}

/**
 * Returns the class histogram.
 * The i-th element stores the probability of the i-th class.
 * Hence, the histograms maximum corresponds with the value of the object's getProbability() member function.
 * @param classes the probabilities for all class indices.
 **/ 
void DkProductInfo::getClassResponses(std::vector<float>& classes) const {

	Mat prop = window.getClassResponse();
	const float* cProp = prop.ptr<float>();

	for (int idx = 0; idx < prop.cols; idx++) {
		classes.push_back(cProp[idx]);
	}
}

/**
 * Returns an editable instance of the object's sliding window.
 * @return DkSlidingWindow& the object's sliding window.
 **/ 
DkSlidingWindow& DkProductInfo::getSlidingWindow() {
	return window;
}

/**
 * Returns the object's sliding window.
 * @return const DkSlidingWindow& the object's sliding window.
 **/ 
const DkSlidingWindow& DkProductInfo::getSlidingWindowConst() const {
	return window;
}

// ObjectRecognition --------------------------------------------------------------------
/**
 * Empty default constructor.
 **/ 
DkObjectRecognition::DkObjectRecognition() {

	// params
	splitWindows = false;
}

/**
 * Object constructor.
 * All external files are loaded in this constructor.
 * A warning is written to the std::out if an external file could not be loaded.
 * @param dirPath the file path to the directory which contains all external files.
 * @param bowName the file name of the Bag-of-Words yml.
 * @param classifierName the file name of the Random Trees classifier yml.
 * @param objectDetectionName the file name of the cascade classifier xml which is needed for object detection.
 * @param lookupName the file name of the lookup file yml which maps label indices to label names.
 **/ 
DkObjectRecognition::DkObjectRecognition(
	const std::string& dirPath, 
	const std::string& bowName, 
	const std::string& classifierName, 
	const std::string& objectDetectionName, 
	const std::string& lookupName) {

	DkTimer dt;
	DkBoW::loadVocabulary(dirPath, bowName);
	DkLabelManager::loadLookup(dirPath, lookupName);
	DkClassifier::loadClassifier(dirPath, classifierName);
	
	if (box_cascade.load(dirPath + objectDetectionName)) {
		mout << dirPath + objectDetectionName << " loaded" << dkendl;
	}
	else {
		if (box_cascade.load(objectDetectionName)) {
			mout << objectDetectionName << " loaded" << dkendl;
		}
		else {
			wout << "sorry, " << objectDetectionName << " could not be loaded..." << dkendl;
		}
	}

	// load templates
	std::vector<DkSlidingWindow> wins;
	std::string tmpName = DkUtils::createFileName(classifierName, "-templates");
	if (DkTemplateMatcher::load(dirPath, tmpName, wins))
		mout << tmpName << " loaded" << dkendl;
	else
		wout << "sorry, " << dirPath + tmpName << " could not be loaded..." << dkendl;

	DkTemplateMatcher::templateWindows = wins;

	// params
	splitWindows = false;

	mout << "classifiers loaded in: " << dt << dkendl;
}

/**
 * Detects McDonalds products in images.
 * This function locates and classifies McDonalds products. The quality depends on the quality of the trained classifiers.
 * @param image the image buffer - an RGB image with one byte per channel is assumed.
 * @param width	the image width in pixel
 * @param height the image height in pixel
 * @param step the step size of the image. this is needed since some image formats have zero padding for faster pointer allocations.
 *	usually the step size is s = width * numChannels.
 * @return std::vector<DkProductInfo> a vector containing all classified products found in the image.
 **/ 
std::vector<DkProductInfo> DkObjectRecognition::recognize(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step) {

	std::vector<DkProductInfo> windows;	

	Mat img;
	if (!bufferToMat(image, width, height, step, img)) {
		wout << "[WANRING] Could not convert image..." << dkendl;
		return std::vector<DkProductInfo>();
	}

	std::vector<cv::Rect> boxes = detectViola(img);

	DkTimer dt;
	
	// nothing found?
	if (boxes.empty())
		return windows;
	
	std::vector<std::string> allClasses;
	//windows = classify(img, boxes, allClasses);	// TODO: convert boxes to product infos

	return windows;
}

/**
 * Performs an Object Detection.
 * @param image the image buffer - an RGB image with one byte per channel is assumed.
 * @param width	the image width in pixel
 * @param height the image height in pixel
 * @param step the step size of the image. this is needed since some image formats have zero padding for faster pointer allocations.
 *	usually the step size is s = width * numChannels.
 * @return std::vector<DkProductInfo> a vector containing all products found in the image.
 **/ 
std::vector<DkProductInfo> DkObjectRecognition::detect(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, int mode, const DkInputParameter& params) {

	Mat img;
	if (!bufferToMat(image, width, height, step, img)) {
		wout << "[WANRING] Could not convert image..." << dkendl;
		return std::vector<DkProductInfo>();
	}

	if (mode < 0 || mode >= DK_DETECT_END) {
		wout << "[DkObjectRecognition] illegal mode: " << mode << dkendl;
		mode = DK_DETECT_VIOLA;
	}

	std::vector<DkProductInfo> productInfo;

	if (mode == DK_DETECT_VIOLA) {
		std::vector<cv::Rect> boxes = detectViola(img);

		for (int idx = 0; idx < boxes.size(); idx++) {
			DkSlidingWindow s;
			s.setInputParams(params);
			s.setWindow(boxes.at(idx));
			productInfo.push_back(s);
		}
	}
	else if (mode == DK_DETECT_RECT) {
		productInfo = detectRect(img, params);
	}

	return productInfo;
}

/**
 * Convenience method.
 * @param img an CV_8UC3 image.
 * @return std::vector<cv::Rect> rectangles around objects detected.
 **/ 
std::vector<cv::Rect> DkObjectRecognition::detectViola(const Mat& img) {

	DkTimer dt;

	// get the grayscale image
	Mat imgG;
	cv::cvtColor(img, imgG, CV_RGB2GRAY);
	float scale = 1.0f;

	// speed-up object detection
	while (imgG.cols > 480) {
		scale *= 0.5;
		cv::resize(imgG, imgG, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
	}

	std::vector<int> rLevels;
	std::vector<double> levelWeights;

	std::vector<cv::Rect> boxes;
	box_cascade.detectMultiScale(imgG, boxes, rLevels, levelWeights, 1.2, 5, 0, cv::Size(15,15), cv::Size(800, 800));
	mout << boxes.size() << " objects detected in: " << dt << dkendl;

	// rescale boxes
	if (scale != 1.0) {
		for (int idx = 0; idx < boxes.size(); idx++) {

			cv::Rect& cBox = boxes.at(idx);
			cBox.x = cvRound(cBox.x*1/scale);
			cBox.y = cvRound(cBox.y*1/scale);
			cBox.width = cvRound(cBox.width*1/scale);
			cBox.height = cvRound(cBox.height*1/scale);
		}
	}

	return boxes;
}

std::vector<DkProductInfo> DkObjectRecognition::detectRect(const Mat& img, const DkInputParameter& params) {

	DkBurgerSegmentation segM(img);
	segM.setInputParams(params);
	segM.compute();

	if (params.filterDuplicates)
		segM.filterDuplicates();
	
	std::vector<DkPolyRect> rects = segM.getRects();
	std::vector<DkCircle> circles = segM.getCircles();
	std::vector<DkProductInfo> win;

	for (size_t idx = 0; idx < rects.size(); idx++) {
		
		DkPolyRect p = rects[idx];
		DkSlidingWindow w;
		w.setPolyRect(p);
		w.setInputParams(params);
		win.push_back(w);
	}
	
	for (size_t idx = 0; idx < circles.size(); idx++) {

		DkSlidingWindow w;
		w.setCircle(circles[idx]);
		w.setInputParams(params);
		win.push_back(w);
	}

	return win;
}

/**
 * Classifies the products stored in objectsDetected.
 * @param image the image buffer - an RGB image with one byte per channel is assumed.
 * @param width	the image width in pixel
 * @param height the image height in pixel
 * @param step the step size of the image. this is needed since some image formats have zero padding for faster pointer allocations.
 *	usually the step size is s = width * numChannels.
 * @param objectsDetected a vector containing rectangles of all objects detected so far.
 * @param params input parameter, including the active class list.
 * @return std::vector<DkProductInfo> the updated productInfos with classResponses, label etc.
 **/ 
std::vector<DkProductInfo> DkObjectRecognition::classify(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, 
														 const std::vector<DkProductInfo>& objectsDetected, const DkInputParameter& params) const {

	Mat img;
	if (!bufferToMat(image, width, height, step, img)) {
		wout << "[WANRING] Could not convert image..." << dkendl;
		return std::vector<DkProductInfo>();
	}

	return classify(img, objectsDetected, params);
}

void DkObjectRecognition::reclassify(std::vector<DkProductInfo>& objectsDetected, const DkInputParameter& params) const {

	DkLabelManager lb;
	lb.setActiveClassList(params.classStrings);

	DkMultiClassifier classifier;
	
	for (size_t idx = 0; idx < objectsDetected.size(); idx++) {

		DkSlidingWindow& win = objectsDetected[idx].getSlidingWindow();
		win.setClassResponse(classifier.predict(win.getBoWDescriptor()));
	}
}

/**
 * Convenience function for me.
 * @param img a CV_8UC3 image
 * @param image the image buffer - an RGB image with one byte per channel is assumed.
 * @param width	the image width in pixel
 * @param height the image height in pixel
 * @param step the step size of the image. this is needed since some image formats have zero padding for faster pointer allocations.
 *	usually the step size is s = width * numChannels.
 **/ 
std::vector<DkProductInfo> DkObjectRecognition::classify(const Mat& img, const std::vector<DkProductInfo>& objectsDetected, 
														 const DkInputParameter& params) const {
	
	std::vector<DkProductInfo> windows;
	std::vector<cv::Rect> extBoxes;

	bool computeFeaturesPerImage = false;

	DkTimer dt;

	DkLabelManager lb;
	lb.setActiveClassList(params.classStrings);

	if (splitWindows) {

		for (int idx = 0; idx < objectsDetected.size(); idx++) {

			extBoxes.push_back(objectsDetected.at(idx).getRectangle().getCvRect());
			DkBox quarters = objectsDetected.at(idx).getRectangle();
			quarters.lc = quarters.center();
			DkBox shiftedQuarters = quarters;
			extBoxes.push_back(shiftedQuarters.getCvRect());
			shiftedQuarters.moveBy(DkVector(0, shiftedQuarters.size().y));
			extBoxes.push_back(shiftedQuarters.getCvRect());
			shiftedQuarters = quarters;
			shiftedQuarters.moveBy(DkVector(shiftedQuarters.size().x, 0));
			extBoxes.push_back(shiftedQuarters.getCvRect());
			shiftedQuarters = quarters;
			shiftedQuarters.moveBy(shiftedQuarters.size());
			extBoxes.push_back(shiftedQuarters.getCvRect());
		}
	}

	// compute features
	DkTimer dtf;
	std::vector<cv::KeyPoint> keyPoints;
	Mat descriptors;

	if (computeFeaturesPerImage) {

		for (size_t idx = 0; idx < objectsDetected.size(); idx++) {

			cv::Rect rect = objectsDetected.at(idx).getRectangle().getCvRect();
			Mat imgRoi = img(rect);

			if (imgRoi.empty())
				continue;

			DkFeatureExtraction fe;
			fe.setInputParams(params);
			fe.enableFast(false);
			fe.compute(imgRoi);

			// copy features
			const std::vector<cv::KeyPoint>& tmpKp = fe.getKeypoints();
			DkBox roi = rect;

			for (int idx = 0; idx < tmpKp.size(); idx++) {
				cv::KeyPoint kp = tmpKp.at(idx);
				kp.pt = Point2f(kp.pt.x + roi.uc.x, kp.pt.y + roi.uc.y);	// shift kps back to image coordinates
				keyPoints.push_back(kp);
			}
			descriptors.push_back(fe.getDescriptors());
		}
	}
	else {
		DkFeatureExtraction fe;
		fe.setInputParams(params);

		if (extBoxes.empty())
			fe.compute(img, objectsDetected);
		else
			fe.compute(img, extBoxes);

		keyPoints = fe.getKeypoints();
		descriptors = fe.getDescriptors();

//		DkBoW bow;
//		std::string savePath = "D:\\McDonalds\\temp\\";
//		std::string fName = "bow.jpg";
//		bow.saveVocabulary(img, fe.getKeypoints(), fe.getDescriptors(), savePath, fName);

	}
	mout << "Feature computation takes: " << dtf << dkendl;

	if (keyPoints.empty()) {
		wout << "WARNING: no interest points detected, skipping..." << dkendl;
		return windows;
	}

	DkTimer dtb;
	DkBagger bagger(keyPoints, descriptors);
	
	if (extBoxes.empty())
		bagger.compute(objectsDetected);
	else
		bagger.compute(extBoxes);
	std::vector<DkSlidingWindow> slidingWindows = bagger.getWindows();
	mout << "bagging takes: " << dtb << dkendl;

	DkTimer dtc;
	DkMultiClassifier classifier;
	classifier.predict(slidingWindows);
	mout << "classification takes: " << dtc.getIvl() << dkendl;

	if (splitWindows) {

		// vote here
		std::vector<DkSlidingWindow> tmpWin;

		for (int idx = 0; idx < slidingWindows.size(); ) {
			
			DkSlidingWindow cWin = slidingWindows.at(idx);
			if (cWin.getBoWDescriptor().empty()) {
				idx += 5;
				continue;
			}
			
			Mat votingMat = cWin.getClassResponse(); idx++;
			float sum = 1;

			for (int i = 0; i < 4; i++, idx++) {
				
				if (slidingWindows.at(idx).getBoWDescriptor().empty())
					continue;

				votingMat += slidingWindows.at(idx).getClassResponse(); 
				sum++;
			}			

			votingMat /= sum;

			cWin.setClassResponse(votingMat);
			tmpWin.push_back(cWin);
		}

		slidingWindows = tmpWin;
	}

	for (int idx = 0; idx < slidingWindows.size(); idx++)
		windows.push_back(slidingWindows.at(idx));

	return windows;
}

bool DkObjectRecognition::bufferToMat(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, Mat& convertedImage) const {

	if (!width || !height) {
		wout << "Illegal size: " << width << " x " << height << dkendl;
		return false;
	}

	// convert image
	convertedImage = Mat(height, width, CV_8UC3, (void*)image, step);
	//convertedImge = convertedImage.clone();	// make a deep copy for safety

	return !convertedImage.empty();
}

/**
 * Tracks an object.
 * If the oldProductInfo does have an empty hist Mat, a color histogram will be computed
 * at the current rectangles location.
 * @param image the image buffer - an RGB image with one byte per channel is assumed.
 * @param width	the image width in pixel
 * @param height the image height in pixel
 * @param step the step size of the image. this is needed since some image formats have zero padding for faster pointer allocations.
 *	usually the step size is s = width * numChannels.
 * @param oldProductInfo the productInfo created by detect() or the last productInfo returned by track().
 * @return std::vector<DkProductInfo> the updated productInfo.
 **/ 
std::vector<DkProductInfo> DkObjectRecognition::track(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, const std::vector<DkProductInfo>& oldProductInfo) const {

	DkTimer dt;
	std::vector<DkProductInfo> productInfo;
	
	// convert image
	cv::Mat img(height, width, CV_8UC3, (void*)image, step);
	img = img.clone();	// make a deep copy for safety

	Mat hsvImg, hImg, vImg;
	cvtColor(img, hsvImg, CV_BGR2HSV);
	
	//int chH[] = {0, 0};
	//int chV[] = {2, 2};
	hImg.create(hsvImg.size(), hsvImg.depth());
	//vImg.create(hsvImg.size(), hsvImg.depth());
	//mixChannels(&hsvImg, 1, &hImg, 1, chH, 1);
	//mixChannels(&hsvImg, 1, &vImg, 1, chV, 1);
	std::vector<Mat> channels;
	cv::split(hsvImg, channels);

	hImg = channels[0];
	vImg = channels[2];

	// set angles to zero if value is low
	for (int rIdx = 0; rIdx < vImg.rows; rIdx++) {

		const unsigned char* vPtr = vImg.ptr<const unsigned char>(rIdx);
		unsigned char* hPtr = hImg.ptr<unsigned char>(rIdx);

		for (int cIdx = 0; cIdx < vImg.cols; cIdx++) {

			if (vPtr[cIdx] < 50)
				hPtr[cIdx] = 0;
		}
	}

	for (int idx = 0; idx < oldProductInfo.size(); idx++) {


		DkSlidingWindow cWin = oldProductInfo.at(idx).getSlidingWindowConst();
		DkBox cBox = cWin.getWindow();
		
		cWin.calcHist(hImg);
		cWin.track(hImg);

		productInfo.push_back(cWin);
	}

	mout << "[tracking] " << productInfo.size() << " takes: " << dt << dkendl;

	return productInfo;
}
