/**************************************************
 * 	DkObjectRecognition.h
 *
 *	Created on:	12.08.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once

#include "DkModuleInclude.h"
#include "DkUtils.h"
#include "DkSlidingWindow.h"

#include "opencv2/objdetect/objdetect.hpp"

class DK_MODULE_API DkInputParameter {

public:
	DkInputParameter();

	// detector
	int detectorMinArea;	/**<Minimum Area in px of an object detected.>*/
	int detectorMaxArea;	/**<Maximum Area in px of an object detected.>*/
	int detectorMaxSide;	/**<Maximum length of a polygon's side in px.>*/
	bool filterDuplicates;	/**<If true, duplicates are removed.>*/
	bool detectCircles;		/**<Detect Circles (only if the mode is DETECT_RECTS) >*/

	// feature extraction
	float rescaleArea;		/**<Rescales the Area such that more features are part of it.>*/

	// classification
	std::vector<std::string> classStrings;	/**<Class strings that are used for classification>*/
};

class DK_MODULE_API DkProductInfo {

public:
	DkProductInfo();
	DkProductInfo(const DkSlidingWindow& window);
	
	void getStorageBuffer(char** buffer, size_t& length) const;
	void setStorageBuffer(const char* buffer);

	bool isEmpty() const;
	int getLabel() const;
	float getProbability() const;
	DkBox getRectangle() const;
	void getRectangle(float& x, float& y, float& width, float& height) const;
	void getClassResponses(std::vector<float>& classes) const;
	void getPolygon(std::vector<float>& xCoords, std::vector<float>& yCoords) const;
	DkSlidingWindow& getSlidingWindow();
	const DkSlidingWindow& getSlidingWindowConst() const;

protected:
	DkSlidingWindow window;

};

class DK_MODULE_API DkObjectRecognition {

public:
	DkObjectRecognition();
	DkObjectRecognition(const std::string& dirPath, const std::string& bowName, const std::string& classifierName, const std::string& objectDetectionName, const std::string& lookupName);
	std::vector<DkProductInfo> recognize(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step);
	std::vector<DkProductInfo> detect(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, int mode = DK_DETECT_VIOLA, const DkInputParameter& params = DkInputParameter());
	std::vector<DkProductInfo> classify(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, const std::vector<DkProductInfo>& objectsDetected, const DkInputParameter& params = DkInputParameter()) const;
	void reclassify(std::vector<DkProductInfo>& objectsDetected, const DkInputParameter& params = DkInputParameter()) const;

	std::vector<DkProductInfo> track(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, const std::vector<DkProductInfo>& oldProductInfo) const;

	enum {
		DK_DETECT_VIOLA = 0,
		DK_DETECT_RECT,

		DK_DETECT_END
	};

protected:
	cv::CascadeClassifier box_cascade;
	bool splitWindows;

	std::vector<cv::Rect> detectViola(const Mat& img);
	std::vector<DkProductInfo> detectRect(const Mat& img, const DkInputParameter& params = DkInputParameter());
	std::vector<DkProductInfo> classify(const Mat& img, const std::vector<DkProductInfo>& objectsDetected, const DkInputParameter& params = DkInputParameter()) const;

	bool bufferToMat(const unsigned char* image, unsigned int width, unsigned int height, unsigned int step, Mat& convertedImage) const;
};

