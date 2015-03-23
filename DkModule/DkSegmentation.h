/**************************************************
 * 	DkSegmentation.h
 *
 *	Created on:	15.01.2015
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once

#include "DkModule.h"
#include "DkSlidingWindow.h"

class DkInputParameter;

class DK_MODULE_API DkKMeansSegmentation : public DkModule {

public:
	DkKMeansSegmentation(const Mat& colImg = Mat());
	virtual void compute();
	virtual std::string toString() const;
	virtual Mat getSegImg();
	virtual Mat getDebugImg() const;
	virtual cv::Rect getBBox() const;
	int k;	// k of k-Means

protected:
	float resize;
	int numObjects;
	Mat img;
	Mat segImg;
	Mat dbImg;
	cv::Rect bbox;

	virtual void checkInput() const;
};

class DK_MODULE_API DkBurgerSegmentation : public DkModule {

public:
	DkBurgerSegmentation(const Mat& colImg = Mat());

	virtual void compute();
	virtual std::string toString() const;
	virtual void filterDuplicates(float overlap = 0.6f, float areaRatio = 0.1f);
	virtual void filterDuplicates(std::vector<DkPolyRect>& rects, float overlap = 0.6f, float areaRatio = 0.1f) const;
	virtual void filterDuplicatedCircles(std::vector<DkCircle>& circles, const std::vector<DkPolyRect>& rects, float overlap = 0.6f) const;

	virtual std::vector<DkPolyRect> getRects() const { return rects; };
	virtual std::vector<DkCircle> getCircles() const { return circles; };
	virtual Mat getDebugImg() const;
	virtual void draw(Mat& img, const cv::Scalar& col = DkUtils::yellow) const;
	virtual void draw(Mat& img, const std::vector<DkPolyRect>& rects, const cv::Scalar& col = DkUtils::yellow) const;
	virtual void setInputParams(const DkInputParameter& params);

	bool looseDetection;

protected:
	Mat img;
	Mat dbgImg;
	int thresh;
	int numThresh;
	double minArea;
	double maxArea;
	float maxSide;
	float maxSideFactor;
	float scale;
	bool detectCircles;

	std::vector<DkPolyRect> rects;
	std::vector<DkCircle> circles;

	virtual void releaseImages();
	virtual void checkInput() const;
	virtual Mat findRectangles(const Mat& img, std::vector<DkPolyRect>& squares) const;
	virtual void findCircles(const Mat& img, std::vector<DkCircle>& circles) const;
};