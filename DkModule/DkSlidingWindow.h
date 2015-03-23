/**************************************************
 * 	DkSlidingWindow.h
 *
 *	Created on:	01.06.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/
#pragma once

#include "DkModuleInclude.h"
#include "DkUtils.h"

#include "DkQuantization.h"

class DkProductInfo;

// data class
class DK_MODULE_API DkPolyRect {

public:
	//DkPolyRect(DkVector p1, DkVector p2, DkVector p3, DkVector p4);
	DkPolyRect(const std::vector<cv::Point>& pts = std::vector<cv::Point>());
	DkPolyRect(const std::vector<DkVector>& pts);

	bool empty() const;
	double getMaxCosine() const { return maxCosine; };
	void draw(Mat& img, const cv::Scalar& col = DkUtils::blue) const;
	std::vector<cv::Point> toCvPoints() const;
	std::vector<DkVector> getCorners() const;
	DkBox getBBox() const;
	double intersectArea(const DkPolyRect& pr) const;
	double getArea();
	double getAreaConst() const;
	void scale(float s);
	void scaleCenter(float s);
	bool inside(const DkVector& vec) const;
	float maxSide() const;
	DkVector center() const;
	static bool compArea(const DkPolyRect& pl, const DkPolyRect& pr);

protected:
	std::vector<DkVector> pts;
	double maxCosine;
	double area;

	void toDkVectors(const std::vector<cv::Point>& pts, std::vector<DkVector>& dkPts) const;
	void computeMaxCosine();
};

class DK_MODULE_API DkCircle {

public:
	DkCircle(const DkVector& center = DkVector(), float radius = 0.0f);
	DkCircle(const cv::Vec3f& circle);
	DkCircle(const cv::KeyPoint& kpt);

	bool isEmpty() const;
	void scale(float s);
	void scaleCenter(float s);
	DkBox getBBox() const;
	DkVector getCenter() const;
	float getRadius() const;
	float getArea() const;
	std::vector<DkVector> toPoly() const;
	void draw(Mat& img, const cv::Scalar& col = DkUtils::yellow) const;

	bool inside(const DkVector& vec) const;

protected:
	DkVector center;
	float radius;
};

class DK_MODULE_API DkSlidingWindow {

public:

	enum classResult {
		DK_GT_UNKNOWN = -1,
		DK_FALSE_POSITIVE,
		DK_TRUE_POSITIVE
	};

	DkSlidingWindow();
	virtual ~DkSlidingWindow();

	friend Mat& operator<<(Mat& img, const DkSlidingWindow& win);

	void setInputParams(const DkInputParameter& params);

	void getStorageBuffer(char** buffer, size_t& length) const;
	void setStorageBuffer(const char* buffer);

	int getLabelResult() const;
	bool isEmpty() const;
	bool isBowEmpty() const;

	void clearDescriptors();

	void setWindow(const DkBox& window);
	DkBox getWindow() const;

	bool collectFeatures(const std::vector<KeyPoint>& keypoints, const Mat& descriptors);
	void computeBoW();

	void setKeypoints(const std::vector<KeyPoint>& keypoints);
	std::vector<KeyPoint> getKeyPoints() const;

	void setDescriptors(const Mat& descriptors);
	Mat getDescriptors() const;

	void setBoWDescriptor(const Mat& bowDescriptor);
	Mat getBoWDescriptor() const;

	void setLabel(int label);
	int getLabel() const;

	void setGtLabel(int label);
	int getGtLabel() const;

	float getProbability() const;

	void setClassResponse(const Mat& classResponse);
	Mat getClassResponse() const;

	float getReliability() const;

	void setPolyRect(const DkPolyRect& polyRect);
	DkPolyRect getPolyRect() const;

	void setCircle(const DkCircle& circle);
	DkCircle getCircle() const;

	void draw(Mat& img, cv::Scalar col = Scalar()) const;

	void filterKeypoints();
	void filterKeypoints(const DkPolyRect& poly);
	void filterKeypoints(const DkCircle& circle);

	void setHist(const Mat& hist);
	Mat getHist() const;
	void calcHist(const Mat& hueImg);
	void track(const Mat& img);

protected:
	DkBox window;
	std::vector<DkVector> polyPts;
	DkCircle circle;

	Mat descriptors;
	std::vector<KeyPoint> keypoints;
	std::vector<int> descriptorIdx;

	Mat classResponse;
	Mat bowDescriptor;
	int label;
	int gtLabel;
	float rescaleArea;

	//DkBoW bow;

	// tracking
	Mat hist; // needed for tracking

	bool removeLarge;
	float minSize;
	int minFeatures;

	Mat poly2Mat(const std::vector<DkVector>& poly) const;
	std::vector<DkVector> mat2Poly(const Mat& mat) const;
};

class DK_MODULE_API DkLabelManager {

public:
	enum gtFormat
	{
		gt_label = 0,
		gt_filename,
		gt_path,
	};
	DkLabelManager(const std::string& filename = "unknown", int labelFormat = gt_label);

	/**
	 * Adds a label name to the lookup
	 * @param label the new label name
	 * @return int the corresponding label index
	 **/ 
	int addLabel(const std::string& label, bool* labelAdded = 0);
	int addLabel(bool* labelAdded = 0);
	int getLabelIdx(const std::string& label) const;
	std::string getLabel(int idx) const;
	void labelWindows(std::vector<DkSlidingWindow>& slidingWindows, const std::string& filename = "");
	void labelWindows(std::vector<DkProductInfo>& slidingWindows, const std::string& filename = "");
	std::string convertToLabel(const std::string& str) const;
	std::string convertPathToLabel(const std::string& str) const;
	std::map<int, std::string> getAllLabels() const;
	int getNumClasses() const;
	int getNumActiveClasses() const;
	int getGtLabelIdx() const;

	static void clearActiveList();
	static void addActiveItem(const std::string& item);
	static void setActiveClassList(const std::vector<std::string>& ignoreList);
	static std::vector<std::string> getActiveClassList();
	std::vector<int> getActiveIdxList() const;
	bool acitveClass(int labelIdx) const;

	static bool write(const std::string& filePath, const std::string& filename);
	static bool loadLookup(const std::string& filePath, const std::string& filename);

protected:
	static std::vector<std::string> activeList;
	static std::map<int, std::string> labelLookup;
	std::string labelName;
};
