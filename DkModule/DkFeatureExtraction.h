/**************************************************
 * 	DkFeatureExtraction.h
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "DkModule.h"
#include "DkFeature.h"
#include "DkIP.h"
#include "DkSlidingWindow.h"
#include "DkQuantization.h"

class DkInputParameter;

class DK_MODULE_API DkFeatureExtraction : public DkModule {

public:
	DkFeatureExtraction(Ptr<FeatureDetector> detector = Ptr<FeatureDetector>(), Ptr<DescriptorExtractor> descriptor = Ptr<DescriptorExtractor>());

	virtual void compute();
	virtual void compute(const Mat& img, const cv::Rect& object);
	virtual void compute(const Mat& img, const std::vector<cv::Rect>& objects = std::vector<cv::Rect>());
	virtual void compute(const Mat& img, const std::vector<DkProductInfo>& objects);
	virtual void computeFeaturesFast(const Mat& img, const std::vector<cv::Rect>& objects = std::vector<cv::Rect>());
	virtual void computeFeatures(const Mat& img, const std::vector<cv::Rect>& objects = std::vector<cv::Rect>());
	virtual void filterKeypoints(const std::vector<cv::Rect>& objects);
	virtual void filterKeypoints(const Mat& segImg);
	virtual void filterDescriptors(const std::vector<int>& rowIdxs, Mat& desc) const;

	void setInputParams(const DkInputParameter& params);

	virtual std::string toString() const;
	virtual std::string printParams() const;

	Mat draw(const Mat& img, const cv::Scalar& color = DkUtils::blue) const;
	static Mat draw(const Mat& img, const std::vector<KeyPoint>& kps, const cv::Scalar& color = DkUtils::blue);

	std::vector<KeyPoint> getKeypoints() const;
	Mat getDescriptors() const;
	std::string getFileAttribute() const;
	void setFileAttribute(const std::string& fileAttribute);

	void setMinSize(int minSize);
	int getMinSize() const;

	void setMaxNumDescriptors(int maxDescriptors);
	int getMaxNumDescriptors() const;

	Ptr<FeatureDetector> getFeatureDetector() const;
	Ptr<DescriptorExtractor> getDescriptorExtractor() const;

	void enableFast(bool enable);
	bool isFastEnabled() const;
	
	bool write(const std::string& path, const std::string& filename) const;
	bool read(const std::string& path, const std::string& filename);

	friend std::ostream& operator<<(std::ostream& os, const KeyPoint& kp) {

		os << DkUtils::keyPointToString(kp);

		return os;
	};

protected:

	Ptr<FeatureDetector> mDetector;
	Ptr<DescriptorExtractor> mDescriptor;
	
	std::vector<KeyPoint> keypoints;
	Mat descriptors;

	std::string fileAttribute;

	int minSize;
	int maxNumDescriptors; // this is needed for speed ups during BoW construction
	float rescaleArea;

	bool pyramidAdapted;
	bool oponentAdapted;
	bool fast;

	void checkInput(const Mat& img) const;
	void checkInput() const;
	virtual void computeFeaturesIntern(const Mat& img, std::vector<cv::KeyPoint>& keypoints, Mat& descriptors, const std::vector<cv::Rect>& objects = std::vector<cv::Rect>()) const;
	virtual void filterKeypointsIntern(const std::vector<cv::Rect>& objects, std::vector<cv::KeyPoint>& keypoints, Mat& descriptors) const;
};

class DK_MODULE_API DkBagger : public DkModule {

public:
	DkBagger(const std::vector<KeyPoint>& keypoints, const Mat& descriptors);
	virtual ~DkBagger() {};

	virtual void compute(const std::vector<DkProductInfo>& objects);
	virtual void compute(const std::vector<cv::Rect>& objects);
	virtual void compute();
	virtual std::vector<DkSlidingWindow>& getWindows();

	Mat draw(const Mat& img) const;
	static Mat draw(const Mat& img, const std::vector<DkSlidingWindow>& windows);
	static Mat draw(const Mat& img, const std::vector<DkProductInfo>& infos);

	virtual std::string toString() const;

	void setWindowSize(const DkVector& winSize);
	DkVector getWindowSize() const;

	void setWindowStep(const DkVector& winStep);
	DkVector getWindowStep() const;

	void setBagRegion(bool bagRegion);

	bool write(const std::string& path, const std::string& filename) const;
	bool read(const std::string& path, const std::string& filename);

	static std::string fileAttribute;

protected:
	std::vector<KeyPoint> keypoints;
	Mat descriptors;
	Mat bowDescriptors;
	std::vector<DkSlidingWindow> slidingWindows;

	DkVector winSize;
	DkVector winStep;
	
	void checkInput(const Mat&img) const;
	void checkInput() const;
	DkBox getRoi(std::vector<KeyPoint> keypoints);
	bool bagRegion;


};

bool dkCompDistKeyPt(const cv::DMatch& ml, const cv::DMatch& mr);

class DK_MODULE_API DkTemplateMatcher : public DkModule {

public:
	DkTemplateMatcher(const std::vector<DkSlidingWindow>& windows = std::vector<DkSlidingWindow>());

	virtual void compute();
	virtual Mat matchWindow(const DkSlidingWindow& win) const;
	virtual float singleMatch(const DkSlidingWindow& win, const DkSlidingWindow& templateWin) const;
	virtual std::vector<DkSlidingWindow> getSlidingWindows() const;
	virtual std::string toString() const;

	static bool load(const std::string& filePath, const std::string& filename, std::vector<DkSlidingWindow>& slidingWindows);
	static bool write(const std::string& filePath, const std::string& filename, const std::vector<DkSlidingWindow>& slidingWindows);

	static std::vector<DkSlidingWindow> templateWindows;
	
protected:
	std::vector<DkSlidingWindow> windows;
	float responseThresh;

	virtual void checkInput() const;
	DkSlidingWindow findTemplateWin(int idx) const;
};

