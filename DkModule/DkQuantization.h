/**************************************************
 * 	DkQuantization.h
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once

#include "DkModuleInclude.h"
#include "DkModule.h"
#include "DkUtils.h"


class DkInputParameter;

class DK_MODULE_API DkBoW : public DkModule {

public:
	DkBoW(int clusterMode = cluster_cv);

	void setInputParams(const DkInputParameter& params);
	virtual void compute();
	virtual Mat compute(const Mat& descriptors, const std::vector<KeyPoint>& keypoints = std::vector<KeyPoint>());
	virtual void saveVocabulary(const Mat& img, const std::vector<KeyPoint>& keypoints, const Mat& descriptors, std::string dirName, std::string fileName);

	virtual std::string toString() const;
	static bool loadVocabulary(const std::string& vocPath, const std::string& vocFileName);

	Mat getBowFeature() const;
	void draw(Mat& img, const DkBox& box, const Scalar& col) const;
	static void draw(Mat& img, const Mat& bowFeature, const DkBox& box, const Scalar& col = DkUtils::blueDark);
	Mat drawClone(const Mat& img, const DkBox& box, const Scalar& col) const;

	static Mat bowVocabulary;

	enum clusterModeEnum{
		cluster_fuzzy = 0,
		cluster_cv,

		cluster_end
	};

protected:
	Mat bowFeature;
	int maxNearestClusters;
	int clusterMode;
	Ptr<DescriptorMatcher> bowMatcher;

	void checkInput(const Mat& descriptors) const;
	void checkInput() const;
	Mat computeCV(const Mat& descriptors);
	Mat computeFuzzy(const Mat& descriptors, const std::vector<KeyPoint>& keypoints = std::vector<KeyPoint>()) const;
};

