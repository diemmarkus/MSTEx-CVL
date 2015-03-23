/***************************************************
 *   DkBaseStefan.cpp
 *   
 *   Created on: 08.11.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *				 Stefan Fiel
 *      Company: Vienna University of Technology
 ***************************************************/

#pragma once

#include "DkBase.h"

#include "DkFeatureExtraction.h"
#include "DkMachineLearning.h"
#include "DkObjectRecognition.h"
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;

class DkBaseBerlinger;


class DkBaseBerlinger : public DkBase {

public:
	DkBaseBerlinger();
	virtual ~DkBaseBerlinger() {};

	enum {
		TP = 0,
		FP,
		FN,

		STATS_END,
	};

	// methods which differs from each user, should be overwritten
	void init(void);
	virtual void createVocabulary(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void trainImage(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void trainImageObjDetect(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void trainSingleImageObjDetect(QSharedPointer<DkImageContainerT> imgFile, cv::Mat img, int scale);
	virtual void saveDebug(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void showVocabulary(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void detectObject(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void testAPI(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void track(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void evaluateResults(const std::vector<DkProductInfo>& windows, const DkLabelManager& labelManager, cv::Mat& confusionMatrix, cv::Mat& detectionStats);
	virtual void preLoad();
	virtual void postLoad();
	virtual DkBaseComputeFunction resolveFunction(int mode);
	virtual void trainBoW();
	virtual void trainML();
	virtual void evalImages();
	virtual void saveEvaluationResults(const std::string& fName);

	std::string configPath;

protected:
	// needed for BoW Clustering
	BOWKMeansTrainer bowTrainer;
	int numBowClusters;
	cv::Mat confusionMatrix;
	cv::Mat detectionStats;
	int tpImg;
	int fpImg;
	cv::Mat dbgImg;
	std::vector<QPoint> pts;

	// classifier
	DkClassifierTrainer classifierTrainer;
	std::vector<DkSlidingWindow> slidingWindows;
	std::vector<DkSlidingWindow> featureWindows; // collect features for matching

	// api test
	DkObjectRecognition recognizer;

	// object detection
	cv::CascadeClassifier boxCascade;

	std::vector<DkProductInfo> trackedObjects;

	virtual void folderFinished();
	virtual DkSlidingWindow getBestBox(const Mat& img, Mat& segImg, bool isBackground = false);
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void mouseClicked(QMouseEvent* event, QPoint imgPos);
	virtual void watershedSegmentation(QSharedPointer<DkImageContainerT> imgFile, const std::vector<QPoint>& pos);
		
};
