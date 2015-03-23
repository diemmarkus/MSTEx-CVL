/**************************************************
 * 	DkMachineLearning.h
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/
#pragma once

#include "DkModuleInclude.h"
#include "DkModule.h"
#include "DkSlidingWindow.h"
#include "DkUtils.h"

#include <ml.h>

// ok: this is a bit frustrating
// opencv has a bug so that we cannot query the training error
// and all interesting params are protected
// so this class is just designed to get a good statistics of the training
class DkRandomTrees : public cv::RandomTrees {

public:

	virtual std::string toString() const;

//protected:


};

class DK_MODULE_API DkClassifierTrainer : public DkModule {

public:
	DkClassifierTrainer();
	virtual ~DkClassifierTrainer() {};
	void releaseData();

	void addSamples(const std::vector<DkSlidingWindow>& slidingWindows);
	virtual void compute();
	virtual void train(bool saveMemory = true);
	virtual void train(const std::vector<DkSlidingWindow>& slidingWindows);

	std::map<int,int> convertData(const std::vector<DkSlidingWindow>& slidingWindows, Mat& trainData, Mat& labels) const;
	virtual std::string toString() const;

	void setSlidingWindows(const std::vector<DkSlidingWindow>& slidingWindows);
	std::vector<DkSlidingWindow> getSlidingWindows() const;

	virtual bool write(const std::string& filePath, const std::string& filename) const;
	bool read(cv::FileStorage& fs);
	std::vector<Ptr<cv::StatModel> > getModels() const;
	std::vector<std::pair<int,int> > getClasses() const;

	static int getModelIndex(const std::vector<std::pair<int, int> >& classes, const std::pair<int, int>& cPair);
	static int getModelIndex(const std::vector<std::pair<int, int> >& classes, const std::pair<int, int>& cPair, bool& isFlipped);

	Mat getFeatures() const;
	Mat getLabels() const;

protected:
	std::vector<Ptr<cv::StatModel> > models;
	std::vector<DkSlidingWindow> slidingWindows;
	Mat features;
	Mat labels;
	std::vector<std::pair<int,int> > classes;

	void checkInput() const;
	void convertDataClassifier(Mat& cSamples, Mat& cLabels, size_t myLabel, size_t otherLabel = -1) const;
	//virtual Ptr<cv::StatModel> initClassifier() = 0;
};

class DK_MODULE_API DkRandomTreesTrainer : public DkClassifierTrainer {

public:
	DkRandomTreesTrainer();
	DkRandomTreesTrainer(const DkClassifierTrainer& trainer);

	virtual void train(const std::vector<DkSlidingWindow>& slidingWindows);
	virtual std::string toString() const;

protected:
	virtual Ptr<cv::StatModel> initClassifier();
};

class DK_MODULE_API DkMultiClassRTTrainer : public DkClassifierTrainer {

public:
	DkMultiClassRTTrainer();
	DkMultiClassRTTrainer(const DkClassifierTrainer& trainer);

	virtual void train(const std::vector<DkSlidingWindow>& slidingWindows);
	std::map<int,int> convertData(const std::vector<DkSlidingWindow>& slidingWindows, Mat& trainData, Mat& labels) const;
	virtual std::string toString() const;

protected:
	virtual std::vector<Ptr<cv::StatModel> > initClassifiers();

};

class DK_MODULE_API DkOneVsOneMultiClassRTTrainer : public DkClassifierTrainer {

public:
	DkOneVsOneMultiClassRTTrainer();
	DkOneVsOneMultiClassRTTrainer(const DkClassifierTrainer& trainer);

	virtual void train(const std::vector<DkSlidingWindow>& slidingWindows);
	virtual void train(const std::vector<DkSlidingWindow>& slidingWindows, DkOneVsOneMultiClassRTTrainer* oldClassifier);
	virtual std::string toString() const;
	virtual bool write(const std::string& filePath, const std::string& filename) const;
	virtual bool load(const std::string& filePath, const std::string& filename);
	virtual bool readFeatures(const std::string& filePath, const std::string& filename);
	virtual void setOldClassifier(DkOneVsOneMultiClassRTTrainer* oldClassifier);

protected:
	virtual std::vector<Ptr<cv::StatModel> > initClassifiers();
	virtual bool writeFeatures(const std::string& filePath, const std::string& filename) const;
	DkOneVsOneMultiClassRTTrainer* oldClassifier;

};

class DK_MODULE_API DkClassifier : public DkModule {

public:
	enum types {
		DK_UNKNOWN_CLASSIFIER,
		DK_RANDOM_TREES,
		DK_RANDOM_TREES_ONE_VS_ALL,
		DK_RANDOM_TREES_ONE_VS_ONE,
	};

	DkClassifier();

	void compute() {};	// dummy
	void predict(std::vector<DkSlidingWindow>& windows) const;
	int predict(const Mat& feature) const;
	int getAccumulatedVoting(const std::vector<DkSlidingWindow>& windows) const;
	int type() const;

	static bool loadClassifier(const std::string& filePath, const std::string& filename);
	virtual std::string toString() const;

protected:
	static std::vector<Ptr<CvStatModel> > models;
	static std::vector<std::pair<int, int> > classes;
	static int classifierType;

	virtual void checkInput() const;
};

class DK_MODULE_API DkMultiClassifier : public DkClassifier {

public:
	void predict(std::vector<DkSlidingWindow>& windows) const;

	Mat predict(const Mat& feature) const;
	int getAccumulatedVoting(const std::vector<DkSlidingWindow>& windows) const;

protected:

};



