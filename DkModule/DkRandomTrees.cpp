/***************************************************
 *   DkSegmentation.cpp
 *   
 *   Created on: 12.02.2010
 *       Author: Markus Diem
 *      Company: Vienna University of Technology
 ***************************************************/


#include "DkRandomTrees.h"

// DkRandomTrees --------------------------------------------------------------------
std::string DkRTrees::toString() const {

	std::string msg;
	msg += "training error: " + DkUtils::stringify(oob_error, 10) + 
		" trees/classes: [" + DkUtils::stringify(ntrees) + "|" + DkUtils::stringify(nclasses) + "]";

	return msg;
}

DkRandomTrees::DkRandomTrees(const DkMSData& data, const cv::Mat&  suImg) {

	className = "DkRandomTrees";
	this->imgs = data;
	this->suImg = suImg;
	nSamples = 1000;
}

cv::Mat DkRandomTrees::getPredictedImage() const {
	
	return pImg;
}

void DkRandomTrees::compute() {

	DkTimer dt;
	cv::Mat data = imgs.getSignal();
	data.convertTo(data, CV_32FC1, 1.0f/255.0f);
	cv::Ptr<cv::StatModel> model = trainOnline(data, suImg);
	pImg = predictImage(data, model);
	mout << "[" << className << "] computed in " << dt << dkendl;
}

cv::Mat DkRandomTrees::predictImage(const cv::Mat& data, const cv::Ptr<cv::StatModel>& classifier) const {

	cv::Ptr<DkRTrees> rt = classifier;
	cv::Mat probabilities(1, data.rows, CV_32FC1);
	float* pPtr = probabilities.ptr<float>();

	for (int rIdx = 0; rIdx < data.rows; rIdx++) {
		cv::Mat feature = data.row(rIdx);
		pPtr[rIdx] = rt->predict_prob(feature);
		//moutc << feature << " p: " << pPtr[rIdx] << dkendl;
	}

	probabilities = imgs.columnVectorToImage(probabilities);

	return probabilities;
}

cv::Ptr<cv::StatModel> DkRandomTrees::trainOnline(const cv::Mat& data, const cv::Mat& fgdImg) const {

	cv::Mat labels = imgs.imageToColumnVector(fgdImg);

	cv::Mat tData = data.clone();
	converData(tData, labels, nSamples);
	
	cv::Ptr<DkRTrees> classifier = new DkRTrees();

	//for (int rIdx = 0; rIdx < tData.rows; rIdx++)
	//	moutc << tData.row(rIdx) << " p: " << labels.row(rIdx) << dkendl;

	CvRTParams p(7, 50, 0, false, 10, 0, false, 0, 25, 0.01f, CV_TERMCRIT_ITER+CV_TERMCRIT_EPS);
	if (!classifier->train(tData, CV_ROW_SAMPLE, labels, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), p))
		moutc << "[" << className << "] sorry, I could not train the random trees model..." << dkendl;
	else
		moutc << "[" << className << "] " << classifier->toString() << dkendl;

	return classifier;
}

void DkRandomTrees::converData(cv::Mat& trainData, cv::Mat& labels, int numPos) const {

	Mat posSamples, negSamples;

	labels = labels.t();
	const unsigned char* lPtr = labels.ptr<unsigned char>();

	for (int lIdx = 0; lIdx < labels.cols; lIdx++) {

		if (lPtr[lIdx] == 255)
			posSamples.push_back(trainData.row(lIdx));
		else if (lPtr[lIdx] == 0)
			negSamples.push_back(trainData.row(lIdx));
	}

	if (posSamples.empty() || negSamples.empty()) {
		woutc << "warning: too few data for training..." << dkendl;
		return;
	}

	// downsample
	cv::Size s(trainData.cols, cv::min(cv::min(numPos, posSamples.rows), negSamples.rows));
	resize(posSamples, posSamples, s, 0, 0, CV_INTER_NN);
	resize(negSamples, negSamples, s, 0, 0, CV_INTER_NN);

	trainData = posSamples;
	trainData.push_back(negSamples);

	//trainData.convertTo(trainData, CV_32FC1, 1.0f/255.0f);
	
	// generate GT
	Mat posLabels(posSamples.rows, 1, CV_32SC1, Scalar(1));
	Mat negLabels(negSamples.rows, 1, CV_32SC1, Scalar(0));

	labels = posLabels;
	labels.push_back(negLabels);

}

std::string DkRandomTrees::toString() const {

	std::string msg;
	msg += "[" + className + "]";

	return msg;
}

void DkRandomTrees::checkInput() const {

}


