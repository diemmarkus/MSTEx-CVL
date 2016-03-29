/*******************************************************************************************************
 DkRandomTrees.cpp
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Markus Diem <markus@nomacs.org>
 Copyright (C) 2014-2015 Fabian Hollaus <holl@caa.tuwien.ac.at>
 Vienna University of Technology, Computer Vision Lab
 This file is part of ViennaMS.

 ViennaMS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 ViennaMS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *******************************************************************************************************/

#include "DkRandomTrees.h"

#ifdef DK_STANDALONE

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
	cv::Ptr<cv::ml::RTrees> model = trainOnline(data, suImg);
	pImg = predictImage(data, model);
	mout << "[" << className << "] computed in " << dt << dkendl;
}

cv::Mat DkRandomTrees::predictImage(const cv::Mat& data, const cv::Ptr<cv::ml::RTrees>& classifier) const {

	cv::Ptr<cv::ml::RTrees> rt = classifier;
	cv::Mat probabilities(1, data.rows, CV_32FC1);
	float* pPtr = probabilities.ptr<float>();

	for (int rIdx = 0; rIdx < data.rows; rIdx++) {
		cv::Mat feature = data.row(rIdx);
		pPtr[rIdx] = rt->predict(feature);
		//moutc << feature << " p: " << pPtr[rIdx] << dkendl;
	}

	probabilities = imgs.columnVectorToImage(probabilities);

	return probabilities;
}

cv::Ptr<cv::ml::RTrees> DkRandomTrees::trainOnline(const cv::Mat& data, const cv::Mat& fgdImg) const {

	cv::Mat labels = imgs.imageToColumnVector(fgdImg);

	cv::Mat tData = data.clone();
	converData(tData, labels, nSamples);



	cv::Ptr<cv::ml::RTrees> classifier = cv::ml::RTrees::create();
	classifier->setMaxDepth(7);
	classifier->setMinSampleCount(50);
	classifier->setCalculateVarImportance(false);
	classifier->setActiveVarCount(10);
	classifier->setRegressionAccuracy(.01f);

	//for (int rIdx = 0; rIdx < tData.rows; rIdx++)
	//	moutc << tData.row(rIdx) << " p: " << labels.row(rIdx) << dkendl;
	cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(tData, cv::ml::ROW_SAMPLE, labels);



	if (!classifier->train(td))
		moutc << "[" << className << "] sorry, I could not train the random trees model..." << dkendl;
	else
		moutc << "[" << className << "] " << "trained..." << dkendl;

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
#endif

