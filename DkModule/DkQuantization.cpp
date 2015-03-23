/**************************************************
 * 	DkQuantization.cpp
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/
#include "DkQuantization.h"
#include "DkObjectRecognition.h"

Mat DkBoW::bowVocabulary = Mat();

// DkBow --------------------------------------------------------------------
DkBoW::DkBoW(int clusterMode) {

	this->clusterMode = clusterMode;
	className = "BoW";
	maxNearestClusters = 3;

	if (clusterMode == cluster_cv)
		bowMatcher = new BFMatcher(NORM_L2);
}

void DkBoW::setInputParams(const DkInputParameter& /*params*/) {

	// TOOD: dummy
}

void DkBoW::compute() {

	compute(Mat());
}

Mat DkBoW::compute(const Mat& descriptors, const std::vector<KeyPoint>& keypoints) {

	checkInput(descriptors);

	// reject rectangles with too few features
	if (descriptors.rows < bowVocabulary.rows) {
		mout << "[" << className << "] Rejecting rectangle, too few features: " << descriptors.rows << dkendl;
		return bowFeature;
	}

	if (clusterMode == cluster_cv) {
		if (bowMatcher->getTrainDescriptors().empty()) {
			bowMatcher->add(vector<Mat>(1, bowVocabulary));
		}

		bowFeature = computeCV(descriptors);
	}
	else if (clusterMode == cluster_fuzzy)
		bowFeature = computeFuzzy(descriptors, keypoints);
	else
		wout << "[" << className << "] cannot cluster, unknown mode: " << clusterMode << dkendl;

	return bowFeature;

}

Mat DkBoW::computeCV(const Mat& descriptors) {

	DkTimer dt;
	//if (descriptors.empty() || descriptors.cols != bowVocabulary.cols)
	//	return bowFeature;

	Mat bowFeature;

	// Match keypoint descriptors to cluster center (to vocabulary)
	std::vector<DMatch> matches;
	bowMatcher->match(descriptors, matches);

	bowFeature = Mat(1, bowVocabulary.rows, CV_32FC1, Scalar::all(0.0));
	float *dptr = (float*)bowFeature.data;
	
	for(size_t idx = 0; idx < matches.size(); idx++) {
		//int queryIdx = matches[idx].queryIdx;
		int trainIdx = matches[idx].trainIdx; // cluster index

		dptr[trainIdx]++;
	}

	// Normalize image descriptor.
	bowFeature /= descriptors.rows;

	//DkUtils::getMatInfo(bowFeature, "bow");
	//mout << "[" << className << "] CV BoW computed in: " << dt << dkendl;

	return bowFeature;
}

Mat DkBoW::computeFuzzy(const Mat& descriptors, const std::vector<KeyPoint>&) const {

	Mat bowFeature = Mat(1, bowVocabulary.rows, CV_32FC1, Scalar(0));
	Mat dist(1, bowVocabulary.rows, CV_64FC1);	// intermediate distances & indexes

	for (int dIdx = 0; dIdx < descriptors.rows; dIdx++) {

		Mat cFeature = descriptors.row(dIdx);
		double* cDist = dist.ptr<double>();

		for (int cIdx = 0; cIdx < bowVocabulary.rows; cIdx++) {

			Mat cCenter = bowVocabulary.row(cIdx);
			cDist[cIdx] = compareHist(cFeature, cCenter, CV_COMP_CHISQR);
		}

		Mat sDistIdx;
		sortIdx(dist, sDistIdx, CV_SORT_ASCENDING);

		cDist = dist.ptr<double>();
		double sumDists = 0;

		for (int idx = 0; idx < maxNearestClusters && idx < bowVocabulary.rows; idx++)
			sumDists += cDist[idx];

		Mat cWeights(bowFeature.size(), CV_32FC1, Scalar(0));

		float* bowPtr = cWeights.ptr<float>();
		int* idxPtr = sDistIdx.ptr<int>();

		for (int idx = 0; idx < maxNearestClusters && idx < bowVocabulary.rows; idx++)
			bowPtr[idxPtr[idx]] += (float)cDist[idx]/(float)sumDists;

		bowFeature += cWeights;	// add to current bow
	}

	normalize(bowFeature, bowFeature, 1.0, 0.0, NORM_MINMAX);
	//bowFeature = bowFeature.mul(bowFeature);

	return bowFeature;
}

void DkBoW::saveVocabulary(const Mat& img, const std::vector<KeyPoint>& keypoints, const Mat& descriptors, std::string dirName, std::string fileName) {

	checkInput(descriptors);

	bowFeature = Mat(1, bowVocabulary.rows, CV_32FC1, Scalar(0));
	Mat dist(1, bowVocabulary.rows, CV_64FC1);	// intermediate distances & indexes

	for (int dIdx = 0; dIdx < descriptors.rows; dIdx++) {

		// skip tiny keypoints
		if (keypoints[dIdx].size < 64)
			continue;

		Mat cFeature = descriptors.row(dIdx);
		double* cDist = dist.ptr<double>();

		for (int cIdx = 0; cIdx < bowVocabulary.rows; cIdx++) {

			Mat cCenter = bowVocabulary.row(cIdx);
			cDist[cIdx] = compareHist(cFeature, cCenter, CV_COMP_CHISQR);
		}

		Mat sDist, sDistIdx;
		sort(dist, sDist, CV_SORT_ASCENDING);
		sortIdx(dist, sDistIdx, CV_SORT_ASCENDING);

		cDist = dist.ptr<double>();
		double sumDists = 0;

		for (int idx = 0; idx < maxNearestClusters || idx < bowVocabulary.rows; idx++)
			sumDists += cDist[idx];

		Mat cWeights(bowFeature.size(), CV_32FC1, Scalar(0));

		int* idxPtr = sDistIdx.ptr<int>();

		DkBox b;
		DkVector s = DkVector(keypoints[dIdx].size, keypoints[dIdx].size);
		b.uc = keypoints[dIdx].pt;
		b.uc -= s*0.5;
		b.setSize(s);
		b.clip(img.size());
		Mat cImg = img(b.getCvRect());
		std::string fName = dirName + DkUtils::stringify(idxPtr[0]) + "-" + DkUtils::createFileName(fileName, "-" + DkUtils::stringify(rand()));
		mout << fName << dkendl;
		DkIP::imwrite(fName, DkIP::swapChannels(cImg.clone()));
	}

	mout << "results written to: " << fileName << dkendl;
}


Mat DkBoW::drawClone(const Mat& img, const DkBox& box, const Scalar& col) const {

	Mat res = img.clone();
	draw(res, box, col);

	return res;
}


void DkBoW::draw(Mat& img, const DkBox& box, const Scalar& col) const {

	DkIP::drawHistToImage(img, bowFeature, box, col);
}

void DkBoW::draw(Mat& img, const Mat& bowFeature, const DkBox& box, const Scalar& col /* = DkUtils::blueDark */) {

	DkIP::drawHistToImage(img, bowFeature, box, col);
}

bool DkBoW::loadVocabulary(const std::string& vocPath, const std::string& vocFileName) {

	FileStorage fs(vocPath + vocFileName, FileStorage::READ);

	if (!fs.isOpened()) {
		DebugResources::wout() << "Sorry, I could not read from: " << vocPath + vocFileName << dkendl;
		return false;
	}

	FileNode kpNode = fs["vocabulary"];
	if (kpNode.empty()) {
		DebugResources::wout() << "Sorry, there is no vocabulary in: " << vocPath + vocFileName  << dkendl;
		return false;
	}
	cv::read(kpNode, bowVocabulary);

	DebugResources::mout() << "BoW vocabulary loaded from: " << vocPath + vocFileName << " [" << bowVocabulary.rows << " x " << bowVocabulary.cols << "]" << dkendl;

	fs.release();

	return true;
}

// getter --------------------------------------------------------------------
Mat DkBoW::getBowFeature() const {
	return bowFeature;
}

// init etc --------------------------------------------------------------------
void DkBoW::checkInput(const Mat& descriptors) const {

	checkInput();

	if (descriptors.empty()) {
		std::string msg = "Descriptors must be assigned before applying BoW.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
	if (descriptors.cols != bowVocabulary.cols) {
		std::string msg = "Wrong vocabulary (" + DkUtils::stringify(bowVocabulary.cols) + " D) loaded. Feature dimension: " + 
			DkUtils::stringify(descriptors.cols) + "\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}

}

void DkBoW::checkInput() const {

	if (bowVocabulary.empty()) {
		std::string msg = "The vocabulary must be computed before applying BoW.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}

}

std::string DkBoW::toString() const {

	std::string str;
	str += "welcome to the brand new " + className;

	return str;
}