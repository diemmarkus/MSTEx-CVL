/**************************************************
 * 	DkMachineLearning.cpp
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkMachineLearning.h"
//
//std::vector<Ptr<cv::StatModel> > DkClassifier::models = std::vector<Ptr<cv::StatModel> >();
//std::vector<std::pair<int, int> > DkClassifier::classes = std::vector<std::pair<int, int> >();
//int DkClassifier::classifierType = DK_UNKNOWN_CLASSIFIER;
//
//// DkRandomTrees --------------------------------------------------------------------
//std::string DkRandomTrees::toString() const {
//
//	std::string msg;
//	msg += "training error: " + DkUtils::stringify(oob_error, 10) + 
// 		" trees/classes: [" + DkUtils::stringify(ntrees) + "|" + DkUtils::stringify(nclasses) + "]";
//
//	return msg;
//}
//
//// DkClassifierTrainer --------------------------------------------------------------------
//DkClassifierTrainer::DkClassifierTrainer() {
//
//	className = "DkClassifierTrainer";
//}
//
//void DkClassifierTrainer::releaseData() {
//	slidingWindows.clear();
//}
//
//void DkClassifierTrainer::addSamples(const std::vector<DkSlidingWindow>& slidingWindows) {
//
//	std::vector<DkSlidingWindow> sw = slidingWindows;
//
//	for (size_t idx = 0; idx < sw.size(); idx++)
//		sw[idx].clearDescriptors();		// free up space - we're just interested in the BoW descriptor
//
//	this->slidingWindows.insert(this->slidingWindows.end(), sw.begin(), sw.end());
//}
//
//void DkClassifierTrainer::compute() {
//	train();
//}
//
//void DkClassifierTrainer::train(bool saveMemory) {
//
//	train(slidingWindows);
//
//	if (saveMemory)
//		slidingWindows.clear();
//}
//
//void DkClassifierTrainer::train(const std::vector<DkSlidingWindow>& slidingWindows) {
//
//	checkInput();
//
//	mout << "[" << className << "] converting " << slidingWindows.size() << " features..." << dkendl;
//
//	// remove empty sliding windows beforehand - that makes things easier
//	std::vector<DkSlidingWindow> cleanedWins;
//	for (int idx = 0; idx < slidingWindows.size(); idx++) {
//		
//		if (!slidingWindows.at(idx).isBowEmpty())
//			cleanedWins.push_back(slidingWindows.at(idx));
//	}
//
//	mout << "[" << className << "] " << cleanedWins.size() << " features after cleaning..." << dkendl;
//
//
//	DkTimer dt;
//	convertData(cleanedWins, features, labels);
//	mout << "[" << className << "] features converted in " << dt << dkendl;
//}
//
//std::map<int, int> DkClassifierTrainer::convertData(const std::vector<DkSlidingWindow>& slidingWindows, Mat& trainData, Mat& labels) const {
//
//	DkTimer dt;
//
//	// assuming that sliding window is not empty (someone must call checkInput beforehand)
//	DkSlidingWindow win = slidingWindows[0];
//	int featDim = win.getBoWDescriptor().cols;
//
//	if (win.getBoWDescriptor().depth() != CV_32FC1) {
//		std::string msg = "The descriptors have the wrong format - Expected CV_32FC1.\n";
//		throw DkMatException(msg, __LINE__, __FILE__);
//	}
//
//	trainData = Mat((int)slidingWindows.size(), featDim, CV_32FC1, Scalar(0));
//	labels = Mat(1, (int)slidingWindows.size(), CV_32SC1, Scalar(0));
//	unsigned int* labelPtr = labels.ptr<unsigned int>();
//	
//	std::map<int, int> stats;
//	std::map<int,int>::iterator statsIter;
//
//	for (size_t idx = 0; idx < slidingWindows.size(); idx++) {
//
//		slidingWindows[idx].getBoWDescriptor().copyTo(trainData.row((int)idx));
//		labelPtr[idx] = slidingWindows[idx].getGtLabel();
//
//		statsIter = stats.find(slidingWindows[idx].getGtLabel());
//
//		if (statsIter == stats.end())
//			stats[slidingWindows[idx].getGtLabel()] = 1;
//		else
//			stats[slidingWindows[idx].getGtLabel()] = statsIter->second++;
//	}
//
//	return stats;
//
//}
//
//void DkClassifierTrainer::convertDataClassifier(Mat& cSamples, Mat& cLabels, size_t myLabel, size_t otherLabel) const {
//
//	const unsigned int* lPtr = labels.ptr<unsigned int>();
//	Mat posSamples, negSamples;
//
//	for (int lIdx = 0; lIdx < labels.cols; lIdx++) {
//
//		if (lPtr[lIdx] == myLabel)
//			posSamples.push_back(features.row(lIdx));
//		else if (lPtr[lIdx] == otherLabel || otherLabel == -1)
//			negSamples.push_back(features.row(lIdx));
//	}
//
//	if (posSamples.empty()) {
//		DkLabelManager manager;
//		DebugResources::mout() << "sorry, no positive samples, i cannot train: " << manager.getLabel((int)myLabel) << dkendl;
//		return;
//	}
//
//	resize(negSamples, negSamples, posSamples.size(), 0, 0, CV_INTER_NN);
//
//	Mat posLabels(posSamples.rows, 1, CV_32SC1, Scalar(1));
//	Mat negLabels(negSamples.rows, 1, CV_32SC1, Scalar(0));
//
//	cSamples.push_back(posSamples);
//	cSamples.push_back(negSamples);
//
//	cLabels.push_back(posLabels);
//	cLabels.push_back(negLabels);
//
//}
//
//void DkClassifierTrainer::setSlidingWindows(const std::vector<DkSlidingWindow>& slidingWindows) {
//	this->slidingWindows = slidingWindows;
//}
//
//std::vector<DkSlidingWindow> DkClassifierTrainer::getSlidingWindows() const {
//	return slidingWindows;
//}
//
//std::vector<Ptr<StatModel> > DkClassifierTrainer::getModels() const {
//
//	return models;
//}
//
//std::vector<std::pair<int, int> > DkClassifierTrainer::getClasses() const {
//
//	return classes;
//}
//
//Mat DkClassifierTrainer::getFeatures() const {
//
//	return features;
//}
//
//Mat DkClassifierTrainer::getLabels() const {
//
//	return labels;
//}
//
//// init --------------------------------------------------------------------
//void DkClassifierTrainer::checkInput() const {
//
//	if (slidingWindows.empty()) {
//		std::string msg = "[" + className + "] cannot train since the training data is empty.\n";
//		throw DkMatException(msg, __LINE__, __FILE__);
//	}
//}
//
//bool DkClassifierTrainer::write(const std::string& filePath, const std::string& filename) const {
//
//
//	if (models.empty()) {
//		std::cout << "WARNING: I cannot write an empty classifier table..." << std::endl;
//		return false;
//	}
//
//	std::string classifierFile = filePath + filename;
//
//	FileStorage fs(classifierFile, FileStorage::WRITE);
//
//	if (!fs.isOpened()) {
//		std::cout << "Sorry, I cannot write to: " << classifierFile << std::endl;
//		return false;
//	}
//
//	int numModels = (int)models.size();
//	fs << "numModels" << numModels;
//
//
//	if (!classes.empty()) {
//
//		fs << "classes" << "[";
//		for (int idx = 0; (size_t)idx < classes.size(); idx++) {
//			fs << classes.at(idx).first;
//			fs << classes.at(idx).second;
//		}
//		fs << "]";
//	}
//
//	for (size_t idx = 0; idx < models.size(); idx++) {
//		models[idx]->write(*fs, (className + DkUtils::stringify(idx)).c_str());
//	}
//	fs.release();
//
//	std::cout << "writing to: " << classifierFile << std::endl;
//
//	return true;
//}
//
//bool DkClassifierTrainer::read(cv::FileStorage& fs) {
//
//	int numModels = (int)fs["numModels"];
//
//	if (numModels != models.size()) {
//		return false;
//	}
//
//	FileNode n = fs["classes"];                         // Read string sequence - Get node
//
//	// read classifier lookup
//	if (!n.empty()) {
//
//		if (n.type() != FileNode::SEQ) {
//			DebugResources::wout() << "cannot load lookup table, lookupStrings is not a sequence" << dkendl;
//			return false;
//		}
//
//		FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
//		while (it != it_end) {
//			classes.push_back(std::pair<int, int>(*it++, *it++));
//		}
//	}
//
//	for (int idx = 0; idx < models.size(); idx++) {
//
//		FileNode kpNode = fs[className + DkUtils::stringify(idx)];
//		if (kpNode.empty())
//			return false;
//
//		models[idx]->read(*fs, *kpNode);	// *fs is not const?!
//	}
//
//	if (numModels <= 0) {
//		wout << "[" << className << "] illegal model count, cannot load classifier..." << dkendl;
//		return false;
//	}
//
//	return true;
//}
//
//std::string DkClassifierTrainer::toString() const {
//
//	std::string str;
//	str += "[" + className + "] -------------------------------------------\n";
//	str += DkUtils::stringify(features.rows) + " features with " + DkUtils::stringify(features.cols) + " dimensions.\n";
//
//	return str;
//}
//
//int DkClassifierTrainer::getModelIndex(const std::vector<std::pair<int, int> >& classes, const std::pair<int, int>& cPair) {
//
//	bool isFlipped = false;
//	return getModelIndex(classes, cPair, isFlipped);
//}
//
//int DkClassifierTrainer::getModelIndex(const std::vector<std::pair<int, int> >& classes, const std::pair<int, int>& cPair, bool& isFlipped) {
//
//	std::vector<std::pair<int,int> >::const_iterator cIter = std::find(classes.begin(), classes.end(), cPair);
//
//	if (cIter != classes.end()) {
//		isFlipped = false;
//		return (int)(cIter - classes.begin());
//	}
//
//	std::pair<int, int> fPair(cPair.second, cPair.first);
//
//	cIter = std::find(classes.begin(), classes.end(), fPair);
//
//	if (cIter != classes.end()) {
//		isFlipped = true;
//		return (int)(cIter - classes.begin());
//	}
//
//	return -1;	// not found
//}
//
//// DkRandomTreesTrainer --------------------------------------------------------------------
//DkRandomTreesTrainer::DkRandomTreesTrainer() {
//	models.push_back(initClassifier());
//}
//
//DkRandomTreesTrainer::DkRandomTreesTrainer(const DkClassifierTrainer& trainer) : DkClassifierTrainer(trainer) {
//	models.push_back(initClassifier());
//}
//
//void DkRandomTreesTrainer::train(const std::vector<DkSlidingWindow>& slidingWindows) {
//
//	DkClassifierTrainer::train(slidingWindows);
//
//	DkTimer dt;
//	if (models.empty()) {
//		wout << "[" << className << "] cannot train an empty model!" << dkendl;
//		return;
//	}
//
//	Ptr<DkRandomTrees> classifier = models[0];
//	if (!classifier) {
//		wout << "[" << className << "] cannot train an empty model!" << dkendl;
//		return;
//	}
//
//	if (!classifier->train(features, CV_ROW_SAMPLE, labels))
//		mout << "[" << className << "] sorry, I could not train the random trees model..." << dkendl;
//
//	mout << "[" << className << "] training takes " << dt << dkendl;
//}
//
//std::string DkRandomTreesTrainer::toString() const {
//
//	std::string str = DkClassifierTrainer::toString();
//
//	//Ptr<cv::RandomTrees> rtModel = model;
//
//	//str+= "Training error: " + DkUtils::stringify(rtModel->get_train_error());
//	//str+= "# trees: " + DkUtils::stringify(rtModel->get_tree_count());
//
//	return str;
//}
//
//// init etc. --------------------------------------------------------------------
//Ptr<cv::StatModel> DkRandomTreesTrainer::initClassifier() {
//	
//	className = "DkRandomTrees";
//	return new DkRandomTrees();
//}
//
//// DkMultiClassRTTrainer --------------------------------------------------------------------
//DkMultiClassRTTrainer::DkMultiClassRTTrainer() {
//	models = initClassifiers();
//}
//
//DkMultiClassRTTrainer::DkMultiClassRTTrainer(const DkClassifierTrainer& trainer) : DkClassifierTrainer(trainer) {
//	models = initClassifiers();
//}
//
//// init etc. --------------------------------------------------------------------
//std::vector<Ptr<cv::StatModel> > DkMultiClassRTTrainer::initClassifiers() {
//
//	className = "DkMultiClassRandomTrees";
//	std::vector<Ptr<cv::StatModel> > models;
//	DkLabelManager manager;
//
//	for (int idx = 0; idx < manager.getNumClasses(); idx++) {
//		models.push_back(new DkRandomTrees());
//	}
//
//	return models;
//}
//
//void DkMultiClassRTTrainer::train(const std::vector<DkSlidingWindow>& slidingWindows) {
//
//	DkClassifierTrainer::train(slidingWindows);
//	
//	DkTimer dt;
//	if (models.empty()) {
//		wout << "[" << className << "] cannot train an empty model!" << dkendl;
//		return;
//	}
//
//	for (size_t idx = 0; idx < models.size(); idx++) {
//
//		Ptr<DkRandomTrees> classifier = models[idx];
//		if (!classifier) {
//			wout << "[" << className << "] cannot train an empty model!" << dkendl;
//			return;
//		}
//
//		Mat cSamples, cLabels;
//		convertDataClassifier(cSamples, cLabels, (int)idx);
//
//		CvRTParams p(8, 10, 0, false, 10, 0, false, 0, 90, 0.001f, CV_TERMCRIT_ITER+CV_TERMCRIT_EPS);
//		
//		if (!classifier->train(cSamples, CV_ROW_SAMPLE, cLabels.t(), cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), p))
//			mout << "[" << className << "] sorry, I could not train the random trees model..." << dkendl;
//	}
//
//	mout << "[" << className << "] training takes " << dt << dkendl;
//}
//
//std::map<int, int> DkMultiClassRTTrainer::convertData(const std::vector<DkSlidingWindow>& slidingWindows, Mat& trainData, Mat& labels) const {
//	
//	// assuming that sliding window is not empty (someone must call checkInput beforehand)
//	DkSlidingWindow win = slidingWindows[0];
//	int featDim = win.getBoWDescriptor().cols;
//
//	const std::map<int, int>& stats = DkClassifierTrainer::convertData(slidingWindows, trainData, labels);
//	std::map<int, int>::const_iterator statsIter = stats.begin();
//	int minElements = INT_MAX;
//
//	for (int idx = 0; idx < stats.size(); idx++) {
//
//		int cStats = statsIter->second;
//
//		if (cStats < minElements)
//			minElements = cStats;
//
//		statsIter++;
//	}
//
//	moutc << "min num elements: " << minElements << std::endl;
//
//	statsIter = stats.begin();
//	Mat trainDataEqual;
//	Mat labelsEqual;
//
//	for (int idx = 0; idx < stats.size(); idx++) {
//
//		unsigned int cLabel = statsIter->first;
//
//		Mat cClass;
//		//float* cClassPtr = cClass.ptr<float>();
//		const unsigned int* labelPtr = labels.ptr<unsigned int>();
//		int classIdx = 0;
//
//		moutc << "label: " << cLabel << std::endl;
//
//		for (int rIdx = 0; rIdx < labels.cols; rIdx++) {
//
//			if (labelPtr[rIdx] == cLabel) {
//				cClass.push_back(trainData.row(rIdx));
//				classIdx++;
//			}
//		}
//
//		DkUtils::getMatInfo(cClass, "cClass");
//
//		cv::resize(cClass, cClass, cv::Size(featDim, minElements), 0, 0, CV_INTER_NN);
//
//		Mat cLabels(minElements, 1, CV_32SC1, Scalar(cLabel));
//
//		trainDataEqual.push_back(cClass);
//		labelsEqual.push_back(cLabels);
//
//		statsIter++;
//	}
//
//	labelsEqual = labelsEqual.t();
//
//	moutc << "feature size before removing samples: " << trainData.rows << std::endl;
//	moutc << "feature size after removing samples: " << trainDataEqual.rows << std::endl;
//		
//	trainData = trainDataEqual;
//	labels = labelsEqual;
//		
//	return stats;
//}
//
//std::string DkMultiClassRTTrainer::toString() const {
//
//	std::string str = DkClassifierTrainer::toString();
//
//	//Ptr<cv::RandomTrees> rtModel = model;
//
//	//str+= "Training error: " + DkUtils::stringify(rtModel->get_train_error());
//	//str+= "# trees: " + DkUtils::stringify(rtModel->get_tree_count());
//
//	return str;
//}
//
//// DkMultiClassRTTrainer --------------------------------------------------------------------
//DkOneVsOneMultiClassRTTrainer::DkOneVsOneMultiClassRTTrainer() {
//	
//	models = initClassifiers();
//}
//
//DkOneVsOneMultiClassRTTrainer::DkOneVsOneMultiClassRTTrainer(const DkClassifierTrainer& trainer) : DkClassifierTrainer(trainer) {
//	models = initClassifiers();
//}
//
//
//// init etc. --------------------------------------------------------------------
//std::vector<Ptr<cv::StatModel> > DkOneVsOneMultiClassRTTrainer::initClassifiers() {
//
//	className = "DkOneVsOneMultiClassRandomTrees";
//	oldClassifier = 0;
//	std::vector<Ptr<cv::StatModel> > models;
//	DkLabelManager manager;
//
//	int numClassifiers = manager.getNumClasses()*(manager.getNumClasses()-1)/2;
//	mout << "initializing (K*(K+1)/2) = " << numClassifiers << " classifiers where K = " << manager.getNumClasses() << dkendl;
//
//	for (int idx = 0; idx < numClassifiers; idx++) {
//		models.push_back(new DkRandomTrees());
//	}
//
//	return models;
//}
//
//void DkOneVsOneMultiClassRTTrainer::train(const std::vector<DkSlidingWindow>& slidingWindows) {
//
//	train(slidingWindows, oldClassifier);
//}
//
//void DkOneVsOneMultiClassRTTrainer::train(const std::vector<DkSlidingWindow>& slidingWindows, DkOneVsOneMultiClassRTTrainer* oldClassifier) {
//
//	DkClassifierTrainer::train(slidingWindows);
//
//	DkTimer dt;
//	if (models.empty()) {
//		wout << "[" << className << "] cannot train an empty model!" << dkendl;
//		return;
//	}
//
//	std::vector<std::pair<int,int> > newClasses;
//	std::vector<std::pair<int,int> > oldClasses;
//	std::vector<Ptr<StatModel> > oldModels;
//
//	if (oldClassifier && !oldClassifier->getLabels().empty()) {
//
//		DkUtils::getMatInfo(labels, "new labels");
//		DkUtils::getMatInfo(oldClassifier->getLabels(), "old labels");
//
//		features.push_back(oldClassifier->getFeatures());
//		labels = labels.t();
//		Mat nL = oldClassifier->getLabels().t();
//		labels.push_back(nL);
//		labels = labels.t();
//
//		oldClasses = oldClassifier->getClasses();
//		oldModels = oldClassifier->getModels();
//	}
//
//	mout << "[" << className << "] training a total of " << features.rows << " features (" << oldClassifier->getFeatures().rows << " old| " << features.rows-oldClassifier->getFeatures().rows << " new)" << dkendl;
//
//	DkLabelManager manager;
//
//	for (size_t idx = 0; idx < manager.getNumClasses(); idx++) {
//		
//		for (size_t oIdx = 0; oIdx < manager.getNumClasses(); oIdx++) {
//
//			bool isFlipped = false;
//			std::pair<int,int> cClassPair((int)idx, (int)oIdx);
//			int oldModelIdx = getModelIndex(oldClasses, cClassPair, isFlipped);
//
//			if (idx == oIdx || getModelIndex(newClasses, cClassPair) != -1)
//				continue;
//
//			Ptr<DkRandomTrees> classifier = models[newClasses.size()];
//
//			// no old model found
//			if (isFlipped || oldModelIdx == -1) {
//
//				if (!classifier) {
//					wout << "[" << className << "] cannot train an empty model!" << dkendl;
//					return;
//				}
//
//				Mat cSamples, cLabels;
//				convertDataClassifier(cSamples, cLabels, idx, oIdx);
//
//				// OpenCV Default: 
//				//CvRTParams p(5, 10, 0, false, 10, 0, false, 0, 50, 0.1, CV_TERMCRIT_EPS+CV_TERMCRIT_ITER);
//				CvRTParams p(7, 50, 0, false, 10, 0, false, 0, 80, 0.001f, CV_TERMCRIT_ITER+CV_TERMCRIT_EPS);
//
//				if (!classifier->train(cSamples, CV_ROW_SAMPLE, cLabels.t(), cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), p))
//					mout << "[" << className << "] sorry, I could not train the random trees model..." << dkendl;
//				else
//					mout << classifier->toString() << dkendl;
//
//			}
//			else
//				classifier = oldModels[oldModelIdx];
//
//			newClasses.push_back(cClassPair);
//		}
//	}
//
//	classes = newClasses;
//
//	mout << "[" << className << "] training takes " << dt << dkendl;
//}
//
//void DkOneVsOneMultiClassRTTrainer::setOldClassifier(DkOneVsOneMultiClassRTTrainer* oldClassifier) {
//	
//	this->oldClassifier = oldClassifier;
//}
//
//bool DkOneVsOneMultiClassRTTrainer::write(const std::string& filePath, const std::string& filename) const {
//
//	if (!writeFeatures(filePath, DkUtils::createFileName(filename, "-features")))
//		return false;	// early break
//
//	return DkClassifierTrainer::write(filePath, filename);
//}
//
//bool DkOneVsOneMultiClassRTTrainer::writeFeatures(const std::string& filePath, const std::string& filename) const {
//
//	std::string featureFile = filePath + filename;
//
//	FileStorage fs(featureFile, FileStorage::WRITE);
//
//	if (!fs.isOpened()) {
//		std::cout << "Sorry, I cannot write to: " << featureFile << std::endl;
//		return false;
//	}
//
//	fs << "features" << features;
//	fs << "labels" << labels;
//
//	fs.release();
//
//	std::cout << "writing to: " << featureFile << std::endl;
//
//	return true;
//}
//
//bool DkOneVsOneMultiClassRTTrainer::load(const std::string& filePath, const std::string& filename) {
//
//
//	std::string classifierPath = filePath + filename;
//	FileStorage fs(classifierPath, FileStorage::READ);
//
//	if (!fs.isOpened()) {
//		DebugResources::wout() << "Sorry, I could not read from: " << classifierPath << dkendl;
//		return false;
//	}
//
//	bool readOk = !read(fs);
//		
//	fs.release();
//
//	if (!readOk)
//		return readOk;
//
//	return readFeatures(filePath, DkUtils::createFileName(filename, "-features"));
//}
//
//bool DkOneVsOneMultiClassRTTrainer::readFeatures(const std::string& filePath, const std::string& filename) {
//
//	std::string featureFile = filePath + filename;
//
//	FileStorage fs(featureFile, FileStorage::READ);
//
//	if (!fs.isOpened()) {
//		mout << "Sorry, I cannot read features from: " << featureFile << dkendl;
//		return false;
//	}
//
//	fs["features"] >> features;
//	fs["labels"] >> labels;
//
//	fs.release();
//	mout << labels.cols << " features read from: " << featureFile << dkendl;
//
//	return true;
//}
//
//std::string DkOneVsOneMultiClassRTTrainer::toString() const {
//
//	std::string str = DkClassifierTrainer::toString();
//
//	//Ptr<cv::RandomTrees> rtModel = model;
//
//	//str+= "Training error: " + DkUtils::stringify(rtModel->get_train_error());
//	//str+= "# trees: " + DkUtils::stringify(rtModel->get_tree_count());
//
//	return str;
//}
//
//// DkClassifier --------------------------------------------------------------------
//DkClassifier::DkClassifier() {
//
//}
//
//void DkClassifier::predict(std::vector<DkSlidingWindow>& windows) const {
//
//	for (size_t idx = 0; idx < windows.size(); idx++) {
//
//		windows[idx].setLabel(predict(windows[idx].getBoWDescriptor()));
//	}
//}
//
//int DkClassifier::predict(const Mat& feature) const {
//
//	checkInput();
//
//	switch(classifierType) {
//		case DK_RANDOM_TREES:
//			Ptr<RandomTrees> classifier = models[0];
//			return (int)classifier->predict(feature);
//			break;
//		//default:
//		//	std::cout << "[" << className << "] unknown classifier, where it should not be" << std::endl;
//	}
//
//	return -1;
//}
//
//int DkClassifier::getAccumulatedVoting(const std::vector<DkSlidingWindow>& windows) const {
//
//	DkLabelManager lm;
//	Mat votingMat(1, lm.getNumClasses(), CV_32SC1, Scalar(0));
//	unsigned int* votingPtr = votingMat.ptr<unsigned int>();
//
//	for (size_t idx = 0; idx < windows.size(); idx++) {
//
//		int cLabel = windows[idx].getLabel();
//		if (cLabel < 0 || cLabel >= votingMat.cols) {
//			std::cout << "Illegal label: " << cLabel << " skipping..." << std::endl;
//			continue;
//		}
//		votingPtr[cLabel]++;
//	}
//
//	int maxClassIdx = -1;
//	unsigned int maxVal = 0;
//
//	for (int cIdx = 0; cIdx < votingMat.cols; cIdx++) {
//		
//		if (votingPtr[cIdx] > maxVal) {
//			maxClassIdx = cIdx;
//			maxVal = votingPtr[cIdx];
//		}
//	}
//
//	// there are bugs arround here...
//	//cv::minMaxIdx(votingMat, 0, 0, 0, &maxClassIdx);
//
//	return maxClassIdx;
//}
//
//int DkClassifier::type() const {
//
//	return classifierType;
//}
//
//bool DkClassifier::loadClassifier(const std::string& filePath, const std::string& filename) {
//
//	std::string classifierPath = filePath + filename;
//	FileStorage fs(classifierPath, FileStorage::READ);
//
//	if (!fs.isOpened()) {
//		DebugResources::wout() << "Sorry, I could not read from: " << classifierPath << dkendl;
//		return false;
//	}
//
//	DkTimer dt;
//
//	// try all classifiers we know
//	DkRandomTreesTrainer rt;
//	DkMultiClassRTTrainer mrt;
//	DkOneVsOneMultiClassRTTrainer omrt;
//	
//	if (rt.read(fs)) {
//		models = rt.getModels();
//		classifierType = DK_RANDOM_TREES;
//		DebugResources::mout() << "Classifier loaded from: " << classifierPath << " in: " << dt << dkendl;
//		return true;
//	} else if (mrt.read(fs)) {
//		models = mrt.getModels();
//		classifierType = DK_RANDOM_TREES_ONE_VS_ALL;
//		DebugResources::mout() << "Classifier loaded from: " << classifierPath << " in: " << dt << dkendl;
//		return true;
//	} else if (omrt.read(fs)) {
//		models = omrt.getModels();
//		classes = omrt.getClasses();
//		classifierType = DK_RANDOM_TREES_ONE_VS_ONE;
//
//		//DkLabelManager m;
//
//		//for (int idx = 0; idx < models.size(); idx++) {
//
//		//	Ptr<DkRandomTrees> c = models[idx];
//		//	moutc << c->toString() << "[" << m.getLabel(classes.at(idx).first) << "|" << m.getLabel(classes.at(idx).second) << "] " << dkendl;
//		//}
//
//		DebugResources::mout() << models.size() << " Classifiers loaded from: " << classifierPath << " in: " << dt << dkendl;
//		return true;
//	}
//	else {
//		DebugResources::wout() << "Sorry, I don't know the classifier stored in: " << classifierPath << dkendl;
//	}
//
//
//	return false;
//}
//
//std::string DkClassifier::toString() const {
//
//	std::string str;
//	str += "[" + className + "] ";
//	str += models.empty() ? " empty model " : " model loaded";
//
//	return str;
//}
//
//void DkClassifier::checkInput() const {
//
//	if (models.empty()) {
//		std::string msg = "[" + className + "] the classifier model is empty.\n have you forgotten to load it?\n";
//		throw DkMatException(msg, __LINE__, __FILE__);
//	}
//
//}
//
//// DkMultiClassifier --------------------------------------------------------------------
//void DkMultiClassifier::predict(std::vector<DkSlidingWindow>& windows) const {
//
//	for (size_t idx = 0; idx < windows.size(); idx++) {
//
//		windows[idx].setClassResponse(predict(windows[idx].getBoWDescriptor()));
//	}
//}
//
//Mat DkMultiClassifier::predict(const Mat& feature) const {
//
//	checkInput();
//
//	if (feature.empty())
//		return Mat();
//
//	switch(classifierType) {
//	case DK_RANDOM_TREES_ONE_VS_ALL: {
//
//		Mat classResponses(1, (int)models.size(), CV_32FC1, Scalar(0));
//		float* crPtr = classResponses.ptr<float>();
//
//		for (size_t idx = 0; idx < models.size(); idx++) {
//
//			Ptr<RandomTrees> classifier = models[idx];
//			crPtr[idx] = classifier->predict_prob(feature);
//		}
//		return classResponses;
//	}
//	case DK_RANDOM_TREES_ONE_VS_ONE: {
//
//		DkLabelManager manager;
//		
//		Mat classResponses(1, manager.getNumClasses(), CV_32FC1, Scalar(0));
//		float* crPtr = classResponses.ptr<float>();
//				
//		for (size_t idx = 0; idx < models.size(); idx++) {
//			
//			int cIdx = classes[idx].first;
//			int oIdx = classes[idx].second;
//		
//			if (!manager.acitveClass(cIdx) || !manager.acitveClass(oIdx))
//				continue;
//			
//			Ptr<DkRandomTrees> classifier = models[idx];
//
//			float pb = classifier->predict_prob(feature);
//			//float p = classifier->predict(feature);
//			//crPtr[cIdx] += 1.0f-p;
//			//crPtr[oIdx] += p;
//			crPtr[cIdx] += 1.0f-pb;
//			crPtr[oIdx] += pb;
//
//			//moutc << "cIdx: " << cIdx << " [" << 1.0f-pb << "|" << 1.0f-p << "] ";
//			//moutc << "oIdx: " << oIdx << " [" << pb << "|" << p << "]" << dkendl;
//
//		}
//
//		classResponses /= (manager.getNumActiveClasses()-1);
//
//		return classResponses;
//	}
//
//	default:
//		std::cout << "[" << className << "] unknown classifier, where it should not be" << std::endl;
//	}
//
//	return Mat();
//}
//
//int DkMultiClassifier::getAccumulatedVoting(const std::vector<DkSlidingWindow>& windows) const {
//
//	DkLabelManager lm;
//	Mat votingMat(1, lm.getNumClasses(), CV_32FC1, Scalar(0));
//	unsigned int* votingPtr = votingMat.ptr<unsigned int>();
//
//	for (size_t idx = 0; idx < windows.size(); idx++) {
//
//		if (windows[idx].getClassResponse().cols == votingMat.cols)
//			votingMat += windows[idx].getClassResponse();
//		else
//			std::cout << "Ignoring sliding window, it's class responses have the wrong dimensionality: " << windows[idx].getClassResponse().cols << std::endl;
//	}
//
//	votingMat /= (float)windows.size();
//
//	int maxClassIdx = -1;
//	unsigned int maxVal = 0;
//
//	for (int cIdx = 0; cIdx < votingMat.cols; cIdx++) {
//
//		if (votingPtr[cIdx] > maxVal) {
//			maxClassIdx = cIdx;
//			maxVal = votingPtr[cIdx];
//		}
//	}
//
//	// there are bugs around here...
//	//cv::minMaxIdx(votingMat, 0, 0, 0, &maxClassIdx);
//
//	return maxClassIdx;
//}
