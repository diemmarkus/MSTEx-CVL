/**************************************************
 * 	DkFeatureExtraction.cpp
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkFeatureExtraction.h"
#include "DkObjectRecognition.h"

#include "opencv2/calib3d/calib3d.hpp"

std::string DkBagger::fileAttribute = "-bow.yml";
std::vector<DkSlidingWindow> DkTemplateMatcher::templateWindows = std::vector<DkSlidingWindow>();

DkFeatureExtraction::DkFeatureExtraction(Ptr<FeatureDetector> detector, Ptr<DescriptorExtractor> descriptor) : 
	mDetector(detector), mDescriptor(descriptor) {

	className = "DkFeatureExtraction";
	fileAttribute = "-features.yml";
	pyramidAdapted = false;
	oponentAdapted = false;
	fast = true;
	rescaleArea = 1.0f;

	minSize = 12;
	minSize = 0;	// diem: test all features for small burgers
	maxNumDescriptors = INT_MAX;

	//if (!mDetector){ 
		mDetector = new SiftFeatureDetector(0, 3, 0.006);
	//}

	if (pyramidAdapted)
		mDetector = new PyramidAdaptedFeatureDetector(mDetector, 9);

	if (!mDescriptor)
		mDescriptor = DescriptorExtractor::create("SIFT");
		//mDescriptor = DescriptorExtractor::create("SIFT");

	if (oponentAdapted)
		mDescriptor = new cv::OpponentColorDescriptorExtractor(mDescriptor);

}

void DkFeatureExtraction::setInputParams(const DkInputParameter& params) {

	rescaleArea = params.rescaleArea;
}


// Computer Vision --------------------------------------------------------------------
void DkFeatureExtraction::compute() {

	compute(Mat());	// fails
}

void DkFeatureExtraction::compute(const Mat& img, const cv::Rect& object) {

	std::vector<cv::Rect> objects;
	objects.push_back(object);

	compute(img, objects);
}

void DkFeatureExtraction::compute(const Mat& img, const std::vector<DkProductInfo>& objects) {

	std::vector<cv::Rect> rects;
	for (size_t idx = 0; idx < objects.size(); idx++)
		rects.push_back(objects[idx].getRectangle().getCvRect());

	compute(img, rects);
}

void DkFeatureExtraction::compute(const Mat& img, const std::vector<cv::Rect>& objects) {
	
	checkInput(img);

	if (fast)
		computeFeaturesFast(img, objects);
	else
		computeFeatures(img, objects);
}

void DkFeatureExtraction::computeFeaturesFast(const Mat& img, const std::vector<cv::Rect>& objects /* = std::vector<cv::Rect> */) {

	for (size_t idx = 0; idx < objects.size(); idx++) {

		// skip empty rectangles
		if (!objects.at(idx).width || !objects.at(idx).height)
			continue;

		DkBox b = objects.at(idx);

		if (rescaleArea != 1.0f)
			b.scaleAboutCenter(rescaleArea);

		b.clip(img.size());

		Mat imgRoi = img(b.getCvRect());

		if (imgRoi.empty())
			continue;

		std::vector<cv::KeyPoint> tmpKp;
		Mat tmpDesc;

		computeFeaturesIntern(imgRoi, tmpKp, tmpDesc);

		// copy features
		//DkBox roi = objects.at(idx);

		for (int idx = 0; idx < tmpKp.size(); idx++) {
			cv::KeyPoint kp = tmpKp.at(idx);
			kp.pt = Point2f(kp.pt.x + b.uc.x, kp.pt.y + b.uc.y);	// shift kps back to image coordinates
			keypoints.push_back(kp);
		}
		descriptors.push_back(tmpDesc);
	}

	mout << keypoints.size() << " keypoints detected..." << dkendl;
}

void DkFeatureExtraction::computeFeatures(const Mat& img, const std::vector<cv::Rect>& objects /* = std::vector<cv::Rect> */) {

	computeFeaturesIntern(img, keypoints, descriptors, objects);

}

void DkFeatureExtraction::computeFeaturesIntern(const Mat& img, std::vector<cv::KeyPoint>& keypoints, Mat& descriptors, const std::vector<cv::Rect>& objects) const {

	mDetector->detect(img, keypoints);
	
	// filter keypoints with respect to their size
	if (minSize != 0) {

		std::vector<KeyPoint> tmpKeyPts;

		for (size_t idx = 0; idx < keypoints.size(); idx++) {

			if (keypoints[idx].size >= minSize) {
				tmpKeyPts.push_back(keypoints[idx]);
			}
		}

		keypoints = tmpKeyPts;
	}

	//mout << keypoints.size() << " keypoints detected..." << dkendl;

	filterKeypointsIntern(objects, keypoints, descriptors);

	mDescriptor->compute(img, keypoints, descriptors);
	descriptors.convertTo(descriptors, CV_32F);
}

void DkFeatureExtraction::filterKeypoints(const std::vector<cv::Rect>& objects) {

	filterKeypointsIntern(objects, keypoints, descriptors);
}

void DkFeatureExtraction::filterKeypointsIntern(const std::vector<cv::Rect>& objects, std::vector<cv::KeyPoint>& keypoints, Mat& descriptors) const {

	std::vector<int> rowIdxs;
	std::vector<KeyPoint> keyPointsWithin;

	if (!objects.empty() && !fast) {

		for (int idx = 0; idx < keypoints.size(); idx++) {

			DkVector tmpKeyPoint = keypoints.at(idx).pt;

			for (int rIdx = 0; rIdx < objects.size(); rIdx++) {

				DkBox cb = objects.at(rIdx);

				if (rescaleArea != 1.0f)
					cb.scaleAboutCenter(rescaleArea);

				if (cb.within(tmpKeyPoint)) {
					rowIdxs.push_back(idx);
					keyPointsWithin.push_back(keypoints.at(idx));
					break;
				}
			}
		}

		keypoints = keyPointsWithin;
	}		
		
	if (keypoints.size() > maxNumDescriptors) {
			
		int step = cvRound((float)keypoints.size()/maxNumDescriptors);

		if (step > 1) {
			
			keyPointsWithin.clear();
			rowIdxs.clear();

			for (int idx = 0; idx < keypoints.size(); idx+=step) {
				rowIdxs.push_back(idx);
				keyPointsWithin.push_back(keypoints.at(idx));
			}

			keypoints = keyPointsWithin;
		}
	}
		
	//mout << keypoints.size() << " keypoints after object detection..." << dkendl;
	if (!rowIdxs.empty())
		filterDescriptors(rowIdxs, descriptors);
}

void DkFeatureExtraction::filterKeypoints(const Mat& segImg) {

	if (segImg.empty())
		return;

	std::vector<int> rIdxs;
	std::vector<cv::KeyPoint> tmpKp;
	
	const unsigned char* ptr = segImg.ptr<unsigned char>();

	for (int idx = 0; idx < keypoints.size(); idx++) {

		const cv::KeyPoint& kp = keypoints.at(idx);
		int x = cvRound(kp.pt.x);
		int y = cvRound(kp.pt.y);

		if (x < 0 || x > segImg.cols || y < 0 || y > segImg.rows) {
			mout << "[" << className << "] WARNING, keypoint out of bounds..." << dkendl;
			tmpKp.push_back(kp);
			continue;
		}

		if (ptr[y*segImg.cols+x] != 0) {
			tmpKp.push_back(kp);
			rIdxs.push_back(idx);
		}
	}

	keypoints = tmpKp;

	filterDescriptors(rIdxs, descriptors);

}

void DkFeatureExtraction::filterDescriptors(const std::vector<int>& rowIdxs, Mat& desc) const {

	// filter descriptors too, if they are already computed
	if (!desc.empty()) {

		Mat tmpDesc;

		for (int rIdx = 0; rIdx < (int)rowIdxs.size(); rIdx++) {

			if (rowIdxs[rIdx] < desc.rows)
				tmpDesc.push_back(desc.row(rIdx));
		}

		desc = tmpDesc;
	}
}

Mat DkFeatureExtraction::draw(const Mat& img, const cv::Scalar& color) const {
	
	Mat res = draw(img, keypoints, color);
	return res;
}

Mat DkFeatureExtraction::draw(const Mat& img, const std::vector<KeyPoint>& kps, const cv::Scalar& color) {

	std::vector<KeyPoint> skps;

	for (size_t idx = 0; idx < kps.size(); idx++) {
		KeyPoint kp = kps[idx];
		kp.size *= 2;			// it seems that OpenCV draws the scale wrong (r != d)
		skps.push_back(kp);
	}

	Mat res;
	drawKeypoints(img, skps, res, color, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	return res;
}

// getter --------------------------------------------------------------------
std::vector<KeyPoint> DkFeatureExtraction::getKeypoints() const {
	return keypoints;
}

Mat DkFeatureExtraction::getDescriptors() const {
	return descriptors;
}

Ptr<FeatureDetector> DkFeatureExtraction::getFeatureDetector() const {
	return mDetector;
}

Ptr<DescriptorExtractor> DkFeatureExtraction::getDescriptorExtractor() const {
	return mDescriptor;
}

int DkFeatureExtraction::getMinSize() const {
	return minSize;
}

void DkFeatureExtraction::setMinSize(int minSize) {
	this->minSize = minSize;
}

int DkFeatureExtraction::getMaxNumDescriptors() const {
	return maxNumDescriptors;
}

void DkFeatureExtraction::setMaxNumDescriptors(int maxDescriptors) {
	this->maxNumDescriptors = maxDescriptors;
}

std::string DkFeatureExtraction::getFileAttribute() const {
	return fileAttribute;
}

void DkFeatureExtraction::setFileAttribute(const std::string& fileAttribute) {
	this->fileAttribute = fileAttribute;
}

void DkFeatureExtraction::enableFast(bool enable) {
	fast = enable;
}

bool DkFeatureExtraction::isFastEnabled() const {
	return fast;
}

// init etc --------------------------------------------------------------------
bool DkFeatureExtraction::write(const std::string& path, const std::string& filename) const {

	if (!mDescriptor || !mDetector) {
		std::cout << "WARNING: Descriptor or Detector is empty where it should not be..." << std::endl;
		return false;
	}
	
	std::string detFName = path + filename + fileAttribute;
	std::cout << "writing to: " << detFName << std::endl;
	FileStorage fs(detFName, FileStorage::WRITE);

	if (!fs.isOpened()) {
		std::cout << "Sorry, I cannot write to: " << detFName << std::endl;
		return false;
	}

	//mDetector->write(fsDet);	// TODO: for now opencv has lots of bugs in I/O of detectors
	cv::write(fs, "keypoints", keypoints);
	cv::write(fs, "descriptors", descriptors);
	
	fs.release();

	return true;
}

bool DkFeatureExtraction::read(const std::string& path, const std::string& filename) {
	
	std::string filePath = path + filename + fileAttribute;
	FileStorage fs(filePath, FileStorage::READ);
	
	if (!fs.isOpened()) {
		mout << "Sorry, I could not read from: " << filePath << dkendl;
		return false;
	}
	
	FileNode kpNode = fs["keypoints"];
	if (kpNode.empty()) {
		mout << "Sorry, there are no keypoints in: " << filePath << dkendl;
		return false;
	}
	cv::read(kpNode, keypoints);
	
	FileNode descNode = fs["descriptors"];
	if (descNode.empty()) {
		mout << "Sorry, there are no descriptors in: " << filePath << dkendl;
		return false;
	}

	cv::read(descNode, descriptors);
	
	fs.release();

	return true;	// CHANGE! - dummy
}

void DkFeatureExtraction::checkInput(const Mat& img) const {

	if (img.empty()) {
		std::string msg = "The image img is empty.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}

	checkInput();
}

void DkFeatureExtraction::checkInput() const {

	if (!mDetector) {
		std::string msg = "Your Detector is empty.\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
	if (!mDescriptor) {
		std::string msg = "Your Descriptor is empty.\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

std::string DkFeatureExtraction::toString() const {

	if (!mDescriptor || !mDetector)
		return "WARNING: Descriptor or Detector is empty where it should not be...";

	std::string str;
	str += "\n" + className + " -----------------------------------------\n";
	str += "Detector: " + ((pyramidAdapted) ? "Pyramid" : mDetector->info()->name()) + " #keypoints: " + DkUtils::stringify(keypoints.size()) + "\n";
	str += "Descriptor: " + (oponentAdapted) ? "Opponent" : mDescriptor->name() + " (" + DkUtils::stringify(descriptors.cols) + " dimensional)";
	//for (size_t idx = 0; idx < keypoints.size(); idx++) {
	//	str += DkUtils::keyPointToString(keypoints[idx]) + "\n";
	//}
	// TODO....
	str += "\n" + className + " -----------------------------------------\n";

	return str;
}

std::string DkFeatureExtraction::printParams() const {

	if (!mDescriptor || !mDetector)
		return "WARNING: Descriptor or Detector is empty where it should not be...";

	std::string str;
	std::vector<std::string> params;

	// fixes OpenCV bug
	if (!pyramidAdapted) {
		str += mDetector->info()->name() + "-----\n";

		mDetector->info()->getParams(params);

		for (size_t idx = 0; idx < params.size(); idx++)
			str += params[idx] + "\n";

		params.clear();
	}

	str += mDescriptor->info()->name() + "-----\n";
	mDescriptor->info()->getParams(params);

	for (size_t idx = 0; idx < params.size(); idx++)
		str += params[idx] + "\n";

	return str;
}

// DkBagger --------------------------------------------------------------------
DkBagger::DkBagger(const std::vector<KeyPoint>& keypoints, const Mat& descriptors) {

	className = "DkBagger";

	// default parameter
	winSize = DkVector(150,150);
	winStep = winSize*0.25;
	bagRegion = false;

	this->keypoints = keypoints;
	this->descriptors = descriptors;
}

void DkBagger::compute(const std::vector<DkProductInfo>& objects) {

	for (int idx = 0; idx < objects.size(); idx++) {
		DkSlidingWindow win = objects.at(idx).getSlidingWindowConst();
		win.collectFeatures(keypoints, descriptors);

		// I am not sure if this place is appropriate to filter the keypoins
		// so if you need it somewhere else - don't hesitate to do so
		win.filterKeypoints();

		if (!win.getDescriptors().empty())
			win.computeBoW();
		slidingWindows.push_back(win);
	}
}

void DkBagger::compute(const std::vector<cv::Rect>& objects) {

	for (int idx = 0; idx < objects.size(); idx++) {
		DkSlidingWindow win;
		win.setWindow(objects.at(idx));
		win.collectFeatures(keypoints, descriptors);

		if (!win.getDescriptors().empty())
			win.computeBoW();
		slidingWindows.push_back(win);
	}
}

void DkBagger::compute() {

	DkTimer dt;
	DkBox roi = getRoi(keypoints);

	if (keypoints.empty() || !roi.size().width || !roi.size().height) {
		wout << "[" << className << "] sorry, I could not extract any sliding window..." << dkendl;
		return;
	}

	//winStep *= 0.25;

	// cache bow descriptors for speed-up
	bowDescriptors = Mat(descriptors.rows, DkBoW::bowVocabulary.rows, CV_32FC1, Scalar(0));

	DkVector steps((roi.size()-winSize) / winStep);
	steps.ceil();

	if (!steps.x || !steps.y) {
		wout << "Cannot compute sliding windows because the ROI is too small, sorry..." << dkendl;
		return;
	}

	// adopt win size
	winSize.width = roi.getWidth()/(float)cvFloor(roi.getWidth()/winSize.width);
	winSize.height = roi.getHeight()/(float)cvFloor(roi.getHeight()/winSize.height);

	mout << "new window size: " << winSize << dkendl;

	for (int rIdx = 0; rIdx < steps.height; rIdx++) {

		DkBox box;
		box.setSize(winSize);
		box.moveBy(roi.uc);		// start at ROI's top left corner
		box.moveBy(DkVector(0, rIdx*winStep.y));

		for (int cIdx = 0; cIdx < steps.width; cIdx++) {

			DkSlidingWindow cWin;
			cWin.setWindow(box);

			if (cWin.collectFeatures(keypoints, descriptors)) {
				cWin.computeBoW();
				slidingWindows.push_back(cWin);
			}

			box.moveBy(DkVector(winStep.x, 0));
		}
	}

	if (bagRegion) {
		DkSlidingWindow sWin;
		sWin.setWindow(roi);
		sWin.collectFeatures(keypoints, descriptors);
		sWin.computeBoW();

		slidingWindows.push_back(sWin);
	}

	mout << "[" << className << "] sliding windows computed in " << dt << dkendl;
}

DkBox DkBagger::getRoi(std::vector<KeyPoint> keypoints) {

	DkBox roi;
	roi.uc = DkVector(FLT_MAX, FLT_MAX);

	for (size_t idx = 0; idx < keypoints.size(); idx++) {

		DkVector coords = keypoints[idx].pt;
		roi.uc.minVec(coords);
		roi.lc.maxVec(coords);
	}

	// be sure to include all even if some dummy compares using <
	roi.uc.floor();
	roi.lc.ceil();

	mout << "Region of interest: " << roi << dkendl;

	return roi;
}

// getters --------------------------------------------------------------------
void DkBagger::setWindowSize(const DkVector& winSize) {
	this->winSize = winSize;
}

DkVector DkBagger::getWindowSize() const {
	return winSize;
}

void DkBagger::setWindowStep(const DkVector& winStep) {
	this->winStep = winStep;
}

DkVector DkBagger::getWindowStep() const {
	return winStep;
}

std::vector<DkSlidingWindow>& DkBagger::getWindows() {
	return slidingWindows;
}

Mat DkBagger::draw(const Mat& img) const {

	return DkBagger::draw(img, slidingWindows);
}

Mat DkBagger::draw(const Mat& img, const std::vector<DkSlidingWindow>& windows) {

	Mat res = img.clone();
	for (size_t idx = 0; idx < windows.size(); idx++) 
		res << windows[idx];

	return res;
}

Mat DkBagger::draw(const Mat& img, const std::vector<DkProductInfo>& infos) {

	Mat res = img.clone();
	for (size_t idx = 0; idx < infos.size(); idx++) 
		res << infos[idx].getSlidingWindowConst();

	return res;
}

void DkBagger::setBagRegion(bool bagRegion) {
	this->bagRegion = bagRegion;
}

// init etc --------------------------------------------------------------------
bool DkBagger::write(const std::string& path, const std::string& filename) const {

	if (slidingWindows.empty()) {
		std::cout << "WARNING: Sliding windows are empty where they should not be..." << std::endl;
		return false;
	}

	std::string detFName = path + filename + fileAttribute;
	std::cout << "writing to: " << detFName << std::endl;
	FileStorage fs(detFName, FileStorage::WRITE);

	if (!fs.isOpened()) {
		std::cout << "Sorry, I cannot write to: " << detFName << std::endl;
		return false;
	}

	Mat bowDescriptors((int)slidingWindows.size(), DkBoW::bowVocabulary.rows, CV_32FC1, Scalar(0));
	int gtLabel = -1;

	for (size_t idx = 0; idx < slidingWindows.size(); idx++) {
		
		gtLabel = slidingWindows[idx].getGtLabel();
		Mat d = slidingWindows[idx].getBoWDescriptor();
		d.copyTo(bowDescriptors.row((int)idx));
	}

	cv::write(fs, "gtLabel", gtLabel);
	cv::write(fs, "bowDescriptors", bowDescriptors);

	fs.release();

	return true;
}

bool DkBagger::read(const std::string& path, const std::string& filename) {

	std::string filePath = path + filename + fileAttribute;
	FileStorage fs(filePath, FileStorage::READ);

	if (!fs.isOpened()) {
		mout << "Sorry, I could not read from: " << filePath << dkendl;
		return false;
	}

	int gtLabel = -1;
	fs["gtLabel"] >> gtLabel;
	
	Mat bowDescriptors;
	fs["bowDescriptors"] >> bowDescriptors;
	fs.release();
	
	if (bowDescriptors.empty()) {
		mout << "Sorry, there are no descriptors in: " << filePath << dkendl;
		return false;
	}

	for (int rIdx = 0; rIdx < bowDescriptors.rows; rIdx++) {

		DkSlidingWindow sWin;

		Mat cDesc = bowDescriptors.row(rIdx);
		sWin.setBoWDescriptor(cDesc);

		slidingWindows.push_back(sWin);
	}

	return true;	// CHANGE! - dummy
}

void DkBagger::checkInput(const Mat& img) const {

	if (img.empty()) {
		std::string msg = "The image img is empty.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
}

void DkBagger::checkInput() const {

	if (keypoints.empty()) {
		std::string msg = "Your Detector is empty.\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
	if (descriptors.empty()) {
		std::string msg = "Your Descriptor is empty.\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

std::string DkBagger::toString() const {

	std::string str;
	str += "this is the brand new " + className + " module...";

	return str;
}

bool dkCompDistKeyPt(const cv::DMatch& ml, const cv::DMatch& mr) {

	return ml.distance < mr.distance;
}

// DkTemplateMatcher --------------------------------------------------------------------
DkTemplateMatcher::DkTemplateMatcher(const std::vector<DkSlidingWindow>& windows) : DkModule() {
	
	this->windows = windows;
	className = "DkTemplateMatcher";

	responseThresh = 0.3f;
}

void DkTemplateMatcher::checkInput() const {

}

void DkTemplateMatcher::compute() {

	DkTimer dt;

	for (DkSlidingWindow& win : windows) {

		Mat matchMat = matchWindow(win);
		//cv::normalize(matchMat, matchMat, 1.0f, NORM_MINMAX);
		win.setClassResponse(matchMat);
	}

	mout << "[" << className << "] matching takes: " << dt << dkendl;
}

Mat DkTemplateMatcher::matchWindow(const DkSlidingWindow& win) const {

	Mat votingMat = win.getClassResponse();
	
	if (votingMat.empty())
		return Mat();

	Mat matchMat(votingMat.size(), CV_32FC1, Scalar(0));
	Mat matchMask(votingMat.size(), CV_32FC1, Scalar(0));
	const float* vPtr = votingMat.ptr<float>();
	float* mPtr = matchMat.ptr<float>();
	float* maskPtr = matchMask.ptr<float>();

	double maxVote = 0;
	cv::minMaxLoc(votingMat, 0, &maxVote);
	float thresh = (float)(maxVote-responseThresh);

	if (cv::sum(votingMat > thresh)[0]/255.0f <= 1) {
		moutc << "[" << className << "] just one class above the threshold - no matching needed..." << dkendl;
		return votingMat;
	}
	//else
	//	moutc << "[" << className << "] number of items to match: " << cv::sum(votingMat > thresh)[0]/255.0f << dkendl;

	for (int idx = 0; idx < votingMat.cols; idx++) {

		DkSlidingWindow tw = findTemplateWin(idx);
		if (vPtr[idx] > thresh && !tw.isEmpty()) {
			mPtr[idx] = singleMatch(win, tw);
			maskPtr[idx] = 1;
		}
	}

	cv::normalize(matchMat, matchMat, 1.0f, NORM_MINMAX);
	matchMat = 1.0f-matchMat;
	matchMat = matchMat.mul(matchMask);
	
	return matchMat;
}

DkSlidingWindow DkTemplateMatcher::findTemplateWin(int idx) const {

	DkSlidingWindow templateWin;

	for (DkSlidingWindow sw : templateWindows) {
		
		if (sw.getGtLabel() == idx) {
			templateWin = sw;
			break;
		}
	}

	if (templateWin.isEmpty()) {
		woutc << "[" << className << "] could not find a template for: " << idx << dkendl;
	}

	return templateWin;
}

float DkTemplateMatcher::singleMatch(const DkSlidingWindow& win, const DkSlidingWindow& templateWin) const {

	Mat desc = win.getDescriptors();
	Mat tDesc = templateWin.getDescriptors();

	if (desc.empty() || tDesc.empty()) {
		woutc << "[" << className << "] cannot match empty descriptors!" << dkendl;
	}

	cv::BFMatcher matcher(cv::NORM_L2, true);
	
	std::vector<cv::DMatch> matches;
	matcher.match(desc, tDesc, matches);

	std::sort(matches.begin(), matches.end(), &dkCompDistKeyPt);

	std::vector<cv::KeyPoint> kpts = win.getKeyPoints();
	std::vector<cv::KeyPoint> tKpts = templateWin.getKeyPoints();
	std::vector<cv::Point2f> mKpts;
	std::vector<cv::Point2f> mtKpts;
	float distEnergy = 0;

	for (int idx = 0; idx < cvRound(matches.size()*0.5); idx++) {

		int qIdx = matches[idx].queryIdx;
		int tIdx = matches[idx].trainIdx;
		bool duplicate = false;
		cv::Point2f kp = tKpts[matches[idx].trainIdx].pt;

		for (int dIdx = 0; dIdx < matches.size(); dIdx++) {

			if (dIdx == idx)
				continue;

			if (kp == tKpts[matches[dIdx].trainIdx].pt) {
				duplicate = true;
				break;
			}
		}

		if (!duplicate) {
			mKpts.push_back(kpts[matches[idx].queryIdx].pt);
			mtKpts.push_back(tKpts[matches[idx].trainIdx].pt);
			distEnergy += matches[idx].distance;
			//moutc << "dist: " << matches[idx].distance << dkendl;
		}
	}

	distEnergy = distEnergy/(float)mKpts.size();
	return distEnergy;

	if (mKpts.empty()) {
		woutc << "[" << className << "] no matches found for template " << templateWin.getGtLabel() << dkendl;
		return 0.0f;
	}

	moutc << "[" << className << "] number of potential matches: " << mKpts.size() << dkendl;

	if (releaseDebug == DK_SAVE_IMGS) {
		Mat img1(1080, 1920, CV_32FC1, Scalar(100, 100, 100));
		Mat img2(1080, 1920, CV_32FC1, Scalar(200, 200, 200));
		Mat resImg;
		cv::drawMatches(img1, kpts, img2, tKpts, matches, resImg);
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", resImg, true);
	}

	//return (float)mKpts.size();

	Mat mask;
	Mat hom = cv::findHomography(mKpts, mtKpts, CV_RANSAC, 15.0, mask);
	mask = mask.t();	// row vector is nicer
	float numMatches = 0;
	std::vector<cv::KeyPoint> fKpts;
	std::vector<cv::KeyPoint> ftKpts;
	std::vector<cv::DMatch> matchesH;

	unsigned char* mPtr = mask.ptr<unsigned char>();
	for (int idx = 0; idx < mask.cols; idx++) {

		if (mPtr[idx]) {
			numMatches++;
			matchesH.push_back(matches[idx]);
		}
	}

	if (releaseDebug == DK_SAVE_IMGS) {
		Mat img1(1080, 1920, CV_32FC1, Scalar(0.5, 0.5, 0.5));
		Mat img2(1080, 1920, CV_32FC1, Scalar(0.75, 0.75, 0.75));
		Mat resImg;
		cv::drawMatches(img1, kpts, img2, tKpts, matchesH, resImg);
		DkIP::imwrite(className + DkUtils::stringify(__LINE__) + ".png", resImg, true);
	}

	moutc << "[" << className << "] number of matches [" << templateWin.getGtLabel() << "]: " << numMatches << dkendl;

	return numMatches;
}

bool DkTemplateMatcher::write(const std::string& filePath, const std::string& filename, const std::vector<DkSlidingWindow>& slidingWindows) {

	std::string featureFile = filePath + filename;

	FileStorage fs(featureFile, FileStorage::WRITE);

	if (!fs.isOpened()) {
		std::cout << "Sorry, I cannot write to: " << featureFile << std::endl;
		return false;
	}

	Mat features;
	Mat labels;
	std::vector<KeyPoint> keypoints;

	// convert features (append all)
	for (DkSlidingWindow sw : slidingWindows) {

		Mat desc = sw.getDescriptors();
		Mat label(desc.rows, 1, CV_16U, Scalar(sw.getGtLabel()));
		std::vector<KeyPoint> kp = sw.getKeyPoints();

		features.push_back(desc);
		keypoints.insert(keypoints.end(), kp.begin(), kp.end());
		labels.push_back(label);

	}

	fs << "keypoints" << keypoints;
	fs << "features" << features;
	fs << "labels" << labels.t();	// row vector is easier to read

	fs.release();

	std::cout << "writing to: " << featureFile << std::endl;

	return true;
}

bool DkTemplateMatcher::load(const std::string& filePath, const std::string& filename, std::vector<DkSlidingWindow>& slidingWindows) {

	std::string featureFile = filePath + filename;

	FileStorage fs(featureFile, FileStorage::READ);

	if (!fs.isOpened()) {
		moutc << "Sorry, I cannot read features from: " << featureFile << dkendl;
		return false;
	}

	Mat features;
	Mat labels;
	std::vector<KeyPoint> keypoints;

	fs["features"] >> features;
	fs["labels"] >> labels;

	FileNode kpNode = fs["keypoints"];
	cv::read(kpNode, keypoints);

	double maxLabel = 0;
	cv::minMaxLoc(labels, 0, &maxLabel);

	if (!maxLabel) {
		moutc << "Max label is zero. Sorry, I cannot read features from: " << featureFile << dkendl;
		return false;
	}

	for (int idx = 0; idx <= cvRound(maxLabel); idx++) {

		Mat cLabels = labels == idx;

		const unsigned char* lIdx = cLabels.ptr<unsigned char>();

		Mat cFeatures;
		std::vector<KeyPoint> cKeypoints;

		for (int rIdx = 0; rIdx < cLabels.cols; rIdx++) {

			if (lIdx[rIdx]) {
				cKeypoints.push_back(keypoints[rIdx]);
				cFeatures.push_back(features.row(rIdx));
			}
		}

		if (!features.empty()) {
			DkSlidingWindow sw;
			sw.setGtLabel(idx);
			sw.setKeypoints(cKeypoints);
			sw.setDescriptors(cFeatures);
			slidingWindows.push_back(sw);
		}
		else
			woutc << "sorry, I could not read any feature for label: " << idx << dkendl;
	}

	fs.release();
	moutc << slidingWindows.size() << " features read from: " << featureFile << dkendl;

	return true;
}

std::vector<DkSlidingWindow> DkTemplateMatcher::getSlidingWindows() const {

	return windows;
}

std::string DkTemplateMatcher::toString() const {

	std::string str;
	str += className;

	return str;
}