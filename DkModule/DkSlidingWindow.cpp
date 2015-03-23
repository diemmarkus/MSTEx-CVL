/**************************************************
 * 	DkSlidingWindow.cpp
 *
 *	Created on:	01.06.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkSlidingWindow.h"
#include "DkObjectRecognition.h"
#include "DkFeatureExtraction.h"

#include "opencv2/video/tracking.hpp"


std::map<int, std::string> DkLabelManager::labelLookup = std::map<int, std::string>();
std::vector<std::string> DkLabelManager::activeList = std::vector<std::string>();

// DkPolyRect --------------------------------------------------------------------
DkPolyRect::DkPolyRect(const std::vector<cv::Point>& pts) {

	toDkVectors(pts, this->pts);
	computeMaxCosine();
	area = DBL_MAX;
}

DkPolyRect::DkPolyRect(const std::vector<DkVector>& pts) {
	this->pts = pts;
	computeMaxCosine();
	area = DBL_MAX;
}

bool DkPolyRect::empty() const {
	return pts.empty();
}

void DkPolyRect::toDkVectors(const std::vector<cv::Point>& pts, std::vector<DkVector>& dkPts) const {

	for (int idx = 0; idx < pts.size(); idx++)
		dkPts.push_back(DkVector(pts.at(idx)));
}

void DkPolyRect::computeMaxCosine() {

	maxCosine = 0;

	for (int idx = 2; idx < pts.size()+2; idx++ ) {

		DkVector& c = pts[(idx-1)%pts.size()];	// current corner;
		DkVector& c1 = pts[idx%pts.size()];
		DkVector& c2 = pts[idx-2];

		double cosine = abs((c1-c).cosv(c2-c));

		maxCosine = max(maxCosine, cosine);
	}
}

double DkPolyRect::intersectArea(const DkPolyRect& pr) const {

	return DkIntersectPoly(pts, pr.pts).compute();
}

void DkPolyRect::scale(float s) {

	for (size_t idx = 0; idx < pts.size(); idx++)
		pts[idx] = pts[idx]*s;

	area = DBL_MAX;	// update
}

void DkPolyRect::scaleCenter(float s) {

	DkVector c = center();

	for (size_t idx = 0; idx < pts.size(); idx++) {
		pts[idx] = DkVector(pts[idx]-c)*s+c;
	}

	area = DBL_MAX;	// update
}

DkVector DkPolyRect::center() const {

	DkVector c;

	for (size_t idx = 0; idx < pts.size(); idx++)
		c += pts[idx];

	c /= (float)pts.size();

	return c;
}


bool DkPolyRect::inside(const DkVector& vec) const {

	float lastsign = 0;

	// we assume, that the polygon is convex
	// so if the point has a different scalar product
	// for one side of the polygon - it is not inside
	for (size_t idx = 1; idx < pts.size()+1; idx++) {

		DkVector dv(pts[idx-1] - pts[idx%pts.size()]);
		float csign = dv.scalarProduct(vec - pts[idx%pts.size()]);

		if (lastsign*csign < 0) {
			return false;
		}

		lastsign = csign;
	}

	return true;
}

float DkPolyRect::maxSide() const {

	float ms = 0;

	for (size_t idx = 1; idx < pts.size()+1; idx++) {
		float cs = DkVector(pts[idx-1] - pts[idx%pts.size()]).norm();

		if (ms < cs)
			ms = cs;
	}

	return ms;
}

double DkPolyRect::getArea() {

	if (area == DBL_MAX)
		area = abs(intersectArea(*this));

	return area;
}

double DkPolyRect::getAreaConst() const {

	if (area == DBL_MAX)
		return abs(intersectArea(*this));

	return area;
}

bool DkPolyRect::compArea(const DkPolyRect& pl, const DkPolyRect& pr) {

	return pl.getAreaConst() < pr.getAreaConst();
}

void DkPolyRect::draw(Mat& img, const cv::Scalar& col) const {

	std::vector<cv::Point> cvPts = toCvPoints();
	if (cvPts.empty())
		return;

	const cv::Point* p = &cvPts[0];
	int n = (int)cvPts.size();
	polylines(img, &p, &n, 1, true, col, 3, CV_AA);
}

std::vector<DkVector> DkPolyRect::getCorners() const {
	return pts;
}

DkBox DkPolyRect::getBBox() const {

	DkVector uc(FLT_MAX, FLT_MAX), lc(-FLT_MAX, -FLT_MAX);

	for (size_t idx = 0; idx < pts.size(); idx++) {
		
		uc.minVec(pts[idx]);
		lc.maxVec(pts[idx]);
	}

	DkBox box(uc, lc-uc);

	return box;
}

std::vector<cv::Point> DkPolyRect::toCvPoints() const {

	std::vector<cv::Point> cvPts;
	for (int idx = 0; idx < pts.size(); idx++) {
		cvPts.push_back(pts[idx].getCvPoint());
	}

	return cvPts;
}

// DkCircle --------------------------------------------------------------------
DkCircle::DkCircle(const DkVector& center /* = DkVector */, float radius /* = 0.0f*/) {
	this->center = center;
	this->radius = radius;
}

DkCircle::DkCircle(const cv::Vec3f& circle) {
	this->center.x = circle[0];
	this->center.y = circle[1];
	this->radius = circle[2];
}

DkCircle::DkCircle(const cv::KeyPoint& kpt) {

	this->center.x = kpt.pt.x;
	this->center.y = kpt.pt.y;
	this->radius = kpt.size;
}

bool DkCircle::isEmpty() const {

	return center.isEmpty() && !radius;
}

bool DkCircle::inside(const DkVector& vec) const {

	return center.euclideanDistance(vec) < radius;
}

void DkCircle::scale(float s) {

	center *= s;
	radius *= s;
}

void DkCircle::scaleCenter(float s) {

	radius *= s;
}

DkVector DkCircle::getCenter() const {
	
	return center;
}

float DkCircle::getRadius() const {

	return radius;
}

DkBox DkCircle::getBBox() const {

	DkBox b(center.x - radius, center.y - radius, 2*radius, 2*radius);
	return b;
}

float DkCircle::getArea() const {

	return (float)(radius*radius*CV_PI);
}

std::vector<DkVector> DkCircle::toPoly() const {

	int numPts = 30;
	double angleStep = 2*CV_PI/numPts;
	std::vector<DkVector> pts;

	for (int idx = 0; idx < numPts; idx++) {

		DkVector v(0, radius);
		v.rotate(angleStep*idx);
		v += center;

		pts.push_back(v);
	}

	return pts;
}

void DkCircle::draw(Mat& img, const cv::Scalar& col) const {

	cv::circle(img, getCenter().getCvPoint(), 3, col, -1, 8, 0);
	cv::circle(img, getCenter().getCvPoint(), (int)getRadius(), col, 3, 8, 0);
}

// DkSlidingWindow --------------------------------------------------------------------
DkSlidingWindow::DkSlidingWindow() {
	removeLarge = true;
	minFeatures = 5;
	minSize = 10;
	label = -1;
	gtLabel = -1;
	rescaleArea = 1.0f;
}

DkSlidingWindow::~DkSlidingWindow() {}

void DkSlidingWindow::getStorageBuffer(char** buffer, size_t& length) const {

	length = 0;
	*buffer = 0;
	
	window.getStorageBuffer(buffer, length);
	DkIP::appendMatToBuffer(buffer, length, poly2Mat(polyPts));
	DkIP::appendMatToBuffer(buffer, length, classResponse);
	DkIP::appendMatToBuffer(buffer, length, bowDescriptor);
	DkIP::appendMatToBuffer(buffer, length, hist);
}

void DkSlidingWindow::setStorageBuffer(const char* buffer) {

	Mat polyMat, cr;
	const char* ptr = 0;
	ptr = window.setSorageBuffer(buffer);
	ptr = DkIP::getMatFromBuffer(ptr, polyMat);
	ptr = DkIP::getMatFromBuffer(ptr, cr);
	ptr = DkIP::getMatFromBuffer(ptr, bowDescriptor);
	ptr = DkIP::getMatFromBuffer(ptr, hist);

	setClassResponse(cr);
	polyPts = mat2Poly(polyMat);
}

Mat DkSlidingWindow::poly2Mat(const std::vector<DkVector>& poly) const {

	if (poly.empty())
		return Mat();

	Mat m(2, (int)poly.size(), CV_32FC1);

	float* xPtr = m.ptr<float>(0);
	float* yPtr = m.ptr<float>(1);

	for (int cIdx = 0; cIdx < poly.size(); cIdx++) {
		xPtr[cIdx] = poly[cIdx].x;
		yPtr[cIdx] = poly[cIdx].y;
	}

	return m;
}

std::vector<DkVector> DkSlidingWindow::mat2Poly(const Mat& mat) const {

	std::vector<DkVector> p;

	if (mat.empty())
		return p;

	const float* xPtr = mat.ptr<const float>(0);
	const float* yPtr = mat.ptr<const float>(1);

	for (int cIdx = 0; cIdx < mat.cols; cIdx++) {
		p.push_back(DkVector(xPtr[cIdx], yPtr[cIdx]));
	}

	return p;
}

void DkSlidingWindow::setInputParams(const DkInputParameter& params) {

	rescaleArea = params.rescaleArea;
}

bool DkSlidingWindow::collectFeatures(const std::vector<KeyPoint>& kps, const Mat& desc) {

	if (window.size().isEmpty()) {
		wout << "[DkSlidingWindow] you did not assign a window - so I cannot assign descriptors" << dkendl;
		return false;
	}

	DkBox win = window;

	if (rescaleArea != 1.0f)
		win.scaleAboutCenter(rescaleArea);

	for (size_t idx = 0; idx < kps.size(); idx++) {

		if (win.within(kps[idx].pt) /*&& (!removeLarge || kps[idx].size < window.size().width) || kps[idx].size > minSize*/) {
			keypoints.push_back(kps[idx]);
			descriptorIdx.push_back((int)idx);
		}
	}

	if (keypoints.empty() || keypoints.size() < (size_t)minFeatures)
		return false;

	// collect features
	descriptors = Mat((int)descriptorIdx.size(), desc.cols, desc.depth());

	for (size_t idx = 0; idx < descriptorIdx.size(); idx++) {
		desc.row(descriptorIdx[idx]).copyTo(descriptors.row((int)idx));
	}

	//mout << "[DkSlidingWindow] I have collected " <<  kpidx.size() << " features..." << dkendl;

	return true;
}

void DkSlidingWindow::computeBoW() {

	DkTimer dt;
	DkBoW bow;
	bowDescriptor = bow.compute(descriptors, keypoints);

	iout << "BoW computed in: " << dt << " for " << descriptors.size() << " descriptors" << dkendl;
}

void DkSlidingWindow::clearDescriptors() {

	keypoints.clear();
	descriptors.release();
}

int DkSlidingWindow::getLabelResult() const {

	if (label == gtLabel && gtLabel != -1 && label != -1)
		return DK_TRUE_POSITIVE;
	else if (label != gtLabel && gtLabel != -1 && label != -1)
		return DK_FALSE_POSITIVE;

	return DK_GT_UNKNOWN;
}

bool DkSlidingWindow::isEmpty() const {

	return keypoints.empty() || descriptors.empty();
}

bool DkSlidingWindow::isBowEmpty() const {

	return bowDescriptor.empty();
}

void DkSlidingWindow::setWindow(const DkBox& window) {
	this->window = window;
}

DkBox DkSlidingWindow::getWindow() const {
	return window;
}

void DkSlidingWindow::setKeypoints(const std::vector<KeyPoint>& keypoints) {
	this->keypoints = keypoints;
}

std::vector<KeyPoint> DkSlidingWindow::getKeyPoints() const {
	return keypoints;
}

void DkSlidingWindow::setDescriptors(const Mat& descriptors) {
	this->descriptors = descriptors;
}

Mat DkSlidingWindow::getDescriptors() const {
	return descriptors;
}

void DkSlidingWindow::setBoWDescriptor(const Mat& bowDescriptor) {
	this->bowDescriptor = bowDescriptor.clone();
}

Mat DkSlidingWindow::getBoWDescriptor() const {
	return bowDescriptor;
}

void DkSlidingWindow::setLabel(int label) {
	this->label = label;
}

int DkSlidingWindow::getLabel() const {
	return label;
}

void DkSlidingWindow::setGtLabel(int label) {
	this->gtLabel = label;
}

int DkSlidingWindow::getGtLabel() const {
	return gtLabel;
}

void DkSlidingWindow::setClassResponse(const Mat& classResponse) {

	this->classResponse = classResponse;

	int maxIdx = -1;
	float maxVal = 0;
	const float* clPtr = classResponse.ptr<float>();

	for (int idx = 0; idx < classResponse.cols; idx++) {

		if (clPtr[idx] > maxVal) {
			maxIdx = idx;
			maxVal = clPtr[idx];
		}
	}
	
	//minMaxIdx(classResponse, 0, 0, 0, &maxIdx);		// has a memory leak if called like this!
	setLabel(maxIdx);
}

Mat DkSlidingWindow::getClassResponse() const {
	return classResponse;
}

float DkSlidingWindow::getReliability() const {

	if (getProbability() == 0)
		return 0;

	float val = (float)(getProbability() - DkIP::statMomentMat(classResponse)) * 2.0f;

	return (val > 1.0f) ? 1.0f : val;	// crop at 1 (it's more intuitive)
}

float DkSlidingWindow::getProbability() const {

	if (getLabel() < 0 || getLabel() >= classResponse.cols)
		return 0;

	const float* crPtr = classResponse.ptr<const float>();
	
	return crPtr[getLabel()];
}

void DkSlidingWindow::setHist(const Mat& hist) {
	this->hist = hist;
}

Mat DkSlidingWindow::getHist() const {
	return hist;
}

void DkSlidingWindow::setPolyRect(const DkPolyRect& polyRect) {

	polyPts = polyRect.getCorners();
	window = polyRect.getBBox();
}

DkPolyRect DkSlidingWindow::getPolyRect() const {

	return DkPolyRect(polyPts);
}

void DkSlidingWindow::setCircle(const DkCircle& circle) {

	this->circle = circle;
	window = circle.getBBox();
}

DkCircle DkSlidingWindow::getCircle() const {

	return circle;
}

void DkSlidingWindow::calcHist(const Mat& hueImg) {

	if (!hist.empty())
		return;

	DkBox w = window;
	w.setSize(w.size()*0.5f);
	Mat roi(hueImg, window.getCvRect());

	int hsize = 16;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	cv::calcHist(&roi, 1, 0, Mat(), hist, 1, &hsize, &phranges);
	normalize(hist, hist, 0, 255, CV_MINMAX);

}

void DkSlidingWindow::track(const Mat& img) {
	
	Mat bpImg;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	
	calcBackProject(&img, 1, 0, hist, bpImg, &phranges);

	DkIP::imwrite("bpImg.png", bpImg, true);

	//backproj &= mask;
	cv::Rect rect = window.getCvRect();
	CamShift(bpImg, rect,
		TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	// update window
	DkBox r = rect;
	window.moveBy(r.center()-window.center());
}

void DkSlidingWindow::filterKeypoints() {

	if (!polyPts.empty()) {
		filterKeypoints(polyPts);
	}
	if (!circle.isEmpty())
		filterKeypoints(circle);
}

void DkSlidingWindow::filterKeypoints(const DkPolyRect& poly) {

	DkPolyRect p = poly;
	
	if (poly.empty()) {
		wout << "WARNING: filter called with empty polygon" << dkendl;
		return;
	}

	if (rescaleArea != 1.0f)
		p.scaleCenter(rescaleArea);

	std::vector<int> rowIdxs;
	std::vector<KeyPoint> kpts;

	for (size_t idx = 0; idx < keypoints.size(); idx++) {

		if (p.inside(keypoints[idx].pt)) {
			kpts.push_back(keypoints[idx]);
			rowIdxs.push_back((int)idx);
		}
	}

	keypoints = kpts;

	DkFeatureExtraction fe;
	fe.filterDescriptors(rowIdxs, descriptors);

}

void DkSlidingWindow::filterKeypoints(const DkCircle& circle) {

	if (circle.isEmpty()) {
		wout << "WARNING: filter called with empty circle" << dkendl;
		return;
	}

	DkCircle circ = circle;

	if (rescaleArea != 1.0f)
		circ.scaleCenter(rescaleArea);

	std::vector<int> rowIdxs;
	std::vector<KeyPoint> kpts;

	for (size_t idx = 0; idx < keypoints.size(); idx++) {

		if (circ.inside(keypoints[idx].pt)) {
			kpts.push_back(keypoints[idx]);
			rowIdxs.push_back((int)idx);
		}
	}

	keypoints = kpts;

	DkFeatureExtraction fe;
	fe.filterDescriptors(rowIdxs, descriptors);

}

void DkSlidingWindow::draw(Mat& img, cv::Scalar col) const {

	double op = 0.7;
	DkLabelManager m;

	bool customColor = col != Scalar(0,0,0);

	if (!customColor) {
		if (getLabelResult() == DkSlidingWindow::DK_FALSE_POSITIVE) {
			col = Scalar(200, 0, 0);
		}
		else if (getLabelResult() == DkSlidingWindow::DK_TRUE_POSITIVE) {
			col = Scalar(0, 200, 0);
		}
		else if (m.getLabel(getLabel()) == "background") {
			col = Scalar(200, 200, 200);
		}
		else if (getLabel() == -1) {
			col = Scalar(200, 200, 200);
		}
		else {
			op = 0.7;
			//col = Scalar(50,50,100);
			col = DkUtils::blue;
			//	return img;
		}
	}

	if (!polyPts.empty()) {

		DkPolyRect p(polyPts);
		p.draw(img, col);
	}

	if (!circle.isEmpty()) {
		circle.draw(img, col);
	}

	Mat res = img.clone();

	rectangle(res, getWindow().uc.getCvPoint(), getWindow().lc.getCvPoint(), col, CV_FILLED, CV_AA);

	// if you want to draw the bow's for each word
	DkBox histWin = getWindow();
	histWin.uc += histWin.size().width*0.3f;
	histWin.lc -= histWin.size().width*0.3f;
	DkBoW::draw(res, bowDescriptor, histWin, cv::Scalar(0,0,0));		// TODO: make it static : )

	// if you want to draw the bow's for each word
	DkBox classesWin = getWindow();
	classesWin.uc += classesWin.getHeight()*0.1f;
	classesWin.lc -= histWin.size().width*0.6f;
	classesWin.setSize(DkVector(getClassResponse().cols*4.0f, classesWin.size().height*0.3f));
	DkIP::drawHistToImage(img, getClassResponse(), classesWin, cv::Scalar(255,255,255));

	if (op != 1.0) {
		
		DkBox roi = getWindow();
		roi.uc.floor();
		roi.lc.ceil();
		roi.clip(img.size());
		Mat imgRoi = img(roi.getCvRect());
		Mat resRoi = res(roi.getCvRect());

		op *= getProbability();
		addWeighted(imgRoi, 1.0-op, resRoi, op, 1.0, imgRoi);
	}

	rectangle(img, getWindow().uc.getCvPoint(), getWindow().lc.getCvPoint(), col, 2, CV_AA);

	// show label
	if (getLabel() != -1) {
		DkVector pos = getWindow().uc - 1;
		std::string msg = m.getLabel(getLabel());
		msg += " [" + DkUtils::stringify(getProbability(), 2); 
		msg += "|" + DkUtils::stringify(getReliability(), 2) + "]";
		cv::putText(img, msg, pos.getCvPoint(), FONT_HERSHEY_COMPLEX, 0.5, col);
	}



	//cv::putText(img, m.getLabel(getLabel()), getWindow().uc.getCvPoint(), FONT_HERSHEY_COMPLEX, 0.35, Scalar(0));
	//std::string str =  "label: " + m.getLabel(getLabel()) + " gt: " + m.getLabel(getGtLabel());
	//std::string str =  "label: " + DkUtils::stringify(getLabel()) + " gt: " + m.getLabel(getGtLabel());
	//std::string str =  m.getLabel(getLabel()) + " " + DkUtils::stringify(getProbability()*100, 1) + "%";
	//cv::putText(img, str, getWindow().uc.getCvPoint(), FONT_HERSHEY_COMPLEX, 0.35, Scalar(0));
	//cv::putText(img, DkUtils::stringify(keypoints.size()), getWindow().uc.getCvPoint(), FONT_HERSHEY_COMPLEX, 0.35, Scalar(0));

}

Mat& operator<<(Mat& img, const DkSlidingWindow& win) {

	win.draw(img);

	return img;
};



// DkLookupManager --------------------------------------------------------------------
DkLabelManager::DkLabelManager(const std::string& filename /* = "unknown" */, int labelFormat) {

	if (labelFormat == gt_label)
		this->labelName = filename;
	else if (labelFormat == gt_filename)
		this->labelName = convertToLabel(filename);
	else if (labelFormat == gt_path)
		this->labelName = convertPathToLabel(filename);
}

void DkLabelManager::labelWindows(std::vector<DkSlidingWindow>& slidingWindows, const std::string& filename /* = "" */) {

	int label = getLabelIdx(filename.empty() ? labelName : filename);
	
	for (size_t idx = 0; idx < slidingWindows.size(); idx++) {
		slidingWindows[idx].setGtLabel(label);
	}
}

void DkLabelManager::labelWindows(std::vector<DkProductInfo>& slidingWindows, const std::string& filename /* = "" */) {

	int label = getLabelIdx(filename.empty() ? labelName : filename);

	for (size_t idx = 0; idx < slidingWindows.size(); idx++) {
		slidingWindows[idx].getSlidingWindow().setGtLabel(label);
	}
}

int DkLabelManager::addLabel(bool* labelAdded) {

	return addLabel(labelName, labelAdded);
}

int DkLabelManager::addLabel(const std::string& label, bool* labelAdded) {

	int lIdx = getLabelIdx(label);

	if (labelAdded)
		*labelAdded = false;

	// could we find the label?
	if (lIdx == -1 && !label.empty()) {

		lIdx = (int)labelLookup.size();
		labelLookup.insert(std::pair<int,std::string>(lIdx, label));
		if (labelAdded)
			*labelAdded = true;
	}
	else if (label.empty()) {
		wout << "[DkLabelManager] sorry, but I will not add an empty label - that's dangerous..." << dkendl;
	}

	return lIdx;
}

int DkLabelManager::getLabelIdx(const std::string& label) const {

	int idx = 0;

	char illegal[] = "-_0123456789";

	// puh... this renders class1 class2 ambigious -> shouldn't we just search for - and then remove the rest of the string
	for (; (size_t)idx < labelLookup.size(); idx++) {

		std::string cLabel = labelLookup.at(idx);

		if (cLabel == label)
			break;

		std::string cLabelClean = cLabel;
		std::string labelClean = label;

		for (unsigned int cIdx = 0; cIdx < strlen(illegal); cIdx++) {

			cLabelClean.erase(std::remove(cLabelClean.begin(), cLabelClean.end(), illegal[cIdx]), cLabelClean.end());
			labelClean.erase(std::remove(labelClean.begin(), labelClean.end(), illegal[cIdx]), labelClean.end());
		}

		if (cLabelClean == labelClean)
			break;

	}

	if (idx == labelLookup.size())
		return -1;

	return idx;
}

int DkLabelManager::getGtLabelIdx() const {
	return getLabelIdx(labelName);
}

std::string DkLabelManager::getLabel(int idx) const {

	if (idx == -1)
		return "unknown";
	if (idx < 0 || (size_t)idx >= labelLookup.size()) {
		wout << "Index (" << idx << ") out of bounds in DkLabelManager::getLabel" << dkendl;
		return "unknown";
	}

	return labelLookup.find(idx)->second;
}

std::string DkLabelManager::convertToLabel(const std::string& str) const {

	return str.substr(0, str.find_last_of("-"));
}

std::string DkLabelManager::convertPathToLabel(const std::string& str) const {

	std::string label = "unknown";
	std::string strl = str;
	std::transform(strl.begin(), strl.end(), strl.begin(), ::tolower);
	std::vector<std::string> labelsFound;

	for (int idx = 0; idx < labelLookup.size(); idx++) {
		std::string l = labelLookup.at(idx);
		std::transform(l.begin(), l.end(), l.begin(), ::tolower);

		if (strl.find(l) != string::npos)
			labelsFound.push_back(labelLookup.at(idx));
	}


	// find the longest match possible (eg. McNuggets vs McNuggets6er)
	int maxl = 0;
	std::string maxLabel;
	for (int idx = 0; idx < labelsFound.size(); idx++) {

		std::string cl = labelsFound.at(idx);

		if (cl.length() > maxl) {
			maxl = (int)cl.length();
			maxLabel = cl;
		}
	}

	if (!maxLabel.empty()) {
		label = maxLabel;
		mout << "label found: " << maxLabel << " in: " << str << dkendl;
	}

	if (label == "unknown")
		wout << "[WARNING]: could not find GT label in " << str << dkendl;

	return label;
}

std::map<int, std::string> DkLabelManager::getAllLabels() const {
	return labelLookup;
}

int DkLabelManager::getNumClasses() const {
	return (int)labelLookup.size();
}

int DkLabelManager::getNumActiveClasses() const {
	
	if (activeList.size())
		return (int)activeList.size();
	else
		return getNumClasses();
}

void DkLabelManager::clearActiveList() {

	activeList.clear();
}

void DkLabelManager::addActiveItem(const std::string& item) {

	activeList.push_back(item);
}

void DkLabelManager::setActiveClassList(const std::vector<std::string>& aList) {

	activeList = aList;
}

std::vector<std::string> DkLabelManager::getActiveClassList() {

	return activeList;
}

std::vector<int> DkLabelManager::getActiveIdxList() const {

	std::vector<int> activeListIdx;

	for (int idx = 0; idx < activeList.size(); idx++) {

		int cIdx = getLabelIdx(activeList.at(idx));

		if (cIdx != -1)
			activeListIdx.push_back(cIdx);
		else
			wout << "could not find valid label for entry: " << activeList.at(idx) << " in the ingore list" << dkendl;
	}

	return activeListIdx;
}

bool DkLabelManager::acitveClass(int labelIdx) const {

	std::vector<int> aList = getActiveIdxList();

	if (aList.size() < 1)
		return true;

	return std::find(aList.begin(), aList.end(), labelIdx) != aList.end();
}


// init etc --------------------------------------------------------------------
bool DkLabelManager::write(const std::string& filePath, const std::string& filename) {

	if (labelLookup.empty()) {
		std::cout << "WARNING: I cannot write an empty lookup table..." << std::endl;
		return false;
	}

	std::string lookupFName = filePath + filename;
	std::cout << "writing to: " << lookupFName << std::endl;
	FileStorage fs(lookupFName, FileStorage::WRITE);

	if (!fs.isOpened()) {
		std::cout << "Sorry, I cannot write to: " << lookupFName << std::endl;
		return false;
	}

	fs << "lookupStrings" << "[";
	for (int idx = 0; (size_t)idx < labelLookup.size(); idx++) {
		fs << labelLookup.at(idx);
	}
	fs << "]";
	
	fs.release();

	return true;
}

bool DkLabelManager::loadLookup(const std::string& filePath, const std::string& filename) {

	std::string lookupFileName = filePath + filename;
	FileStorage fs(lookupFileName, FileStorage::READ);

	if (!fs.isOpened()) {
		mout << "Sorry, I could not read from: " << lookupFileName << dkendl;
		return false;
	}

	FileNode n = fs["lookupStrings"];                         // Read string sequence - Get node
	if (n.type() != FileNode::SEQ) {
		DebugResources::wout() << "cannot load lookup table, lookupStrings is not a sequence" << dkendl;
		return false;
	}

	DebugResources::mout() << "loading labels from: " << lookupFileName << dkendl;

	FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
	for (int idx = 0; it != it_end; it++, idx++) {
		labelLookup.insert(std::pair<int, std::string>(idx, (std::string)*it));
		DebugResources::mout() << (std::string)*it << ": " << idx << dkendl; 
	}

	DebugResources::mout() << dkendl;	// add empty line

	fs.release();

	return true;
}
