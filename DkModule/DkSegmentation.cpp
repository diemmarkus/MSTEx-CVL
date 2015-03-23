/**************************************************
 * 	DkSegmentation.cpp
 *
 *	Created on:	15.01.2015
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkSegmentation.h"
#include "DkObjectRecognition.h"

DkKMeansSegmentation::DkKMeansSegmentation(const Mat& colImg) {

	resize = 0.25;
	numObjects = 1;
	k = 3;

	className = "[DkKMeansSegmentation]";
	this->img = colImg;
}

void DkKMeansSegmentation::checkInput() const {

	if (img.empty()) {
		std::string msg = "The image img is empty.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
}

void DkKMeansSegmentation::compute() {

	checkInput();

	DkTimer dt;
	Mat colImg = img;

	// speed-up
	if (resize != 1.0f) {
		cv::resize(colImg, colImg, cv::Size(), resize, resize, CV_INTER_AREA);
	}
	
	cvtColor(colImg, colImg, CV_RGB2Lab);
	std::vector<cv::Mat> imgLAB;
	cv::split(colImg, imgLAB);

	int n = colImg.rows * colImg.cols;
	cv::Mat img3xN(n,2,CV_8U);
	
	// ignore the Luminance -> start with 1
	for(int idx = 1; idx < colImg.channels(); idx++)  
		imgLAB[idx].reshape(1,n).copyTo(img3xN.col(idx-1));

	//segImg = colImg;

	img3xN.convertTo(img3xN,CV_32F);
	cv::kmeans(img3xN, k, segImg, cv::TermCriteria(), 3, cv::KMEANS_RANDOM_CENTERS);
	
	segImg.convertTo(segImg, CV_8U);	// we have Max 4-5 labels so that's save
	segImg = segImg.reshape(0, colImg.rows);
	
	// find the background
	double mVal = 0;
	cv::minMaxLoc(segImg, 0, &mVal);

	double maxArea = 0;
	int mIdx = 0;
	
	for (int idx = 0; idx <= mVal; idx++) {

		double area = cv::sum(segImg == idx)[0];
		if (area > maxArea) {
			maxArea = area;
			mIdx = idx;
		}
	}

	segImg = segImg == mIdx;	// get the inverted background
	DkIP::invertImg(segImg);
	
	// fuse the kmeans result with a luminance thresholded image
	cv::Mat imgL;
	cv::threshold(imgLAB.at(0), imgL, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	segImg |= imgL;		// combine luminance result with color segmentation

	// remove artifacts
	int shape = MORPH_ELLIPSE;
	int msize = 5;
	Mat se = getStructuringElement(shape, Size(2*msize + 1, 2*msize+1 ), Point(msize, msize));

	// close (close holes)
	cv::dilate(segImg, segImg, se);
	cv::erode(segImg, segImg, se);

	// open (remove noise)
	cv::erode(segImg, segImg, se);
	cv::dilate(segImg, segImg, se);

	Mat hImg = DkIP::convexHull(segImg, bbox);

	// security check
	double s = cv::sum(hImg-segImg)[0]/255.0f;
	if (s > 100000.0f) {
		mout << "-----------------------------------------------------" << dkendl;
		mout << "WARNING: convex hull is way larger: " << s << dkendl;
	}
	else
		segImg = hImg;

	// speed-up
	if (resize != 1.0f) {
		float rinv = 1.0f/resize;
		cv::resize(segImg, segImg, cv::Size(), rinv, rinv, CV_INTER_AREA);
		bbox = cv::Rect(cvRound(bbox.x*rinv), cvRound(bbox.y*rinv), cvRound(bbox.width*rinv), cvRound(bbox.height*rinv));
	}

	mout << className << " computed in: " << dt << dkendl;
}

Mat DkKMeansSegmentation::getSegImg() {

	if (segImg.empty())
		compute();

	return segImg;
}

cv::Rect DkKMeansSegmentation::getBBox() const {

	return bbox;
}

Mat DkKMeansSegmentation::getDebugImg() const {

	return dbImg;
}

std::string DkKMeansSegmentation::toString() const {

	return className + " no info here";
}

// DkSegmentBurger --------------------------------------------------------------------
// This code is based on OpenCV's rectangle sample (squares.cpp)
DkBurgerSegmentation::DkBurgerSegmentation(const Mat& colImg /* = Mat */) {
	
	this->img = colImg;
	thresh = 80;
	numThresh = 10;
	
	DkInputParameter p;
	setInputParams(p);
	maxSideFactor = 0.75f;
	scale = 1.0f;
	detectCircles = true;
	looseDetection = true;

	className = "DkBurgerSegmentation";
}

void DkBurgerSegmentation::checkInput() const {

	if (img.empty()) {
		std::string msg = "The image img is empty.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
}

void DkBurgerSegmentation::releaseImages() {

	if (releaseDebug == DK_RELEASE_IMGS) {
		img.release();
		dbgImg.release();
	}
}

void DkBurgerSegmentation::setInputParams(const DkInputParameter& params) {

	minArea = params.detectorMinArea;
	maxArea = params.detectorMaxArea;
	maxSide = (float)params.detectorMaxSide;
	detectCircles = params.detectCircles;
}

Mat DkBurgerSegmentation::getDebugImg() const {

	return dbgImg;	// is NULL if releaseDebug is DK_RELEASE_IMGS
}

void DkBurgerSegmentation::compute() {

	checkInput();

	DkTimer dt;

	Mat imgLab;
	
	if (scale == 1.0f && 960.0f/img.cols < 0.8f)
		scale = 960.0f/img.cols;

	cv::cvtColor(img, imgLab, CV_RGB2Lab);	// boost colors
	Mat lImg = findRectangles(imgLab, rects);
	

	if (detectCircles) {
		findCircles(img, circles);
	}

	releaseImages();

	mout << "[" << className << "] " << rects.size() << " rectangles " << circles.size() << " circles found in: " << dt << " resize factor: " << scale << dkendl;
}

Mat DkBurgerSegmentation::findRectangles(const Mat& img, std::vector<DkPolyRect>& rects) const {

	Mat tImg, gray;


	if (scale != 1.0f)
		cv::resize(img, tImg, cv::Size(), scale, scale, CV_INTER_AREA);	// inter nn -> assuming resize to be 1/(2^n)
	else
		tImg = img;
	//}
	//else {
		//pyrDown(image, timg, Size(timg.cols/2, timg.rows/2));
		//pyrUp(timg, timg, Size(timg.cols*2, timg.rows*2));
	//	scaleF = 1.0f;
	//}

	//// down-scale and upscale the image to filter out the noise
	//pyrUp(timg, timg, Size(timg.cols*2, timg.rows*2));
	vector<vector<Point> > contours;

	Mat gray0(tImg.size(), CV_8UC1);
	Mat lImg(tImg.size(), CV_8UC1);

	// find squares in every color plane of the image
	for( int c = 0; c < 3; c++ ) {
		
		int ch[] = {c, 0};
		mixChannels(&tImg, 1, &gray0, 1, ch, 1);
		cv::normalize(gray0, gray0, 255, 0, NORM_MINMAX);

		if (c == 0)	// back-up the luminance channel - we use it as precomputed image for the circle detection
			lImg = gray0.clone();

		int nT = numThresh;//(c == 0) ? numThresh*2 : numThresh;	// more luminance thresholds

		// try several threshold levels
		for( int l = 0; l < nT; l++ ) {
			
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if( l == 0 ) {
				//pyrDown(gray0, gray, Size(gray.cols/2, gray.rows/2));
				//pyrDown(gray, gray, Size(gray.cols/2, gray.rows/2));
				//pyrUp(gray, gray, Size(gray.cols*2, gray.rows*2));
				//pyrUp(gray, gray, Size(gray.cols*2, gray.rows*2));

				Canny(gray0, gray, thresh, thresh*3, 5);
				// dilate canny output to remove potential
				// holes between edge segments
				dilate(gray, gray, Mat(), Point(-1,-1));

				//DkIP::imwrite("edgeImg.png", gray);
			}
			else {
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l+1)*255/numThresh;
				//Mat k = getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(2,2));
			}

			// find contours and store them all as a list
			findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

			if (looseDetection) {
				vector<vector<Point> > hull;
				for(int i = 0; i < contours.size(); i++) { 

					double cArea = contourArea(Mat(contours[i]));

					if (fabs(cArea) > minArea*scale*scale && (!maxArea || fabs(cArea) < maxArea*(scale*scale))) {
						vector<Point> cHull;
						cv::convexHull(Mat(contours[i]), cHull, false);
						hull.push_back(cHull);
					}
				}

				contours = hull;
			}

			vector<Point> approx;

			// DEBUG ------------------------
			//Mat pImg = image.clone();
			//cv::cvtColor(pImg, pImg, CV_Lab2RGB);
			// DEBUG ------------------------

			// test each contour
			for( size_t i = 0; i < contours.size(); i++ ) {
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

				double cArea = contourArea(Mat(approx));

				// DEBUG ------------------------
				//if (fabs(cArea) < maxArea)
				//	fillConvexPoly(pImg, &approx[0], (int)approx.size(), DkUtils::blue);
				// DEBUG ------------------------

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if( approx.size() == 4 &&
					fabs(cArea) > minArea*scale*scale &&
					(!maxArea || fabs(cArea) < maxArea*scale*scale) && 
					isContourConvex(Mat(approx)) ) {

					DkPolyRect cr(approx);
					//moutc << minArea*scale*scale << " < " << fabs(cArea) << " < " << maxArea*scale*scale << dkendl;

					// if cosines of all angles are small
					// (all angles are ~90 degree)
					if(cr.maxSide() < std::min(tImg.rows, tImg.cols)*maxSideFactor && 
						(!maxSide || cr.maxSide() < maxSide*scale) && 
						cr.getMaxCosine() < 0.3 ) {
						rects.push_back(cr);
					}
				}
			}
			// DEBUG ------------------------
			//cv::cvtColor(pImg, pImg, CV_RGB2BGR);
			//DkIP::imwrite("polyImg" + DkUtils::stringify(c) + "-" + DkUtils::stringify(l) + ".png", pImg);
			// DEBUG ------------------------
		}
	}

	for (size_t idx = 0; idx < rects.size(); idx++)
		rects[idx].scale(1.0f/scale);

	return lImg;
}

void DkBurgerSegmentation::findCircles(const Mat& img, std::vector<DkCircle>& circles) const {

	DkTimer dt;

	float scale = 1.0f;		// TODO: scale as input
	if (scale == 1.0f && 200.0f/img.cols < 0.8f)
		scale = 200.0f/img.cols;

	Mat rImg = img;
	if (scale != 1.0f)
		cv::resize(img, rImg, cv::Size(), scale, scale, CV_INTER_AREA);

	cv::SimpleBlobDetector::Params p;
	p.minArea = (float)minArea*scale*scale;
	p.maxArea = (float)maxArea*scale*scale;

	p.filterByColor = true;
	p.blobColor = 255;
	p.filterByCircularity = false;

	std::vector<KeyPoint> kpts;
	cv::SimpleBlobDetector detector(p);
	detector.detect(rImg, kpts);

	for (size_t idx = 0; idx < kpts.size(); idx++) {
		DkCircle c(kpts[idx]);
		c.scale(1.0f/scale);
		circles.push_back(c);
	}

	moutc << "[" << className << "] " << circles.size() << " circles detected in: " << dt << dkendl;
}

void DkBurgerSegmentation::filterDuplicates(float overlap, float areaRatio) {

	filterDuplicates(rects, overlap, areaRatio);
	filterDuplicatedCircles(circles, rects, overlap*0.5f);	// *0.5: circles should be removed if only a bit is overlapping
}

void DkBurgerSegmentation::filterDuplicates(std::vector<DkPolyRect>& rects, float overlap, float areaRatio) const {

	std::vector<int> delIdx;
	
	std::sort(rects.rbegin(), rects.rend(), &DkPolyRect::compArea);	// rbegin() -> sort descending

	for (int idx = 0; idx < rects.size(); idx++) {

		// if we already deleted a rectangle, we can safely skip it
		if (std::find(delIdx.begin(), delIdx.end(), idx) != delIdx.end())
			continue;

		DkPolyRect& cR = rects[idx];
		double cA = cR.getArea();

		std::vector<int> tmpDelIdx;

		for (int oIdx = idx+1; oIdx < rects.size(); oIdx++) {

			// if we already deleted a rectangle, we can safely skip it
			if (idx == oIdx || std::find(delIdx.begin(), delIdx.end(), oIdx) != delIdx.end())
				continue;

			DkPolyRect& oR = rects[oIdx];
			double oA = oR.getArea();

			// ignore rectangles with totally different area
			if (cA/oA < areaRatio)	// since we sort, we know that oA is larger
				continue;

			double intersection = abs(oR.intersectArea(cR));

			if (max(intersection/cR.getArea(), intersection/oR.getArea()) > overlap) {

				double cVal, oVal;

				// if the cosine is more or less the same, take the larger rectangle
				//if (fabs(cR.getMaxCosine() - oR.getMaxCosine()) < 0.02) {
				//	cVal = oA;	// inverse since we want to prefer larger areas
				//	oVal = cA;
				//}
				//else {
					cVal = cR.getMaxCosine();
					oVal = oR.getMaxCosine();
				//}

				// delete the rect which has an inferior cosine value
				if (cVal > oVal) {
					delIdx.push_back(idx);
					tmpDelIdx.clear();
					break; // we're done if we delete the current rect
				}
				else {
					tmpDelIdx.push_back(oIdx);
				}
			}
		}

		delIdx.insert(delIdx.end(), tmpDelIdx.begin(), tmpDelIdx.end());
	}

	if (!delIdx.empty()) {
		std::vector<DkPolyRect> filtered;

		for (int idx = 0; idx < rects.size(); idx++) {
		
			if (std::find(delIdx.begin(), delIdx.end(), idx) == delIdx.end())
				filtered.push_back(rects[idx]);
		}

		moutc << "[" << className << "] " << rects.size()-filtered.size() << " rectangles removed, remaining: " << filtered.size() << dkendl;
		rects = filtered;
	}
}

void DkBurgerSegmentation::filterDuplicatedCircles(std::vector<DkCircle>& circles, const std::vector<DkPolyRect>& rects, float overlap /* = 0.6f */) const {

	std::vector<DkCircle> filtered;

	for (DkCircle circle : circles) {

		DkPolyRect cP = circle.toPoly();
		bool delCircle = false;
		for (DkPolyRect poly : rects) {

			double intersection = abs(cP.intersectArea(poly));
			if (intersection/circle.getArea() > overlap) {
				moutc << "[" << className << "] intersection: " << intersection/circle.getArea() << dkendl;
				delCircle = true;
				break;
			}
		}

		if (!delCircle)
			filtered.push_back(circle);
	}

	circles = filtered;
}

std::string DkBurgerSegmentation::toString() const {

	std::string msg = DkUtils::stringify(rects.size()) + " found";

	return msg;
}

void DkBurgerSegmentation::draw(Mat& img, const cv::Scalar& col) const {

	draw(img, rects, col);
}

void DkBurgerSegmentation::draw(Mat& img, const std::vector<DkPolyRect>& rects, const cv::Scalar& col) const {

	for(size_t idx = 0; idx < rects.size(); idx++) {
		const DkPolyRect& r = rects[idx];
		r.draw(img, col);
	}
}
