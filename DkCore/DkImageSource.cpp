/*******************************************************************************************************
 DkImageSource.cpp
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

#include "DkImageSource.h"


DkImageSource::DkImageSource(Mat rgb, Mat mask) {

	this->rgb = rgb;
	this->mask = mask;

	init();
}

DkImageSource::DkImageSource(Mat rgb) {

	this->rgb = rgb;

	init();
}

void DkImageSource::init() {

	sigmaErodeBoundary = 3.0f*6.0f;
}

DkImageSource::DkImageSource() {

	init();
}


void DkImageSource::setInput(Mat rgb, Mat mask) {

	this->rgb = rgb;
	this->mask = mask;

	checkInput();
}

void DkImageSource::release() {

	if (!rgb.empty()) rgb.release();
	if (!mask.empty()) mask.release();
	if (!normRgb32F.empty()) normRgb32F.release();
	if (!grayImg.empty()) grayImg.release();
	if (!grayImg32F.empty()) grayImg32F.release();
	if (!normGrayImg32F.empty()) normGrayImg32F.release();
	if (!mask32F.empty()) mask32F.release();
	if (!maskEroded.empty()) maskEroded.release();
	if (!maskEroded32F.empty()) maskEroded32F.release();
}

void DkImageSource::setSigmaErodeMask(float sigma) {
	
	if (sigma > 1)
		sigmaErodeBoundary = sigma;
	else {
		std::string msg = "Sigma must be > 1, it is: " + DkUtils::stringify(sigma) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}


bool DkImageSource::empty() {

	return (rgb.empty() || mask.empty());
}


void DkImageSource::checkInput() {

	if (rgb.empty()) throw DkMatException("empty rgb mat", __LINE__, __FILE__);
	if (rgb.type() != CV_8UC3) throw DkMatException("not a CV_8UC3 rgb input image", __LINE__, __FILE__);

	// check if the mask fits to the image
	if (!mask.empty() && rgb.size() != mask.size()) {
		std::string msg = "Image size does not correspond to the mask's size (" + DkVector(rgb.size()).toString() + " != " + DkVector(mask.size()).toString() + ")";
		throw DkMatException(msg, __LINE__, __FILE__);
	}
}

Mat DkImageSource::getRgbImg() {

	checkInput();
	return rgb;
}

Mat DkImageSource::getMask() {

	checkInput();
	if (mask.empty()) estimateMask();
	return mask;
}

Mat DkImageSource::getNormRgb32f() {

	checkInput();

	if (normRgb32F.empty()) {
		rgb.convertTo(normRgb32F, CV_32F);
		normalize(normRgb32F, normRgb32F, 1, 0, NORM_MINMAX);
	}
	return normRgb32F;
}

Mat DkImageSource::getGrayImg() {
	
	checkInput();

	if (grayImg.empty()) {
		cvtColor(rgb, grayImg, CV_RGB2GRAY);
	}
	return grayImg;
}

Mat DkImageSource::getGrayImg32F() {
	
	checkInput();

	if (grayImg32F.empty()) {
		if (grayImg.empty()) {
			cvtColor(rgb, grayImg, CV_RGB2GRAY);
		}
		grayImg.convertTo(grayImg32F, CV_32F, 1.0f/255.0f);		
	}
	return grayImg32F;
}

Mat DkImageSource::getNormGrayImg32F() {
	
	checkInput();
	if (normGrayImg32F.empty()) {
		if (grayImg.empty()) {
			cvtColor(rgb, grayImg, CV_RGB2GRAY);
		}
		if (grayImg32F.empty()) {
			grayImg.convertTo(grayImg32F, CV_32F, 1.0f/255.0f);
		}
		normalize(grayImg32F, normGrayImg32F, 1, 0, NORM_MINMAX);
	}
	return normGrayImg32F;
}

Mat DkImageSource::getMask32F() {
	
	checkInput();
	if (mask32F.empty()) {
		getMask().convertTo(mask32F, CV_32F, 1.0f/255.0f);
	}
	return mask32F;
}

Mat DkImageSource::getErodedMask() {
	
	checkInput();
	double minVal, maxVal;
	
	if (maskEroded.empty()) {
		minMaxLoc(getMask(), &minVal, &maxVal);
		if (minVal != maxVal)
			maskEroded = DkIP::fastErodeImage(getMask(), (int)sigmaErodeBoundary);
		else
			maskEroded = mask;
	}
	return maskEroded;
}

Mat DkImageSource::getErodedMask32F() {
	checkInput();
	double minVal, maxVal;

	if (maskEroded32F.empty()) {
		if (maskEroded.empty()) {
			minMaxLoc(getMask(), &minVal, &maxVal);
			if (minVal != maxVal)
				maskEroded = DkIP::fastErodeImage(getMask(), (int)sigmaErodeBoundary);
			else
				maskEroded = mask;
		}
		maskEroded.convertTo(maskEroded32F, CV_32F, 1.0f/255.0f);
	}
	return maskEroded32F;
}

void DkImageSource::estimateMask() {

	//Mat imgF = getGrayImg32F();
	//Mat hist = DkIP::computeHist(imgF);
	//double thresh = DkIP::getThreshOtsu(hist);
	//mask = imgF > thresh/255.0f;

	//// check the ratio of the border pixels if there is foreground
	//int borderPixelCount = 0;
	//for(int y = 1; y < mask.rows-1; y++) {
	//	const unsigned char* curRow = mask.ptr<unsigned char>(y);
	//	borderPixelCount += curRow[0] + curRow[mask.cols-1];
	//}
	//const unsigned char* firstRow = mask.ptr<unsigned char>(0);
	//const unsigned char* lastRow = mask.ptr<unsigned char>(mask.rows-1);
	//for (int x = 0; x < mask.cols; x++) {
	//	borderPixelCount += firstRow[x] + lastRow[x];
	//}

	//if (((float)borderPixelCount/255)/((mask.rows*2+2*mask.cols)-4) > 0.50) {
	//	DkUtils::printDebug(DK_MODULE, "using empty mask\n");
	//	mask.setTo(255);
	//	return;
	//}

	//Mat zeroBorderMask = Mat(mask.rows+2, mask.cols+2, mask.type()); // make a zero border image because findContours uses zero borders
	//zeroBorderMask.setTo(0);
	//Mat smallZeroBorderMask = zeroBorderMask(Rect(1,1,mask.cols, mask.rows)); // copyTo needs reference
	//mask.copyTo(smallZeroBorderMask);

	//DkBlobs<DkAttr> blobs = DkBlobs<DkAttr>(zeroBorderMask);
	//blobs.calcProps();
	//blobs.calcArea();

	//vector<DkAttr> attr = blobs.getProps();
	//vector<DkAttr>::iterator attrIter = attr.begin();

	//int maxArea = 0;
	//int maxIdx = 0;

	//for (int idx = 0; attrIter != attr.end(); idx++) {

	//	if ((*attrIter).getArea() > maxArea) {

	//		maxArea = (*attrIter).getArea();
	//		maxIdx = idx;
	//	}

	//	attrIter++;
	//}

	//if (attr.size() > 0 && maxArea != 0) {
	//	// calculate mean and standard deviation of the gray values at the border of the image
	//	Mat border(imgF.rows, imgF.cols, CV_8UC1);
	//	border.setTo(255);
	//	border(Rect(1,1,imgF.cols-2, imgF.rows-2)).setTo(0);
	//	border=border-mask; // set pixels at the border which are detected by otsu to 0
	//	Scalar meanBorder;
	//	Scalar stdDevBorder;
	//	meanStdDev(imgF, meanBorder, stdDevBorder, border);
	//	
	//	// if the background is perfectly uniform, the std might get < 0.003
	//	if (stdDevBorder[0] < 0.051) stdDevBorder[0] = 0.051;

	//	zeroBorderMask.setTo(0); 
	//	blobs.drawBlob(zeroBorderMask, attr[maxIdx],0);

	//	for (int x = 0; x < imgF.cols; ) { 
	//		cv::Rect rect;
	//		cv::Point p1(x,0);
	//		cv::Point p2(x,imgF.rows-1);
	//		if (imgF.at<float>(p1) <= meanBorder[0])
	//			floodFill(imgF, zeroBorderMask, cv::Point(x,0), Scalar(2000), &rect, Scalar(5), stdDevBorder, FLOODFILL_MASK_ONLY); // newVal (Scalar(2000)) is ignored when using MASK_ONLY
	//		if (imgF.at<float>(p2) <= meanBorder[0])
	//			floodFill(imgF, zeroBorderMask, cv::Point(x,imgF.rows-1), Scalar(2000), &rect, Scalar(5), stdDevBorder, FLOODFILL_MASK_ONLY); // newVal (Scalar(2000)) is ignored when using MASK_ONLY
	//		if (imgF.cols/20 > 0)
	//			x+=imgF.cols/20; // 5 seed points for the first and last row
	//		else 
	//			x++;
	//	}
	//	for (int y = 0; y < imgF.rows; ) { 
	//		cv::Rect rect;
	//		cv::Point p1(0,y);
	//		cv::Point p2(imgF.cols-1,y);
	//		if (imgF.at<float>(p1) <= meanBorder[0])
	//			floodFill(imgF, zeroBorderMask, cv::Point(0,y), Scalar(2000), &rect, Scalar(5), stdDevBorder, FLOODFILL_MASK_ONLY); // newVal (Scalar(2000)) is ignored when using MASK_ONLY
	//		if (imgF.at<float>(p2) <= meanBorder[0])
	//			floodFill(imgF, zeroBorderMask, cv::Point(imgF.cols-1,y), Scalar(2000), &rect, Scalar(5), stdDevBorder, FLOODFILL_MASK_ONLY); // newVal (Scalar(2000)) is ignored when using MASK_ONLY

	//		if (imgF.rows/20 > 0)
	//			y+=imgF.rows/20; // 5 seed points for the first and last col
	//		else
	//			y++;
	//	}

	//	zeroBorderMask = zeroBorderMask != 1;				
	//	mask = zeroBorderMask(Rect(1,1,mask.cols, mask.rows));
	//	
	//	DkBlobs<DkAttr> filterBlobs = DkBlobs<DkAttr>(zeroBorderMask); 
	//	filterBlobs.calcArea();
	//	filterBlobs.imgFilterArea(mask.rows*mask.cols/8);

	//	//Mat maskClean = mask & getGrayImg() > 5;

	//	//mout << "mask difference: " << sum(mask-maskClean)[0]/255.0f << dkendl;

	//	//if (sum(mask-maskClean)[0]/255.0f < 100000) {
	//	//	mask = maskClean;
	//	//}
	//}
	//else {
	//	DkUtils::printDebug(DK_WARNING, "[DkImageSource] the thresholded image seems to be empty -> no mask created\n");
	//	mask = 255;
	//}
}

void DkImageSource::checkConsistency(DkImageSource *imgsCmp) {
	
	DkUtils::printDebug(DK_MODULE, "[DkImageSource] consistency check -----------------------------\n");
	
	Scalar diff = sum(this->getRgbImg() - imgsCmp->getRgbImg());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the RGB (8U) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getMask() - imgsCmp->getMask());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the mask (8U) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getNormRgb32f() - imgsCmp->getNormRgb32f());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the normalized RGB (32F) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getGrayImg() - imgsCmp->getGrayImg());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the gray-scale (8U) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getGrayImg32F() - imgsCmp->getGrayImg32F());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the gray-scale (32F) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getNormGrayImg32F() - imgsCmp->getNormGrayImg32F());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the normalized gray-scale (32F) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getMask32F() - imgsCmp->getMask32F());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the mask (32F) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getErodedMask() - imgsCmp->getErodedMask());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the eroded mask (8U) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);

	diff = sum(this->getErodedMask32F() - imgsCmp->getErodedMask32F());
	if (diff[0] != 0.0f) DkUtils::printDebug(DK_WARNING, "the eroded mask (32F) images in line: %i are not the same: %.3f\n", __LINE__, diff[0]);
	
	DkUtils::printDebug(DK_MODULE, "[DkImageSource] consistency check completed -------------------\n");
}
