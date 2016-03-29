/*******************************************************************************************************
 DkMath.cpp
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

#include "DkMath.h"

double DkMath::computePcaAngle(const std::vector<DkVector>& points) {

	Mat ptMat((unsigned int)points.size(), 2, CV_32FC1);

	for (int rIdx = 0; rIdx < ptMat.rows; rIdx++) {
		float* ptrM = ptMat.ptr<float>(rIdx);

		ptrM[0] = points[rIdx].x;
		ptrM[1] = points[rIdx].y;
	}

	return computePcaAngle(ptMat);
}

double DkMath::computePcaAngle(const Mat& points) {

	cv::PCA pca(points, Mat(), CV_PCA_DATA_AS_ROW);
	float dx = pca.eigenvectors.at<float>(0,0);
	float dy = pca.eigenvectors.at<float>(0,1);
	
	return atan2f(dy, dx);
}



DkLine::DkLine() {
	slope = 0;
}

DkLine::DkLine(const DkVector start, const DkVector end) {
	
	this->start = (start.x < end.x) ? start : end;
	this->end = (start.x < end.x) ? end : start;

	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;
}

DkLine::DkLine(const float xStart, const float yStart, const float xEnd, const float yEnd) {
	
	start = (xStart < xEnd) ? DkVector(xStart, yStart) : DkVector(xEnd, yEnd);
	end = (xStart < xEnd) ?  DkVector(xEnd, yEnd) : DkVector(xStart, yStart);

	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;

}

DkLine::DkLine(const Point2f start, const Point2f end) {

	this->start = (start.x < end.x) ? DkVector(start) : DkVector(end);
	this->end = (start.x < end.x) ?  DkVector(end) : DkVector(start);

	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;
}

void DkLine::setStart(const float x, const float y) {
	
	this->start = DkVector(x,y);

	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;

}
void DkLine::setEnd(const float x, const float y) {

	this->end = DkVector(x,y);
	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;

}


float DkLine::euclidianDistanceSq(const DkLine& l) {

	DkVector dist = end - l.getStartVector();
	return dist * dist;
}

void DkLine::rotateLine(DkVector center, double angle) {
	DkVector start = this->getStartVector();
	DkVector end = this->getEndVector();

	start -= center;
	end -= center;

	start.rotate(angle);
	end.rotate(angle);

	start += center;
	end += center;

	setStart(start.x, start.y);
	setEnd(end.x, end.y);
	slope = (end.x - start.x) != 0 ? (end.y-start.y) / (end.x-start.x) : FLT_MAX;
}

DkLine DkLine::extendBorder(DkVector leftUp, DkVector rightDn) {

	DkVector gradient = end - start;
	DkVector gradientBorderEnd, gradientBorderStart;
	DkLine finalLine;

	if (gradient.x == 0)		//line is vertical
		return DkLine(start.x, leftUp.y, start.x, rightDn.y);
	if (gradient.y == 0)		//line is horizontal
		return DkLine(leftUp.x, start.y, rightDn.x, start.y);

	float yEnd, xEnd;
	float yStart, xStart;

	//gradientBorder is needed to check if line cuts horizontal or vertical border
	//vector points to first or third quadrant
	if (gradient.x*gradient.y < 0) {
		gradientBorderEnd.x = rightDn.x - start.x;	//+
		gradientBorderEnd.y  = leftUp.y - start.y;	//-

		gradientBorderStart.x = leftUp.x - start.x;	//-
		gradientBorderStart.y = rightDn.y - start.y;//+

		yEnd = leftUp.y;
		yStart = rightDn.y;
		xEnd = rightDn.x;
		xStart = leftUp.x;
		//vector goes down
		//vector points to second or fourth quadrant
	} else {
		gradientBorderEnd.x = rightDn.x -start.x;		//+
		gradientBorderEnd.y = rightDn.y-start.y;		//+

		gradientBorderStart.x = leftUp.x-start.x;		//-
		gradientBorderStart.y = leftUp.y - start.y;		//-

		yEnd = rightDn.y;
		yStart = leftUp.y;
		xEnd = rightDn.x;
		xStart = leftUp.x;
	}

	if (fabs(gradient.y/gradient.x) > fabs(gradientBorderEnd.y/gradientBorderEnd.x))
		finalLine.setEnd(start.x + gradient.x/gradient.y * gradientBorderEnd.y, yEnd);
	else
		finalLine.setEnd(xEnd, start.y + gradient.y/gradient.x * gradientBorderEnd.x);

	if (fabs(gradient.y/gradient.x) > fabs(gradientBorderStart.y/gradientBorderStart.x))
		finalLine.setStart(start.x + gradient.x/gradient.y * gradientBorderStart.y , yStart);
	else
		finalLine.setStart(xStart, start.y + gradient.y/gradient.x * gradientBorderStart.x);


	if (start.x > end.x) {	//line direction is from right to left -> switch end points
		DkVector tmpV = finalLine.getEndVector();
		finalLine.setEnd(finalLine.getStartVector().x, finalLine.getStartVector().y);
		finalLine.setStart(tmpV.x,tmpV.y);
	}


	return finalLine;
}

DkVector DkLine::getLineInterSect(const DkLine& line) {

	DkVector g1 = end-start;
	DkVector g2 = line.end-line.start;

	float d = g1.vectorProduct(g2);

	// lines are parallel or line == this
	if (!d)
		return DkVector();

	float d1 = (line.start - start).vectorProduct(g2);
	//float d2 = (start - line.start)*g2;

	return start + d1/d*g1;
}

DkVector DkLine::getLineInterSect(DkVector line) {
	
	DkVector gradient = end - start;
	DkVector diff;
	DkVector final;

	//lines are parallel
	if ((gradient.x == 0 && line.x == 0) || (gradient.y==0 && line.y==0))
		return DkVector();

	//line is neither horizontal or vertical
	if (line.x!=0 && line.y!=0)
		return DkVector();

	if (gradient.x == 0)		//line is vertical
		return DkVector(start.x, line.y);
	if (gradient.y == 0)		//line is horizontal
		return DkVector(line.x, start.y);

	diff = line - start;

	if (line.y==0) { //line is vertical
		final.x = line.x;
		final.y = start.y + diff.x * gradient.y/gradient.x;
	} else {
		final.y = line.y;
		final.x = start.x + diff.y * gradient.x/gradient.y;
	}

	return final;
}



float DkLine::euclidianDistance(const DkLine& l) {

	return (end - l.getStartVector()).norm();
}

float DkLine::getLen() {

	return (end-start).norm();
}

Point DkLine::getStartPoint() const {

	return start.getCvPoint();
}

Point DkLine::getEndPoint() const {

	return end.getCvPoint();
}

DkVector DkLine::getStartVector() const {
	
	return start;
}

DkVector DkLine::getEndVector() const {

	return end;
}

DkVector DkLine::getCenter() const {

	DkVector center = end-start;
	center *= 0.5f;
	center += start;

	return center;
}

void DkLine::swap() {
	start.swap();
	end.swap();
}

std::string DkLine::toString() {

	return "start: " + start.toString() + " end: " + end.toString() + " ";
}


//-------------------------------------------------------------------------------------------------------------------------------------------

DkLineExt::DkLineExt() : DkLine () {

	this->orientation = 0;
	this->thickness = 0;
	this->lineWeight = 0.0f;
}

DkLineExt::DkLineExt(const float xStart, const float yStart, const float xEnd, const float yEnd, const float orientation, const float thickness) : DkLine(xStart, yStart, xEnd, yEnd) {

	this->orientation = orientation;
	this->thickness = thickness;
	this->lineWeight = 0.0f;
}

DkLineExt::DkLineExt(const Point2f start, const Point2f end, const float orientation, const float thickness) : DkLine(start, end) {
	this->orientation = orientation;
	this->thickness = thickness;
	this->lineWeight = 0.0f;
}

DkLineExt::DkLineExt(const DkVector start, const DkVector end, const float orientation, const float thickness) : DkLine(start, end) {
	this->orientation = orientation;
	this->thickness = thickness;
	this->lineWeight = 0.0f;
}

void DkLineExt::setOrientation(float o) {
	this->orientation = o;
}

void DkLineExt::setThickness(float t) {
	this->thickness = t;
}

void DkLineExt::setLineWeight(float w) {
	this->lineWeight = w;
}

std::string DkLineExt::toString() {

	std::string outS = DkLine::toString();
	outS += "angle: " + DkUtils::stringify(orientation*DK_RAD2DEG, 2);
	outS += " width: " + DkUtils::stringify(thickness);
	outS += " weight: " + DkUtils::stringify(lineWeight);

	return outS;
}

//-------------------------------------------------------------------------------------------------------------------------------------------

DkBox::DkBox(DkRectCorners &r) {

	DkVector minV = r.a, maxV = r.a;
	minV = minV.getMinVec(r.b);
	minV = minV.getMinVec(r.c);
	minV = minV.getMinVec(r.d);

	maxV = maxV.getMaxVec(r.b);
	minV = minV.getMinVec(r.c);
	minV = minV.getMinVec(r.d);

	this->uc = minV;
	this->lc = maxV;

	if (size().width < 0 || size().height < 0)
		DkUtils::printDebug(DK_WARNING, "the size is < 0: %s\n", size().toString().c_str());
	
}

std::string DkRect::toString() {
	std::string msg = "[center: " + center.toString() + " size: " + size.toString() + " angle: " + DkUtils::stringify(angle * DK_RAD2DEG, 3) + "]";
	return msg;
}

std::string DkInterestPoint::toString() {

	std::string msg = vec.toString() + " val: " + DkUtils::stringify(val, 3);
	return msg;
}

std::string DkDescriptor::toString() {

	std::string msg = pos.toString() + " val: " + DkUtils::stringify(val, 3) + " radius: " + DkUtils::stringify(getRadius(), 3);
	return msg;
}

//-------------------------------------------------------------------------------------------------------------------------------------------

double DkRectCorners::intersectArea(DkRectCorners *r) {

	DkIntersectPoly ip(this->getCorners(), r->getCorners());

	return ip.compute();
}

//-------------------------------------------------------------------------------------------------------------------------------------------
std::string DkVector::toString() {

	return "<" + DkUtils::stringify(x) + ", " + DkUtils::stringify(y) + ">";
}

std::string DkVector3::toString() {

	return "<" + DkUtils::stringify(x) + ", " + DkUtils::stringify(y) + ", " + DkUtils::stringify(z) + ">";
}
