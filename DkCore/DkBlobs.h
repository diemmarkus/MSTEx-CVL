/*******************************************************************************************************
 DkBlobs.h
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Markus Diem <markus@nomacs.org>
 Copyright (C) 2014-2015 Fabian Hollaus <holl@caa.tuwien.ac.at>
 Copyright (C) 2014-2015 Florian Kleber <kleber@caa.tuwien.ac.at>
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

#pragma once

#include "DkAttr.h"
#include "DkError.h"
#include "DkUtils.h"
#include "DkMath.h"
#include "DkIP.h"


/**
 * The DkBlobs class is used to label a binary image. It calculates the properties
 * specified in DkAttr and allows to filter the blobs in the image according e.g. the size
 * of a blob. Blobs are saved as contours (a labeled image is not returned). 
 * In addition it allows to delete blobs in the image, or otherwise analyze
 * the saved blob properties without manipulating the image.
 **/
template <class Attr> class DkBlobs {

public:

	bool saveContour;		/**< save contours to the attr vector**/
	int approxMethod;			/**< contour approximation. (default: CV_CHAIN_APPROX_SIMPLE - see OpenCV findContours)**/

	/**
	 * Default constructor.
	**/
	DkBlobs();
	/**
	 * Default constructor (the image referenced has to be CV_8UC1).
	 * 0's are background, non-zero pixels are treated as foreground.
	 * @param img The binary input image as unsigned char Mat with 1 channel (CV_8UC1).
	**/
	DkBlobs(Mat img);

	//void setImg(Mat img);
	/** 
	 * Filters the image according the specified area. All blobs smaller than this area
	 * are removed in the input image.
	 * @param threshArea The blob size threshold in pixel.
	 */
	void imgFilterArea(int minArea, int maxArea = -1);
	/** 
	 * Filters the blobs according the specified area. All blobs smaller than this area
	 * are removed in attribute list. The image is not manipulated.
	 * @param threshArea The blob size threshold in pixel.
	 */
	void filterArea(int threshArea);
	/** 
	 * Filters the image according the aspect ratio of the minimum area rectangle containing the blob. 
	 * All blobs with a higher aspect ratio are removed.
	 * @param ratio The ratio threshold.
	 */
	void imgFilterMarRatio(float ratio);
	/** 
	 * Filters the image according the width of the minimum area rectangle containing the blob. 
	 * All blobs with a smaller width are removed.
	 * @param minWidth The minimum width in pixel.
	 */
	void imgFilterMarWidth(int minWidth);
	void imgFilterMarWidthCheckLines(int minWidth, std::list<DkLine> rulLin);
	/** 
	 * Filters the image according the aspect ratio and the width of the minimum area rectangle containing the blob. 
	 * All blobs with a higher aspect ratio or a smaller width are removed.
	 * @param ratio The ratio threshold.
	 * @param minWidth The minimum width in pixel.
	 */
	void imgFilterMar(float ratio, int minWidth);
	/** 
	 * Filters the image according the ripple value of a blob: a blob is interpreted as line and the ripple of a line
	 * is defined as ratio of the blob area/area of the minimum area rectangle.
	 * @param ratio The ratio threshold.
	 * @param minWidth The minimum width in pixel of the line.
	 */
	void imgFilterRipple(float ratio, int minWidth);
	/** 
	 * Filters the image according the orientation of the snippet and orientation of the blob.
	 * Only horizontal and vertical lines are allowed.
	 * @param angle of the snippet in rad
	 * @param maxAngleDiff the maximal angle deviation of a blob from 0° or 90° 
	 */
	void imgFilterOrientation(float angle, float maxAngleDiff);
	/** 
	 * Calculates the Area property of all blobs without manipulating the image.
	 */
	void calcArea();
	/** 
	 * Calculates the Orientation property of all blobs without manipulating the image.
	 */
	void calcOrientation();
	/** 
	 * Calculates the Bounding Box of all blobs without manipulating the image.
	 */
	void calcBb();
	/** 
	 * Calculates the Minimum Area Rectangle of all blobs without manipulating the image.
	 */
	void calcMar();
	/** 
	 * Calculates all properties of all blobs without manipulating the image.
	 */
	void calcProps();
	/** 
	 * Calculates the contours of all blobs without manipulating the image.
	 * The border of the image is set to 0 !!! (blobs located at the border are not correctly reproduced)
	 */
	void calcContours();	
	/** 
	 * Calculates the labled image (the values in the image are 0 for background and contourIdx+1 (!!!) for the labels)
	 */
	void calcLabeledImage();	
	/** 
	 * Clears all saved Contours to save memory.
	 */
	void clearContours();
	/** 
	 * Returns the image reference of the image.
	 * @return Reference to the analyzed image, respectively manipulated image according the blob properties.
	 */
	Mat getBwImg(){return bwImg;};
	/** 
	 * All calculated properties are stored in vector and returned.
	 * @return calculated attributes.
	 */
	vector<Attr> getProps() {/*calcProps();*/ return props;};
	/** 
	 * Overwrites the props vector with the one given
	 * @param the new props vector
	 */
	void setProps(vector<Attr> props) {this->props = props;};
	/** 
	 * Returns the number of blobs in an image.
	 * @return number of blobs in an image.
	 */
	int getSize() {return size;};
	/** 
	 * Draws a blob. The resulting image has the size of the bounding box of the blob.
	 * @param blob to draw.
	 * @param contour level [default: 1], if it is 0, solely the outlines are drawn
	 * @return blob image with bounding box dimensions.
	 */
	Mat getBlobImg(Attr *blob, int maxLevel = 1, Scalar val = Scalar(255));

	static Mat getFilledBlobImg(Attr *blob, Scalar val = Scalar(255));

	/** 
	 * Draws a blob into the image obtained
	 * In findContours the border of the image is set to 0 !!! (blobs located at the border are not correctly reproduced)
	 * @param img a CV_8U image.
	 * @param blob to draw.
	 * @param contour level [default: 1], if it is 0, solely the outlines are drawn
	 * @return blob image with bounding box dimensions.
	 */
	void drawBlob(Mat &img, Attr &blob, int maxLevel = 1, Scalar val = Scalar(255));

	/** 
	 * Returns the first point of a contour
	 * @param the index of the contour.
	 * @return the first point of the contour or 0 if the contour does not exist.
	 */
	Point getContourStartPoint(int contourIdx) { if (contours[contourIdx].size() != 0) return contours[contourIdx][0]; else return 0;};
	/** 
	 * Returns the contour of a  blob
	 * @param the index of the contour.
	 * @return the contour of the blob, 0 if the contour does not exist
	 */
	vector<Point> getContourOfBlob(int contourIdx) { return contours[contourIdx];};
	/** 
	 * Returns all contours
	 * @return all contours
	 */
	vector<vector<Point> > getContours() { return contours;};

	/** 
	 * Returns the labeled image. Each blob in the image has the value of the contourIdx+1 (ATTENTION!!!) 
	 * @return the labeled image. Each blob in the image has the value of the contourIdx+1 (ATTENTION!!!) [CV_16U]
	 */
	Mat getLabeledImage() {if (labeledImage.empty()) calcLabeledImage(); return labeledImage;};

	/** 
	 * Returns the Attr of the blob at the given position
	 * @return the Attr of the blob at the given position
	 */
	Attr getBlobAt(int y, int x) {
		if (labeledImage.empty()) calcLabeledImage(); 
		if (labeledImage.at<unsigned short>(y,x)-1==-1) return Attr(); 
		else return props[contourIdxInPropVector[labeledImage.at<unsigned short>(y,x)-1]];
	};

	/** 
	 * Returns the Attr of the blob with the given contourIdx
	 * @return the Attr of the blob with the given contourIdx
	 */
	Attr getBlob(int contourIdx) { return props[contourIdxInPropVector[contourIdx]]; };

	/**
	* Replaces the blob with the given contourIdx in the props vector with the given DkAttr
	* @param contourIdx the contourIdx which should be replaced
	* @param attr the new DkAttr which should be written into the props vector
	*/
	void setBlob(int contourIdx, Attr attr) {props[contourIdxInPropVector[contourIdx]]=attr;};


	/**
	 * Calculates the median word height and width in a given image.
	 * @param img segmented image in which the median word height should be calculated
	 * @param angle the angle of the page
	 * @param medianWidth the median width
	 * @param medianHeight the median height
	 **/ 
	static void calculateMedianWordDims(const Mat img, double angle, float* medianWidth, float* medianHeight);
	/** 
	 * Default Destructor.
	 */
	~DkBlobs() {};

protected:
	int size;							/**< the number of blobs **/							
	Mat bwImg;							/**< the segmented image which is labelled **/
	vector<vector<Point> > contours;	/**< vector with all blob contours  **/
	vector<Vec4i> hierarchy;			/**< hierarchy vector of all blobs (see findContours)  **/
	vector<Attr> props;					/**< property vector  **/
	std::map<int, int> contourIdxInPropVector; /**< map to allocate which contour index is which blob in the props vector **/
	Mat labeledImage;					/**< the image with the labels (the values in the image are 0 for background and contourIdx+1 (!!!) for the labels) **/

};


/**
 * The DkColorBlobs class is a derived class of DkBlobs. In addition to the properties in
 * DkBlobs the mean color weighted with the magnitude image and the saturation weighted hue histogram is calculated.
 **/
template <class ColorAttr> class DkColorBlobs  : public DkBlobs<ColorAttr> {

public:
	/**
	 * Default constructor.
	 * The images referenced has to be CV_8UC1. 0's are background, non-zero pixels are treated as foreground. img is the result of the segmentation.
	 * The mean color is calculated for all blobs.
	 * @param img The binary input image as unsigned char Mat with 1 channel (CV_8UC1) (segmented image)
	**/
	DkColorBlobs(Mat img);
	/**
	 * Default destructor.
	**/
	~DkColorBlobs() {}; 
	/** 
	 * Calculates the mean color [0 255] of blobs defined in image img, weighted with the magnitude image magn [0 1]. In addition the saturation
	 * weighted hue histogram is calculated.
	 * @param rgb The color image [0 255].
	 * @param magn The magnitude image [0 1].
	 */
	void calcMeanColors(Mat rgb, Mat magn=Mat());
	//void calcColorProps();
	/** 
	 * Returns the number of blobs in an image.
	 * @return number of blobs in an image.
	 */
	int getNumBlobs() {return (int)this->props.size();};
	/** 
	 * Returns the saturation histogram of the analyzed image. The default value for the number of bins is 360.
	 * @return sat histogram of an Image as 1-dimensional Mat(1,bins,CV_32FC1).
	 */
	Mat getSatHist() {return satHist;}
	/** 
	 * Returns the hue histogram of the image.
	 * @return hue histogram of an Image as 1-dimensional Mat(1,bins,CV_32FC1).
	 */
	Mat getHueHist() {return hueHist;};
	/** 
	 * Returns the l histogram of the image (number of bins is 100).
	 * @return l histogram of an Image as 1-dimensional Mat(1,bins,CV_32FC1).
	 */
	Mat getValHist() {return valHist;};
	/** 
	 * Returns the v weighted hue histogram the of the image.
	 * @return v weighted hue histogram of an Image as 1-dimensional Mat(1,bins,CV_32FC1).
	 */
	Mat getValColHist() {return valColHist;};
	/** 
	 * Sets the number of bins for hue histogram (Mat(1,bins,CV_32FC1)). The default value is 360.
	 * @param c The number of bins for the hue histogram.
	 */
	void setBins(int c);
	/** 
	 * Returns the current number of bins of the  hue histogram of the analyzed image.
	 * @return Bin value for the  hue Histogram
	 */
	int getBins();
	/** 
	 * Sets the the threshold value. Peaks with circular mean length below the THRESHOLD are grayvalues. The default is 10;
	 * @param t the threshold for color peaks.
	 */
	void setThreshold(float t);

private:
	int bins;	//number of bins
	float threshold;	//peaks with circular mean length below the THRESHOLD are skipped
	//Mat hueSatHist;			//saturation weighted hue histogram
	Mat satHist;		//sat histogram
	Mat hueHist;		//hue histogram
	Mat valHist;		//value histogram
	Mat valColHist;		

};

template <class ColorAttr> void DkColorBlobs<ColorAttr>::setBins(int c) {
	if (c > 1)
		bins = c;
	else {
		std::string msg = "Bins > 1, it is: " + DkUtils::stringify(c) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

template <class ColorAttr> void DkColorBlobs<ColorAttr>::setThreshold(float t) {
	if (t > 0.01f)
		threshold = t;
	else {
		std::string msg = "threshold > 0.01, it is: " + DkUtils::stringify(t) + "\n";
		throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
	}
}

template <class ColorAttr> int DkColorBlobs<ColorAttr>::getBins() {
	return bins;
}

template <class Attr> DkBlobs<Attr>::DkBlobs() {
	props.clear();
	contourIdxInPropVector.clear();
	labeledImage.setTo(0);
	this->approxMethod = CV_CHAIN_APPROX_SIMPLE;
	saveContour = false;
}

template <class Attr> DkBlobs<Attr>::DkBlobs(Mat img) {

	// TODO: never do that...
	if (img.empty()) throw DkMatException("empty mat", __LINE__, __FILE__);
	if (img.channels() != 1) throw DkMatException("not a single channel input image", __LINE__, __FILE__);
	if (img.depth() != CV_8U) throw DkMatException("not a CV_8U image", __LINE__, __FILE__);

	//img.convertTo(bwImg, CV_8U);
	bwImg = img;
	props.clear();
	contourIdxInPropVector.clear();
	size = -1;
	labeledImage.setTo(0);
	approxMethod = CV_CHAIN_APPROX_SIMPLE;
	saveContour = false;
}

template <class Attr> void DkBlobs<Attr>::calcLabeledImage() {

	labeledImage = Mat(bwImg.size(),CV_16U);
	labeledImage.setTo(0);

	int idx = 0;
	if (!this->contours.size())
		return;


	for( ; idx >= 0; idx = this->hierarchy[idx][0])
	{
		if (this->contours[idx].size() == 0)
			continue;

		drawContours(labeledImage, this->contours, idx, Scalar(idx+1), CV_FILLED, 8, hierarchy, 1); 
	}

	//DkUtils::getMatInfo(labeledImage, "labeledImage");
}

template <class ColorAttr> DkColorBlobs<ColorAttr>::DkColorBlobs(Mat img) : DkBlobs<ColorAttr>(img) {
	bins = 360;
	threshold = 10.0f;
	//threshold = 15.0f;
}

//template <class Attr> void DkBlobs<Attr>::setImg(Mat img) {
//
//	if (img.empty()) throw DkMatException("empty mat", __LINE__, __FILE__);
//	if (img.channels() != 1) throw DkMatException("not a single channel input image", __LINE__, __FILE__);
//
//	img.convertTo(bwImg, CV_8U);
//	props.clear();
//}

template <class Attr> void DkBlobs<Attr>::imgFilterArea(int minArea, int maxArea) {
	
	Mat bwImg2 = this->bwImg.clone();
#if 0	// old method -> slow if there is a whole bunch of very small blobs

	int currArea = 0;
	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}


	//some properties are calculated, insert the missing ones

	typename vector<Attr>::iterator it;

	it = props.begin();
	dout << "area blobs #: " << props.size() << dkendl;

	bwImg.setTo(0);
	int numberOfDeleted = 0;
	while( it != props.end()) {

		if ((*it).getArea() == -1) {
			currArea = (int) fabs(contourArea(Mat(contours[(*it).getContourIdx()])));
		} else
			currArea = (*it).getArea();


		if (currArea <= minArea) {
			//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

			if (hierarchy[(*it).getContourIdx()][1] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
			if (hierarchy[(*it).getContourIdx()][0] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

			contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
			contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
			it = props.erase(it);																//clear props
			//fprintf(stdout,"idx:%d\n",(*it).getContourIdx());
			//fprintf(stdout,"itr:%d\n",(*contourIdxInPropVector.find((*it).getContourIdx())));
			numberOfDeleted++;

		} else {
			drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
			contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
			++it;
		}
	}
#else
	
	DkIP::setBorderConst(bwImg2);
	int w = bwImg2.cols;
	int h = bwImg2.rows;
	unsigned char *si  = bwImg2.data + w + 1;
	unsigned char *sie = si + w * h - 2 * w - 2;

	unsigned char **ppFGList = new unsigned char*[w * h];	// >DIR: w*h/4 -> bug when images have large blobs [6.10.2011 markus]
	int fgListPos;
	int cur_fgListPos;
	unsigned char *pCurImgPos;

#define _PUSH_FG_LIST(a) (ppFGList[fgListPos++] = (a))
#define _PUSH_FG_LIST_CLR(a) (ppFGList[fgListPos++] = (*a = 0, a))
#define _POP_FG_LIST (ppFGList[cur_fgListPos++])

	while(si < sie)
	{
		if (*si)
		{
			fgListPos = 0;
			cur_fgListPos = 0;
			_PUSH_FG_LIST_CLR(si);

			// 3 2 1
			// 4 X 8
			// 5 6 7

			while(cur_fgListPos < fgListPos)
			{
				pCurImgPos = _POP_FG_LIST - 1 - w;

				if (*pCurImgPos)
					// 3
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos++;
				if (*pCurImgPos)
					// 2
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos++;
				if (*pCurImgPos)
					// 1
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos += w;
				if (*pCurImgPos)
					// 8
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos-=2;
				if (*pCurImgPos)
					// 4
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos += w;
				if (*pCurImgPos)
					// 5
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos++;
				if (*pCurImgPos)
					// 6
					_PUSH_FG_LIST_CLR(pCurImgPos);

				pCurImgPos++;
				if (*pCurImgPos)
					// 7
					_PUSH_FG_LIST_CLR(pCurImgPos);
			}

			if (fgListPos <= minArea || (maxArea != -1 && fgListPos >= maxArea))
			{
				cur_fgListPos = 0;

				while(cur_fgListPos < fgListPos)
				{
					pCurImgPos = _POP_FG_LIST;

					*(bwImg.data + (int(pCurImgPos) - int(bwImg2.data))) = 0;
				}
			}
		}

		si++;
	}

	delete[] ppFGList;
#endif
}

template <class Attr> void DkBlobs<Attr>::filterArea(int threshArea) {

	int currArea = 0;
	Scalar color(0);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}


	//some properties are calculated, insert the missing ones
	typename vector<Attr>::iterator it;

	int numberOfDeleted = 0;
	it = props.begin();
	while( it != props.end()) {

		if ((*it).getArea() == -1) {
			currArea = (int) fabs(contourArea(contours[(*it).getContourIdx()]));
			(*it).setArea(currArea);
		} else
			currArea = (*it).getArea();

		if (currArea <= threshArea) {

			if (hierarchy[(*it).getContourIdx()][1] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
			if (hierarchy[(*it).getContourIdx()][0] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

			contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
			contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
			it = props.erase(it);															//clear props
			numberOfDeleted++;
		} else {
			++it;
			contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
		}
	}
}

template <class Attr> void DkBlobs<Attr>::imgFilterMarRatio(float ratio) {

	float currRatio = -1;
	RotatedRect rect;

	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterMarRatio(ratio);
	} else {
		//some properties are calculated, insert the missing ones
		typename vector<Attr>::iterator it;

		int numberOfDeleted = 0;
		it = props.begin();
		bwImg = 0;
		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(contours[(*it).getContourIdx()]);
				(*it).setMinAreaRect(rect);
			} else
				rect = (*it).getMinAreaRect();

			currRatio = rect.size.height > rect.size.width ? rect.size.width/rect.size.height : rect.size.height/rect.size.width;

			if (currRatio > ratio) {
				//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);															//clear props
				numberOfDeleted++;
			} else {
				drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
	}
}


template <class Attr> void DkBlobs<Attr>::imgFilterMarWidth(int minWidth) {

	float currWidth = -1;
	DkRect rect;

	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterMarWidth(minWidth);
	} else {
		//some properties are calculated, insert the missing ones
		typename vector<Attr>::iterator it;

		int numberOfDeleted = 0;
		it = props.begin();
		bwImg.setTo(0);
		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setMinAreaRect(rect);
			} else
				rect = (*it).getMinAreaRect();

			currWidth = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;

			if (currWidth < minWidth) {
				//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);															//clear props
				numberOfDeleted++;
			} else {
				drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
	}
}


template <class Attr> void DkBlobs<Attr>::imgFilterMarWidthCheckLines(int minWidth, std::list<DkLine> rulLin) {

	float currWidth = -1;
	float thickness = -1;
	float maxDistExtern = 20.0f;
	float xVec, yVec;
	float p1X, p1Y, p2X, p2Y;
	float orientation;
	DkRect rect;

	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterMarWidthCheckLines(minWidth,rulLin);
	} else {
		//some properties are calculated, insert the missing ones
		typename vector<Attr>::iterator it;

		int numberOfDeleted = 0;
		it = props.begin();
		bwImg.setTo(0);

		calcOrientation();

		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setMinAreaRect(rect);
			} else
				rect = (*it).getMinAreaRect();

			currWidth = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
			thickness = rect.size.height < rect.size.width ? rect.size.height : rect.size.width;

			//check if a ruling line is doubled by a line in the image
			std::list<DkLine>::iterator currRulLin;
			int doubled = 0;
			for(currRulLin= rulLin.begin(); currRulLin != rulLin.end(); currRulLin++) {

				orientation = (*it).getOrientation();

				xVec = -(currWidth*0.5f*(float)cos(orientation));
				yVec =  (currWidth*0.5f*(float)sin(orientation));

				p1X = rect.center.x - xVec;
				p1Y = rect.center.y - yVec;
				p2X = rect.center.x + xVec;
				p2Y = rect.center.y + yVec;

				DkLine l = p1X < p2X ? DkLine(p1X,p1Y,p2X,p2Y) : DkLine(p2X,p2Y,p1X,p1Y);
				
				float minDifFirst = l.distance(currRulLin->getStartVector());
				float minDifSecond = l.distance(currRulLin->getEndVector());

				if (minDifFirst < maxDistExtern && minDifSecond < maxDistExtern) {
					//line is doubled -> remove
					//mout << "line doubled..." << dkendl;
					doubled = 1;
					break;
				}
			}
			

			if (currWidth < minWidth && !doubled) {
				//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);															//clear props
				numberOfDeleted++;
			} else {
				drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
	}
}




template <class Attr> void DkBlobs<Attr>::imgFilterMar(float ratio, int minWidth) {

	float currWidth = -1;
	float currRatio = -1;
	DkRect rect;

	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}


	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterMar(ratio, minWidth);
	} else {
		//some properties are calculated, insert the missing ones
		typename vector<Attr>::iterator it;

		it = props.begin();
		bwImg.setTo(0);
		int numberOfDeleted = 0;
		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setMinAreaRect(rect);
			} else
				rect = (*it).getMinAreaRect();

			currWidth = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
			if ((rect.size.width == 0) || (rect.size.height == 0))
				currRatio = 0;
			else
				currRatio = rect.size.height > rect.size.width ? rect.size.width/rect.size.height : rect.size.height/rect.size.width;

			if ((currWidth < minWidth) || (currRatio > ratio)) {
				//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);															//clear props
				numberOfDeleted++;
			} else {
				drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
	}
}


template <class Attr> void DkBlobs<Attr>::imgFilterRipple(float ratio, int minWidth) {

	float currWidth = -1;
	float currRatio = -1;
	int currArea = -1;
	RotatedRect rect;

	Scalar color(255);

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}


	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterRipple(ratio, minWidth);
	} else {
		//some properties are calculated, insert the missing ones
		typename vector<Attr>::iterator it;
		it = props.begin();
		bwImg = 0;
		int numberOfDeleted = 0;
		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(contours[(*it).getContourIdx()]);
				(*it).setMinAreaRect(rect);
			} else
				rect = (*it).getMinAreaRect();

			if ((*it).getArea() == -1) {
				currArea = (int) fabs(contourArea(contours[(*it).getContourIdx()]));
				(*it).setArea(currArea);
			} else
				currArea = (*it).getArea();


			currWidth = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
			if ((rect.size.width == 0) || (rect.size.height == 0) || (currArea == 0))
				currRatio = 1;
			else
				currRatio = (float)currArea/(float)(rect.size.width*rect.size.height);

			printf("currWidth:  %f  currRatio:  %f  currArea:   %d\n", currWidth, currRatio, currArea);
			printf("width:  %f  height:  %f\n", rect.size.width, rect.size.height);
			printf("\n");


			if ((currWidth < minWidth) && (currRatio < ratio)) {
				//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);															//clear props
				numberOfDeleted++;
			} else {
				drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
	}
}

template <class Attr> void DkBlobs<Attr>::imgFilterOrientation(float angle, float maxAngleDiff) {
	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		imgFilterOrientation(angle, maxAngleDiff);
	}

	calcOrientation();
	Scalar color(255);

	typename vector<Attr>::iterator it;
	it = props.begin();
	bwImg.setTo(0);
	int numberOfDeleted = 0;
	while( it != props.end()) {

		float a = DkMath::normAngleRad((float)angle, 0.0f, (float)CV_PI);
		a = a > (float)CV_PI*0.5f ? (float)CV_PI-a : a;
		float angleNewLine = DkMath::normAngleRad((*it).getOrientation(),0.0f,(float)CV_PI);
		angleNewLine = angleNewLine > (float)CV_PI*0.5f ? (float)CV_PI-angleNewLine : angleNewLine;

		float diffangle = fabs(a - (float)angleNewLine);

		a = a > (float)CV_PI*0.25f ? (float)CV_PI*0.5f-a :a;
		angleNewLine = angleNewLine > (float)CV_PI*0.25f ? (float)CV_PI*0.5f-angleNewLine : angleNewLine;

		diffangle = diffangle < fabs(a - (float)angleNewLine) ? diffangle : fabs(a - (float)angleNewLine);

		if (diffangle > maxAngleDiff/180.0f*(float)CV_PI) {
			//drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image

			if (hierarchy[(*it).getContourIdx()][1] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];     //set sucessor of predecessor of idx to sucessor of idx
			if (hierarchy[(*it).getContourIdx()][0] != -1)
				hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

			contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
			contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
			it = props.erase(it);															//clear props
			numberOfDeleted++;
		} else {
			drawContours(bwImg, contours, (*it).getContourIdx(), color, CV_FILLED, 8, hierarchy, 1);	//delete blob within image
			contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
			++it;
		}
	}
}

template <class Attr> void DkBlobs<Attr>::calcProps() {

	DkTimer dt = DkTimer();

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}


	//some properties are calculated, insert the missing ones
	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcProps();
	} else {

		typename vector<Attr>::iterator it;
		RotatedRect rect;
		Rect bb;
		it = props.begin();
		while( it != props.end()) {

			if ((*it).calcMinAreaRect() && (*it).calcBbRect() && ((*it).getArea() != -1)) break;  //already calculated

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setMinAreaRect(rect);
			}

			if (!(*it).calcBbRect()) {
				bb = boundingRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setBbRect(bb);
			}

			if ((*it).getArea() == -1) {
				int a = (int) fabs(contourArea(Mat(contours[(*it).getContourIdx()])));
				(*it).setArea(a);
			}
			it++;
		}//while
	}//else (contours.size() == 0)

	mout << "[DkBlobs] BlobProps computed in: " << dt << dkendl;
}

template <class Attr> void DkBlobs<Attr>::clearContours() {

	if (contours.size() > 0) contours.clear();
	if (hierarchy.size() > 0) hierarchy.clear();
}

template <class Attr> Mat DkBlobs<Attr>::getBlobImg(Attr *blob, int maxLevel, Scalar val) {

	DkBox bb = blob->getBbRect();
	DkVector offset = bb.uc * -1.0f;

	Mat bImg = Mat(bb.getSize(), CV_8UC1);
	bImg.setTo(0);

	drawContours(bImg, contours, blob->getContourIdx(), val, CV_FILLED, 8, hierarchy, maxLevel, offset.getCvPoint());

	return bImg;

}

template <class Attr> Mat DkBlobs<Attr>::getFilledBlobImg(Attr *blob, Scalar val) {

	if (!blob)
		return Mat();
	if (!blob->getContour()) {
		wout << "[WARNING] cannot draw empty contour" << dkendl;
		return Mat();
	}

	DkBox bb = blob->getBbRect();
	DkVector offset = bb.uc * -1.0f;

	Mat bImg = Mat(bb.getSize(), CV_8UC1);
	bImg.setTo(0);

	std::vector<std::vector<Point> > contours;
	contours.push_back(*blob->getContour());

	drawContours(bImg, contours, -1, val, CV_FILLED, 8, noArray(), INT_MAX, offset.getCvPoint());

	return bImg;

}


template<class Attr> void DkBlobs<Attr>::drawBlob(Mat &img, Attr &blob, int maxLevel, Scalar val) {

	if (img.empty()) {
		std::string msg = "the image is empty...\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}

	if (img.channels() > 1) {
		std::string msg = "The image has: " + DkUtils::stringify(img.channels()) + ", but 1 channel is required.\n";
		throw DkMatException(msg, __LINE__, __FILE__);
	}

	if (img.type() != CV_8U) {
		std::string msg = "The image must be CV_8U but it is: " + DkUtils::getMatInfo(img);
		throw DkMatException(msg, __LINE__, __FILE__);
	}

	drawContours(img, contours, blob.getContourIdx(), val, CV_FILLED, 8, hierarchy, maxLevel);
}

template <class Attr> void DkBlobs<Attr>::calcArea() {

	//if not calculated, calculate all properties for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	//some properties are calculated, insert the missing ones

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcArea();
		//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	} else {

		typename vector<Attr>::iterator it;
		RotatedRect rect;
		Rect bb;
		it = props.begin();
		while( it != props.end()) {
			if ((*it).getArea() == -1) {
				int a = (int)fabs(contourArea(Mat(contours[(*it).getContourIdx()])));
				(*it).setArea(a);
			} else break;  //already calculated
			it++;
		}//while
	}//else (contours.size() == 0)


}

template <class Attr> void DkBlobs<Attr>::calcContours() {
	//if not calculated, calculate the contours of the blobs
	if (props.size() == 0) {
		//if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
			Mat help = bwImg.clone();   //findContours alters input image
			contours.clear();
			hierarchy.clear();
			findContours(help, contours, hierarchy, RETR_CCOMP, approxMethod);
			size = (int)contours.size();
			if (size == 0) return;
			//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		//}

		int idx = 0;
		for( ; idx >= 0; idx = hierarchy[idx][0]) {
			Attr a;
			a.setContourIdx(idx);
			if (saveContour)
				a.setContour(contours[idx]);

			contourIdxInPropVector[idx]=(int)props.size();
			props.push_back(a);
		}
	}
}

template <class Attr> void DkBlobs<Attr>::calcOrientation() {

	double u00,u11,u01,u10,u20,u02, num, den;
	float o;
	Moments m;
		//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	//some properties are calculated, insert the missing ones
	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcOrientation();
		//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	} else {

		//formula according http://public.cranfield.ac.uk/c5354/teaching/dip/opencv/SimpleImageAnalysisbyMoments.pdf
		//and http://en.wikipedia.org/wiki/Image_moment
		typename vector<Attr>::iterator it;
		it = props.begin();
		while( it != props.end()) {
			if ((*it).getOrientation() == -1) {

				m = moments(Mat(contours[(*it).getContourIdx()]));
				u00 = m.m00;

				if (m.m00 <= 0)
					o = 0;
				else {

					u10 = m.m10 / u00;
					u01 = m.m01 / u00;

					u11 = -(m.m11 - m.m10 * m.m01 / u00 ) / u00;
					u20 = (m.m20 - m.m10 * m.m10 / u00 ) / u00;
					u02 = (m.m02 - m.m01 * m.m01 / u00 ) / u00;

					num = 2*u11;
					den = u20 - u02;// + sqrt((u20 - u02)*(u20 - u02) + 4*u11*u11);

					if( num != 0 && den  != 00 )
					{
						//o = (float)(180.0 + (180.0 / CV_PI) * atan( num / den ));
						o = 0.5f*(float)(atan( num / den ));

						if (den < 0) {
							o += num > 0 ? (float)CV_PI/2.0f : (float)-CV_PI/2.0f;;
						}
					}
					else if (den == 0 && num > 0)
						o = (float)CV_PI/4.0f;
					else if (den == 0 && num < 0)
						o = (float)-CV_PI/4.0f;
					//covered with else
					//else if (num == 0 && den > 0)
					//	o = 0;
					else if (num == 0 && den < 0)
						o = (float)-CV_PI/2.0f;
					else
						o = 0.0f;


				}
//printf("orientation innerhalb:  %f\n", o);
				(*it).setOrientation(o);
			} else break;  //already calculated
			it++;
		}//while
	}//else (contours.size() == 0)

}

template <class Attr> void DkBlobs<Attr>::calcBb() {

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}
	//some properties are calculated, insert the missing ones
	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcBb();
		//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	} else {

		typename vector<Attr>::iterator it;
		RotatedRect rect;
		Rect bb;
		it = props.begin();
		while( it != props.end()) {


			if (!(*it).calcBbRect()) {
				bb = boundingRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setBbRect(bb);
			} else break;  //already calculated
			it++;
		}//while
	}//else (contours.size() == 0)

}


template <class Attr> void DkBlobs<Attr>::calcMar() {

	//if not calculated, calculate the contours  for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	//some properties are calculated, insert the missing ones
	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcMar();
		//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	} else {

		typename vector<Attr>::iterator it;
		//RotatedRect rect;
		//Rect bb;
		DkRect rect;
		it = props.begin();
		while( it != props.end()) {

			if (!(*it).calcMinAreaRect()) {
				rect = minAreaRect(Mat(contours[(*it).getContourIdx()]));
				(*it).setMinAreaRect(rect);
			} else break;  //already calculated
			it++;
		}//while
	}//else (contours.size() == 0)
}

template <class Attr> void DkBlobs<Attr>::calculateMedianWordDims(const Mat img, double angle, float* medianWidth, float* medianHeight) {
	
	Mat blobImg = img.clone();

	DkBlobs<DkAttr> blobs(blobImg);
	blobs.calcMar();
	blobs.calcArea();
	blobs.imgFilterArea(50);

	std::vector<DkAttr> attrs = blobs.getProps();

	double normAngle = DkMath::normAngleRad(angle);

	std::list<float> wHeights;
	std::list<float> wWidths;
	for (unsigned int idx = 0; idx < attrs.size(); idx++) {
		double normMarAngle = DkMath::normAngleRad(attrs[idx].getMinAreaRect().angle);
		if (abs(normMarAngle - normAngle) < CV_PI*0.5) {
			wHeights.push_back(attrs[idx].getMinAreaRect().size.width);
			wWidths.push_back(attrs[idx].getMinAreaRect().size.height);
		}
		else {
			wHeights.push_back(attrs[idx].getMinAreaRect().size.height);
			wWidths.push_back(attrs[idx].getMinAreaRect().size.width);
		}
	}
	
	*medianHeight = (float)DkMath::statMoment(&wHeights, 0.5, false);
	*medianWidth = (float)DkMath::statMoment(&wWidths, 0.5, false);
}



template <class ColorAttr> void DkColorBlobs<ColorAttr>::calcMeanColors(Mat rgb, Mat magn) {

	Mat imgPatch;
	Mat imgMagnPatch;


//if props not calculated, calculate it
	this->calcBb();
	this->calcArea();

	//printf("props.size:  %i\n", (int)props.size());

	iout << "calculate mean color: # of blobs: " << this->props.size() << dkendl;

	if (this->props.size() != 0) {
		typename vector<ColorAttr>::iterator it;

		DkBox bb;
		it = this->props.begin();
		DkVector offset;
		Mat tmpMask;
		Scalar color(255);
		Scalar mean;
		Scalar magnS;
		Scalar std;
		//hist.~Mat();
		satHist = Mat(1, bins, CV_32FC1);
		satHist = 0.0f;
		float *ptrSatHist = satHist.ptr<float>(0);

		hueHist = Mat(1, bins, CV_32FC1);
		hueHist = 0.0f;
		float *ptrHueHist = hueHist.ptr<float>(0);

		valHist = Mat(1, 100, CV_32FC1);
		valHist = 0.0f;
		float *ptrValHist = valHist.ptr<float>(0);

		valColHist = Mat(1, bins, CV_32FC1);
		valColHist = 0.0f;
		float *ptrValColHist = valColHist.ptr<float>(0);


		//printf("blobs:  %i\n", (int)props.size());

		while( it != this->props.end()) {

			bb = (*it).getBbRect();
			offset = bb.uc * -1.0f;

			tmpMask.create(bb.getHeight(), bb.getWidth(), CV_8UC1);
			tmpMask.setTo(0);

			drawContours(tmpMask, this->contours, (*it).getContourIdx(), color, CV_FILLED, 8, this->hierarchy, 1, offset.getCvPoint());
			
			imgPatch = rgb(bb.getCvRect()).clone();

//DkIP::imwrite("patch1701.png", imgPatch);
//DkIP::imwrite("patch_mask1701.png", tmpMask);

			// diem
			if (imgPatch.depth() != CV_32F)
				imgPatch.convertTo(imgPatch, CV_32F);

			if (!magn.empty()) {

				imgMagnPatch = magn(bb.getCvRect()).clone();
				//normalize(imgMagnPatch, imgMagnPatch, 1.0f, 0.0f, NORM_MINMAX);
				imgMagnPatch = imgMagnPatch*-1.0f + 1.0f;
				meanStdDev(imgMagnPatch, magnS, std, tmpMask);
//DkIP::imwrite("magnpatch1712.png", imgMagnPatch, true);
//DkIP::imwrite("maskPatch1713.png", tmpMask);
//DkUtils::getMatInfo(imgMagnPatch, "magnppatch");
//DkUtils::getMatInfo(imgMagnPatch, "magnpatch");
//DkUtils::getMatInfo(imgPatch, "imgpatch");
				std::vector<Mat> rgbCh;
				split(imgPatch, rgbCh);
				rgbCh[0] = rgbCh[0].mul(imgMagnPatch);
				rgbCh[1] = rgbCh[1].mul(imgMagnPatch);
				rgbCh[2] = rgbCh[2].mul(imgMagnPatch);
				merge(rgbCh, imgPatch);
				//imgPatch = imgPatch.mul(imgMagnPatch);
			}
			meanStdDev(imgPatch, mean, std, tmpMask);
			
			unsigned char rTmp, gTmp, bTmp;
			DkVector3 ihlsVal;
			if (!magn.empty()) {
				rTmp = (unsigned char)(mean[0]/magnS[0]);
				gTmp = (unsigned char)(mean[1]/magnS[0]);
				bTmp = (unsigned char)(mean[2]/magnS[0]);
				//printf("test:  %i  %i %i", rTmp, gTmp, bTmp);
				ihlsVal = DkIP::convertRGBtoIHLS(DkVector3((float)(mean[0]/magnS[0]),(float)(mean[1]/magnS[0]),(float)(mean[2]/magnS[0])));
			}
			else {
				rTmp = (unsigned char)(mean[0]);
				gTmp = (unsigned char)(mean[1]);
				bTmp = (unsigned char)(mean[2]);

				ihlsVal = DkIP::convertRGBtoIHLS(DkVector3((float)mean[0], (float)mean[1], (float)mean[2]));
			}

			(*it).setRGB(rTmp,gTmp,bTmp);
			(*it).setStdRGB((unsigned char)std[0],(unsigned char)std[1],(unsigned char)std[2]);

			(*it).center = bb.center();

			(*it).h = ihlsVal.h;
			(*it).l = ihlsVal.l;
			(*it).s = ihlsVal.s;
		
			//DkIP::imwrite("patch.tif", imgPatch);
			//DkIP::imwrite("mask.tif", tmpMask);
			//printf("patch: %s", DkUtils::getMatInfo(imgPatch).c_str());
			//printf("mask: %s", DkUtils::getMatInfo(tmpMask).c_str());

			imgPatch = rgb(bb.getCvRect()).clone();

			
			//DkIP::imwrite("colorpatch.png", imgPatch);

			// diem
			if (imgPatch.depth() != CV_32F)
				imgPatch.convertTo(imgPatch, CV_32F);

			tmpMask = DkIP::fastErodeImage(tmpMask, 3);

			//DkIP::imwrite("colorpatchMask.png", tmpMask);

			Mat ihls = DkIP::convertRGBtoIHLS(imgPatch, tmpMask);
			std::vector<Mat> ihlsCh;
			split(ihls, ihlsCh);
			//saturation weighted hue histogram
			//printf("h: %f l: %f s: %f\n", ihlsVal.x, ihlsVal.y, ihlsVal.z);						
			for (int i=0; i<imgPatch.rows; i++) {
				float *ptrH = ihlsCh[0].ptr<float>(i);
				float *ptrS = ihlsCh[2].ptr<float>(i);
				float *ptrL = ihlsCh[1].ptr<float>(i);
				unsigned char *ptrMask = tmpMask.ptr<unsigned char>(i);
				for (int j=0; j<imgPatch.cols; j++) {
					//ihls channels
					if ((ptrH[j] != -1.0f) && (ptrMask[j] != 0)) {
						//threshold = 20 for S lt. paper (wahrnehmbare grenze für farbe/grauwert)
						//hier nach tests threshold default = 10
						//printf("S:  %f   L:  %f\n", ptrS[j], ptrL[j]);
						if ((ptrS[j] >= threshold) && (ptrL[j] >= threshold)) {  //decision color/gray for each pixel
						//if ((ihlsVal.z >= threshold) && (ihlsVal.y >= threshold)) { //decision color/gray blob based

							//printf("in Color S:  %f   L:  %f\n", ptrS[j], ptrL[j]);
							int bin = (int)((ptrH[j]/(float)CV_PI *180.0f)/360.0f * (float)bins);
							bin = bin > bins-1 ? bins-1 : bin;
							ptrHueHist[bin]++;
							ptrSatHist[bin] += ptrS[j];
							ptrValColHist[bin] += ptrL[j];

							//printf("H: %f  L:  %f  S:  %f\n", ptrH[j]/(float)CV_PI * 180.0f, ptrL[j], ptrS[j]);
						} else if ((ptrL[j] < 0.8f*255.0f) && (ptrS[j] < threshold)) {		//decision color/gray for each pixel  formerly ptrL[j] < 80 instead of 0.8f*255
						//} else if ((ihlsVal.y < 150) && (ihlsVal.z < threshold+10)){		//decision color/gray blob based
							//printf("H: %f  L:  %f  S:  %f\n", ptrH[j], ptrL[j], ptrS[j]);
							int bin = (int)(ptrL[j] / 255.0f * 100.0f);
							bin = bin > 99 ? 99 : bin;
							ptrValHist[bin]++;
						}
					}
				}
			}

			//Mat xxx;										//printf debug patch
			//tmpMask.convertTo(xxx, CV_32F);
			//DkUtils::printMatCmd(xxx, "patch");
			it++;

		}
	}
}



template <class DkCentAttr> class DkCentBlobs  : public DkBlobs<DkCentAttr> {

public:
	/**
	 * Default constructor.
	 * The images referenced has to be CV_8UC1. 0's are background, non-zero pixels are treated as foreground. img is the result of the segmentation.
	 * The mean color is calculated for all blobs.
	 * @param img The binary input image as unsigned char Mat with 1 channel (CV_8UC1) (segmented image)
	**/
	DkCentBlobs(Mat img) : DkBlobs<DkCentAttr>(img) {};
	/**
	 * Default destructor.
	**/
	~DkCentBlobs() {}; 
	
	/** 
	 * Calculates the centroid of every blob in the image
	 */
	void calcCenterOfMass();
	/** 
	 * Calculates the centroid of every blob in the image
	 */
	void calcCentroids();
	/**
	 * Draws the centroids in a given image.
	 * @param src the image in which the interest points are drawn.
	 * @return image with drawn interest points.
	 **/
	Mat drawCentroids(const Mat& src);
	/**
	 * Returns the centroids as a vector (no further Blob-Attributes)
	 * @return vector of centroids 
	 */
	vector<DkInterestPoint> getCentroids() {return centroids;};

	
	/**
	 * Filters those blobs, which are not within a threshold of the mean centroid
	 * changes the input image!
	 * @param thresh threshold
	 * @return image with filtered blobs (those which are removed in the original image) 
	 */
	Mat imFilterOutOfCent(float thresh =-1);

private: 
	/** 
	 * Calculates the centroid of a given coordinate list based on the min- and max-coordinate values 
	 * @param p Mat of DkVectors (coordinates from a contour)
	 * @return coordinates of the centroid (all x- (resp. y-) coordinates in one vector)
	 */
	DkInterestPoint calcCent(const Mat& p);

	vector<DkInterestPoint> centroids;
};

template <class DkCentAttr> void DkCentBlobs<DkCentAttr>::calcCentroids() {

	//if not calculated, calculate the contours  for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return;
	}

	//some properties are calculated, insert the missing ones
	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
		calcCentroids();
		//findContours(help, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//if not calculated, calculate all properties for all blobs
	//if (this->props.size() == 0) {
		//if ((this->contours.size() == 0) || (this->contours.size() != this->hierarchy.size())) {
		//	Mat help = this->bwImg.clone();   //findContours alters input image
		//	this->contours.clear();
		//	this->hierarchy.clear();
		//	findContours(help, this->contours, this->hierarchy, CV_RETR_EXTERNAL, this->approxMethod);
		//	this->size = (int)this->contours.size();
		//	if (this->size == 0) return;
		//}

		/*int idx = 0;
		for( ; idx >= 0; idx = hierarchy[idx][0]) {
			Attr a;
			a.setContourIdx(idx);
			a.setMinAreaRect(minAreaRect(contours[idx]));
			props.push_back(a);
		}*/
		//
		//vector<vector<Point> >::iterator it; // iterator over all contours of an image (list of "blobs")
		//it = this->contours.begin(); // first element/contour vector of first blob
		//while( it != this->contours.end()) {
		//	DkCentAttr a;
		//	DkInterestPoint ip; 
		//	ip = calcCent(Mat(*it, true));
		//	centroids.push_back(ip); // centroids list

		//	a.setCentroid(ip); // properties 
		//	this->props.push_back(a);

		//	it++;
		//}//while

	//some properties are calculated, insert the missing ones
	} else {
			vector<DkCentAttr>::iterator it; // iterator over all contours of an image (list of "blobs")
			it = this->props.begin(); // first element/contour vector of first blob
			while( it != this->props.end()) {
				DkInterestPoint ip; 
				ip = calcCent(Mat(contours[(*it).getContourIdx()], true));
				centroids.push_back(ip); // centroids list

				it->setCentroid(ip); 
				it++;
			}//while
	}//else
}
//http://en.wikipedia.org/wiki/Image_moments
template <class DkCentAttr> DkInterestPoint DkCentBlobs<DkCentAttr>::calcCent(const Mat& p) {
	Moments mom = cv::moments(p, false);
	DkInterestPoint centroid;
	centroid.vec.x = (float) (mom.m10 / mom.m00);
	centroid.vec.y = (float) (mom.m01 / mom.m00);
	centroid.val   = (float) (centroid.vec.x); 

	return centroid;
}
template <class DkCentAttr> Mat DkCentBlobs<DkCentAttr>::drawCentroids(const Mat& src) {
	Mat img = src.clone();

	typename vector<DkCentAttr>::iterator it; // iterator on properties
	it = this->props.begin();

	while (it != this->props.end()) {
		circle(img, (*it).getCentroid().vec.getCvPoint32f(), 3, Scalar(0, 126, 228), -1);
		it++;
	}
	return img;
}


template <class DkCentAttr> Mat DkCentBlobs<DkCentAttr>::imFilterOutOfCent(float thresh) {

	// now apply it to the image.... 
	Mat bImg = this->getBwImg();
	Mat returnMat(bImg.rows, bImg.cols, CV_8U);
	returnMat.setTo(0);

	int currArea = 0;
	Scalar color(0);

	//if not calculated, calculate the contours for all blobs
	if (centroids.size() == 0) {
		calcCentroids();
	}

	//if not calculated, calculate the contours for all blobs
	if (props.size() == 0) {
		calcContours();
		if (size == 0) return returnMat;
	}

	if ((contours.size() == 0) || (contours.size() != hierarchy.size())) {
		props.clear();
		contourIdxInPropVector.clear();
	} else {
		//some properties are calculated, insert the missing ones
		vector<DkCentAttr>::iterator it;



		/* find the approx y-center of the line by averaging the contour points, in order to exclude those 
		 * blobs which are out of center.
		 */
		float centMean  = std::accumulate(contours.begin(), contours.end(), 0, VectorAccumulation()); // accumulate the y-coordinates
		centMean = centMean / (float)std::accumulate(contours.begin(), contours.end(), 0, VectorCount()); // divide by  number of contour points
	
		if (thresh < 0)
			thresh = centMean/3;
		int numberOfDeleted = 0;
		it = props.begin();
		while( it != props.end()) {
			Mat tmp = this->getBwImg().clone(); 		
				vector<vector<Point>> ctrs; 
				ctrs.push_back(contours[it->getContourIdx()]);
				drawContours(tmp, ctrs, -1, Scalar(100), CV_FILLED, 8, noArray(), INT_MAX, Point(0,0)); //hierarchy
				imwrite("C:\\VSProjects\\test.png", tmp); 	
			// if the centroid is not within the boundaries (mean +- thresh), then remove it ... 
			if (it->getCentroid().vec.y < centMean - thresh || it->getCentroid().vec.y > centMean + thresh) {

				drawContours(returnMat, ctrs, -1, Scalar(255), CV_FILLED, 8);

				if (hierarchy[(*it).getContourIdx()][1] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][1]][0] = hierarchy[(*it).getContourIdx()][0];  //set successor of predecessor of idx to successor of idx
				if (hierarchy[(*it).getContourIdx()][0] != -1)
					hierarchy[hierarchy[(*it).getContourIdx()][0]][1] = hierarchy[(*it).getContourIdx()][1];	//set predecessor of successor of idx to predecessor of idx

				contours[(*it).getContourIdx()].clear();										//clear only contour points but not the element
				contourIdxInPropVector.erase(contourIdxInPropVector.find((*it).getContourIdx())); // erase entry in the allocation map
				it = props.erase(it);														//clear props
				numberOfDeleted++;
			} else {
				contourIdxInPropVector[(*it).getContourIdx()] = contourIdxInPropVector[(*it).getContourIdx()] - numberOfDeleted;
				++it;
			}
		}
		bImg.setTo(0);
		drawContours(bImg, contours, -1, Scalar(255), CV_FILLED, 8, hierarchy, INT_MAX, Point(0,0));
		return returnMat; 
	}

	return returnMat;
}

