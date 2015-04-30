/*******************************************************************************************************
 DkImageSource.h
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

#pragma once

#include "DkError.h"
#include "DkUtils.h"
#include "DkTimer.h"
#include "DkIP.h"
#include "DkMath.h"

using namespace cv;

/**
 * The Image Source class provides the source images.
 * Additionally basic computations
 * such as normalization, erodeMaskBoundary, grayValue image, ...
 **/
class DK_CORE_API DkImageSource {

public:
	/**
	 * Default constructor.
	 * @param rgb The color input image [0 255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1) [0 255].
	**/
	DkImageSource(Mat rgb, Mat mask);
	/**
	 * Default constructor.
	**/
	DkImageSource();

	DkImageSource(Mat rgb);
	
	/**
	 * Sets the input images
	 * @param rgb The color input image [0 255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1) [0 255].
	**/
	void setInput(Mat rgb, Mat mask);
	/**
	 * Sets the sigma for the erodeBoundary function of the mask.
	 * @param sigma sigma for the erodeBoundary function.
	**/
	void setSigmaErodeMask(float sigma);
	/**
	 * Returns the RGB image.
	 * @return rgb image 8UC3 [0 255].
	**/
	Mat getRgbImg();
	/**
	 * Returns the mask image.
	 * @return mask image 8UC1 [0 255].
	**/
	Mat getMask();
	/**
	 * Returns the normalized RGB image.
	 * @return rgb image 32FC3 [0 1].
	**/
	Mat getNormRgb32f();
	/**
	 * Returns the gray image.
	 * @return gray image 8UC1 [0 255].
	**/
	Mat getGrayImg();
	/**
	 * Returns the gray image as 32F.
	 * @return gray image 32FC1 [0 1].
	**/
	Mat getGrayImg32F();
	/**
	 * Returns the normalized gray image.
	 * @return gray image 32FC1 [0 1].
	**/
	Mat getNormGrayImg32F();
	/**
	 * Returns the mask.
	 * @return mask image 32FC1 [0 1].
	**/
	Mat getMask32F();
	/**
	 * Returns the eroded mask image.
	 * @return gray image 8UC1 [0 255].
	**/
	Mat getErodedMask();
	/**
	 * Returns the eroded mask image.
	 * @return gray image 32FC1 [0 1].
	**/
	Mat getErodedMask32F();

	/**
	 * Check if any module modified a source image.
	 * The input class must be initialized at the beginning
	 * with cloned images. If any image was modified by a
	 * module, a warning is printed.
	 * @param imgsCmp a class with source images which are definitely
	 * not called by any function.
	 **/
	void checkConsistency(DkImageSource *imgsCmp);
	/**
	 * Proves if the input has been defined.
	 * @return True if the input has not been defined, zero otherwise.
	**/
	bool empty();
	/**
	 * Default destructor.
	**/
	~DkImageSource() {};
	/**
	 * Releases all default images
	 **/
	void release();

	DkBox getMaskBox() {
		return maskBox;
	};

private:

	void estimateMask();
	void checkInput();
	void init();

	DkBox maskBox;

	Mat rgb;		//rgb input image
	Mat mask;		//the mask image
	Mat normRgb32F;
	Mat grayImg;
	Mat grayImg32F;
	Mat normGrayImg32F;
	Mat mask32F;
	Mat maskEroded;
	Mat maskEroded32F;

	float sigmaErodeBoundary;	//sigma for the 

};
