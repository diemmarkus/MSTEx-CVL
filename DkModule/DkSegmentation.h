/*******************************************************************************************************
 DkSegmentation.h
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
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

#include "DkModuleInclude.h"
#include "DkError.h"
#include "DkUtils.h"
#include "DkTimer.h"
#include "DkIP.h"
#include "DkImageSource.h"
#include "DkModule.h"
//#include "DkDogDetector.h"
#include <opencv/ml.h>
#include <math.h>

using namespace cv;

#define DK_ERODED_MASK_SIZE 3*6
enum scalemode{DK_SEGSU = 0, DK_SEGSUFGD, DK_SEGFGD, DK_SEGSAT, DK_IPK};


class CDistHistList
{
private:
	int count;
	int elemCount;
	int *pDiffs;
	float *pLocInt;

public:
	CDistHistList(int count)
	{
		this->count = count;
		this->elemCount = 0;
		this->pDiffs  = 0;
		this->pLocInt = 0;
		this->pDiffs  = new int[this->count];
		//this->pLocInt = new float[this->count];
	}

	~CDistHistList()
	{
		if (this->pDiffs)
			delete[] this->pDiffs;
		if (this->pLocInt)
			delete[] this->pLocInt;
	}

	void add(int a/*, float b*/)
	{
		this->pDiffs [this->elemCount] = a;
		//this->pLocInt[this->elemCount] = b;
		this->elemCount++;
	}

	int getElemCount()
	{
		return this->elemCount;
	}

	int operator[](int i)
	{
		return this->pDiffs[i];
	}

	void Reset()
	{
		this->elemCount = 0;
	}
};


//  call
//	segM.getFilteredSegmented(10);
// the image chosen is calculated and filtered
// if segImg is needed instead of getSegmented (e.g. DkRuling) call
//  segM.filterSegImg(10);
/**
 * Interface class for Binarization classes
 * contrastImg, binary contrastImg, FgdEst and setInput functions are necessary for Scale Space Segmentation
  **/
class DK_MODULE_API DkSegmentationBase : public DkModule {

public:

	/**
	 * Default constructor.
	**/
	DkSegmentationBase() {erodeMaskSize = DK_ERODED_MASK_SIZE;};
	/**
	 * Default constructor.
	 * @param imgs the image source class
	**/
	DkSegmentationBase(DkImageSource *imgs) : DkModule(imgs) {erodeMaskSize = DK_ERODED_MASK_SIZE;};
	/**
	 * Default destructor.
	**/
	virtual ~DkSegmentationBase() {};
	/**
	 * Calculates the binarized image.
	 * @return The segmented image (CV_8UC1) [0..255].
	**/
	virtual Mat getSegmented() = 0;
	/**
	 * Returns the threshold image.
	 * @return The threshold image (CV_8UC1) [0..255].
	**/
	virtual Mat getThrImg() = 0;
	/**
	 * Calculates the contrast image.
	 * @return The contrast image (CV_32FC1) [0..1].
	**/
	virtual Mat getContrastImg() = 0;
	/**
	 * Calculates the binarized contrast image.
	 * @return The binarized contrast image (CV_8UC1) [0..255].
	**/
	virtual Mat getBinContrastImg() = 0;
	/**
	 * Sets the grayvalue and the mask input image (only needed
	 * for scale space segmentation
	 * @param g the grayvalue input image (CV_8UC1) [0..255].
	 * @param m the mask (CV_8UC1) [0..255].
	 **/ 
	virtual void setInputG(Mat g, Mat m);
	/**
	 * Sets the color and the mask input image (only needed
	 * for scale space segmentation
	 * @param g the grayvalue input image (CV_8UC3) [0..255].
	 * @param m the mask (CV_8UC1) [0..255].
	 **/ 
	virtual void setInputRGB(Mat rgb, Mat m);
	/**
	 * Returns the foreground estimation
	 * @return foreground estimation (CV_8UC1) [0..255].
	 **/ 
	virtual Mat getFgdEstImg() = 0;
	/**
	 * Enhances the grayvalue image by combining the saturation image with the grayvalue image
	 * @return the enhanced image (CV_8UC1) [0..255].
	 **/ 
	virtual Mat enhanceImg() = 0;
	/**
	 * Returns the mean contrast of the image.
	 * @return mean contrast.
	 **/ 
	virtual float getMeanContrast() = 0;
	/**
	 * Sets the size for the erosion of the mask boundary (default = 18).
	 * @param erodeMaskSize filter size for the erosion of the boundary to remove artefacts at the border.
	 **/
	void setRemoveBorderSize(int erodeMaskSize);
	/**
	 * Returns the size for the erosion of the mask boundary (default = 18).
	 * @return erodeMaskSize filter size for the erosion of the boundary to remove artifacts at the border.
	 **/
	int getRemoveBorderSize(void) {return erodeMaskSize;};

	bool medianFilter;	/**< median filter of segmented image that speeds up contour computations**/

protected:
	int erodeMaskSize;									//size for the boundary erosion

	Mat rgbImg;											//the rgb input image [0 255]
	Mat grayImg;										//the input img as gray value image [0 255]
	Mat mask;											//the mask image [0 255]
	Mat segImg;											//the segmented image
};

/**
 * The class binarize a grayvalue image. The segmentation algorithm is based on
 * "Binarization of Historical Document Images Using Local Maximum and Minimum", Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010.
 **/
class DK_MODULE_API DkSegmentationSu : public DkSegmentationBase {

public:
	/**
	 * Default constructor.
	**/
	DkSegmentationSu();
	/**
	 * Default constructor.
	 * @param rgb The color input image [0..255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1).
	**/
	DkSegmentationSu(Mat rgb, Mat mask);
	/**
	 * Default constructor.
	 * @param imgs the image source class
	**/
	DkSegmentationSu(DkImageSource *imgs);
	/**
	 * Calculates the binarized image based on "Binarization of Historical Document Images Using the Local Maximum and Minimum"
	 * Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010 without improvement.
	 * @return The segmented image (CV_8UC1) [0..255].
	**/
	virtual Mat getSegmented();

	void compute();

	/**
	 * Filters the result of the approach of B Su, S Lu and CL Tan according the blob size.
	 * @param size The maximal Size of a blob in pixel.
	**/
	void filterSegImg(int size);
	static cv::Mat filterSegImgAuto(const cv::Mat& segImg);
		/**
	 * Converts all parameters and results to a string.
	 * @return a string containing the values of the module's
	 * results and parameters
	 **/
	virtual std::string toString() const;
	/**
	 * Clears the class (all debug images, etc).
	**/
	virtual void release();
	/**
	 * Returns the threshold image (debug image).
	 * @return Mat threshold image.
	**/
	virtual Mat getThrImg();
	/**
	 * Returns the contrast image (debug image).
	 * @return Mat contrast image.
	**/
	virtual Mat getContrastImg();
	/**
	 * Returns the thresholded contrast image (debug image).
	 * @return Mat binary contrast image.
	**/
	virtual Mat getBinContrastImg();
	/**
	 * Returns the estimated foreground.
	 * @return Mat binary contrast image.
	**/
	virtual Mat getFgdEstImg() {return Mat();};
	/**
	 * Enhances the grayvalue image by combining the saturation image with the grayvalue image
	 * @return the enhanced image (CV_8UC1) [0..255].
	 **/
	virtual Mat enhanceImg() {return Mat();};
	/**
	 * Proves if the input has been defined.
	 * @return True if the input has not been defined, zero otherwise.
	**/
	bool empty();
	/**
	 * Saves all debug images of the class.
	 * @param path The path where the images are saved
	 * @param filename The filename of the current image
	**/
	virtual void saveDebugImg(Mat img, std::string path, std::string filename);
	/**
	 * Release all debug images of the class.
	**/
	virtual void clearDebugImg();
	/**
	 * Returns the mean contrast of the image.
	 * @return mean contrast.
	 **/ 
	virtual float getMeanContrast();
	/**
	 * Default destructor.
	**/
	virtual ~DkSegmentationSu() {};

protected:

	void convertInputData();							//calculate grayvalue image and eroded mask
	void computeSu(Mat &resultSegImg);								//computes the segmentation based on Local Maximum and Minimum (Su, Lu and Tan)
	virtual float getStrokeWidth(Mat contrastImg);				//estimates the stroke width
	Mat computeContrastImg();							//computes the contrast image as return value
	Mat computeBinaryContrastImg(Mat contrast);					//computes the binarized contrastImage
	void computeThrImg(Mat grayImg32F, Mat binContrast, Mat &thresholdImg, Mat &thresholdContrastPxImg);					//compute threshold image
	void computeDistHist(Mat src, std::list<int> *maxDiffList, std::list<float> *localIntensity, float gSigma); //distance histogram
	void computeDistHist(Mat src, CDistHistList *pDHL, CDistHistList *pLocalDHL, float gSigma); //distance histogram
	virtual float thresholdVal(float *mean, float *std);
	virtual float contrastVal(unsigned char* maxVal, unsigned char * minVal);
	virtual void calcFilterParams(int &filterS, int &Nm);

	void checkInput() const;
	virtual void init();

	float strokeW;										//the estimated stroke width
	
	int segmentedPx;									//the amount of segmented pixel
	int maskPx;											//the amount of pixel of the snippet area

	//debug Images
	Mat contrastImg;									//contrast image [0.0f 1.0f]
	Mat binContrastImg;									//binContrastImg; [0.0f 1.0f]
	Mat thrImg;											//the threshold image
	Mat erodedMask;
};

/**
 * The class binarize a grayvalue image. The segmentation algorithm is based on
 * "Binarization of Historical Document Images Using Local Maximum and Minimum", Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010.
 * In contrast to DkSegmentationSu the ContrastImg is adapted and the strokeWidth is constant.
 **/
class DkSegmentationSuIpk : public DkSegmentationSu {
public:
	/**
	 * Default constructor.
	**/
	DkSegmentationSuIpk() : DkSegmentationSu() {init();};
	/**
	 * Default constructor.
	 * @param rgb The color input image [0..255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1).
	**/
	DkSegmentationSuIpk(Mat rgb, Mat mask) : DkSegmentationSu(rgb,mask) {init();};
	/**
	 * Default constructor using the imagesource class.
	 * @param imgs all the input images and pre calculated images from DkImageSource class.
	**/
	DkSegmentationSuIpk(DkImageSource *imgs) :  DkSegmentationSu(imgs) {init();};
	/**
	 * Calculates the binarized image.
	 * @return The segmented image (CV_8UC1) [0..255].
	**/
	virtual Mat getSegmented();

	/**
	 * Default destructor.
	**/
	virtual ~DkSegmentationSuIpk() {};

protected:
	//TODO: move to protected (untested)
	void computeSuIpk(Mat &resultSegImg);
	virtual void init();
	virtual float contrastVal(unsigned char* maxVal, unsigned char * minVal);
	virtual float getStrokeWidth(Mat contrastImg);
	virtual void calcFilterParams(int &filterS, int &Nm);
};

/**
 * The class binarize a rgb color image. To make the algorithm robust against noise,
 * the foreground is estimated and the image is weighted with the foreground.
 * (Foreground estimation is performed by a Mean Filter with a size of 32x32 px)
 **/
class DkSegmentationFgdIpk : public DkSegmentationSuIpk {
public:
	/**
	 * Default constructor.
	**/
	DkSegmentationFgdIpk() : DkSegmentationSuIpk() {init();};
	/**
	 * Default constructor.
	 * @param rgb The color input image [0..255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1).
	**/
	DkSegmentationFgdIpk(Mat rgb, Mat mask) : DkSegmentationSuIpk(rgb,mask) {init();};
	/**
	 * Default constructor using the imagesource class.
	 * @param imgs all the input images and pre calculated images from DkImageSource class.
	**/
	DkSegmentationFgdIpk(DkImageSource *imgs) :  DkSegmentationSuIpk(imgs) {init();};
	/**
	 * Calculates the binarized image based on "Binarization of Historical Document Images Using the Local Maximum and Minimum"
	 * Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010 without improvement.
	 * @return The segmented image (CV_8UC1) [0..255].
	**/
	virtual Mat getSegmented();
	/**
	 * Converts all parameters and results to a string.
	 * @return a string containing the values of the module's
	 * results and parameters
	 **/
	virtual std::string toString() const;
	/**
	 * Clears the class (all debug images, etc).
	**/
	virtual void release();
	/**
	 * Sets the filter size for the foreground estimation (default: 32).
	 * @param s The filter size for the foreground estimation.
	**/
	void setfgdEstFilterSize(int s);
	/**
	 * Sets the slope of the sigmoid function to supress noise (default: 15).
	 * @param s The filter size for the foreground estimation.
	 **/
	void setSigmSlope(float s);
	/**
	 * Release alle debug images
	 **/ 
	virtual void clearDebugImg();
	/**
	 * Returns the estimated foreground.
	 * @return Mat binary contrast image.
	**/
	virtual Mat getFgdEstImg();
	
	/**
	 * Standard Destructor
	 **/ 
	virtual ~DkSegmentationFgdIpk() {};

protected:
	void computeFgd(Mat &resultSegImg);
	Mat computeMeanFgdEst(Mat grayImg32F);
	void computeConfidence();
	virtual void init();
	virtual void weightFunction(Mat &grayImg, Mat &thrImg, Mat mask);

	int fgdEstFilterSize;								//the filter size for the foreground estimation
	float sigmSlope;
	Mat fgdEstImg;
	Scalar meanContrast;
	Scalar stdContrast;
	float confidence;
};

/**
 * The class binarize a rgb color image. In addition to the foreground estimation the grayvalue image
 * is enhanced by the saturation channel to segment also low saturated colors.
 * The decision if the segmented image of the enhanced grayvalue image or of the original grayvalue image is choosen,
 * is based on the mean contrast.
 **/
class DkSegmentationSatIpk : public DkSegmentationFgdIpk {
public:
	/**
	 * Default constructor.
	**/
	DkSegmentationSatIpk() : DkSegmentationFgdIpk() {init();};
	/**
	 * Default constructor.
	 * @param rgb The color input image [0..255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1).
	**/
	DkSegmentationSatIpk(Mat rgb, Mat mask) : DkSegmentationFgdIpk(rgb,mask) {init();};
	/**
	 * Default constructor using the imagesource class.
	 * @param imgs all the input images and pre calculated images from DkImageSource class.
	**/
	DkSegmentationSatIpk(DkImageSource *imgs) :  DkSegmentationFgdIpk(imgs) {init();};

	/**
	 * Calculates the binarized image based on "Binarization of Historical Document Images Using the Local Maximum and Minimum"
	 * Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010 without improvement.
	 * @return The segmented image (CV_8UC1) [0..255].
	**/
	virtual Mat getSegmented();
	/**
	 * Returns the segmented image filtered according size.
	 * @param size the filter size
	 * @return the segmented image (CV_8UC1) [0..255].
	 **/ 
	Mat getFilteredSegmented(int size);
	/**
	 * Returns the segmented image calculated by DkSegmentationFgdIpk.
	 * (without color enhancement)
	 * @return the segmented image (CV_8UC1) [0..255].
	 **/ 
	Mat getSegSuFgdImg();
	/**
	 * Returns the segmented image with enhanced color image (regardless of
	 * the mean contrast and the confidence threshold.
	 * @return the segmented image (CV_8UC1) [0..255].
	 **/ 
	Mat getSegSatImg();
	/**
	 * Sets the sensitivity for the color enhancement. A value of 0.0 means no enhancement.
	 * A value closer to 1.0 boosts the s channel. A good choice is a value of 0.5 to 1.0.
	 * (Smaller values can also produce more noise). If the sensitivity is set to 1.0f, 
	 * the histogram stretching is not computed.
	 * @param s The filter size for the foreground estimation.
	 **/
	void setSensitivity(float sens);
	/**
	 * Sets the threshold which is used to decide if the result ist the color enhanced or the conventional
	 * segmented image (obtained by getSegmented). 0.1 is a good choice.
	 * @param th The threshold value (initial 0.1f).
	 **/
	void setConfidenceThresh(float th);
	/**
	 * Calculates the confidence for the segmented image (based on LocalMinMaxSat).
	 * The confidence is based on the fact that on a fully described sheet of paper
	 * the text is approx. 20% of the area. The confidence is weighted with the contrast.
	 * @return the confidence of the segmentation
	 **/
	float getConfidence();

	/**
	 * Returns the enhanced grayvalue image.
	 * @return enhanced grayvalue image.
	 **/ 
	virtual Mat enhanceImg();
	/**
	 * Returns the mean Contrast of the image.
	 * @return mean contrast.
	 **/ 
	virtual float getMeanContrast();

	/**
	 * Filters the segmented image (based on the enhanced grayvalue image).
	 * @param size the filter size
	 **/ 
	void filterSatImg(int size);
	/**
	 * Converts all parameters and results to a string.
	 * @return a string containing the values of the module's
	 * results and parameters
	 **/
	virtual std::string toString() const;
	/**
	 * Saves all debug images of the class.
	 * @param path The path where the images are saved
	 * @param filename The filename of the current image
	**/
	virtual void saveDebugImg(Mat img, std::string path, std::string filename);
	/**
	 * Clears the class (all debug images, etc).
	**/
	virtual void release();
	/**
	 * Enhances the grayvalue image by the combination with the H channel of the IHLS color space.
	 * @param satImg the color image
	 * @return the enhanced grayval image
	**/
	Mat enhanceSatImg(Mat satImg);				//enhances the satImg

	/**
	 * Standard Destructor.
	 **/ 
	virtual ~DkSegmentationSatIpk() {};

protected:
	void computeSat(bool computeAlways=false);
	Mat computeSatImg();						//computes a white balanced saturation image
	virtual void init();

	Mat segSatImg;										//the segmented image with combined saturation channel

	float sensitivity;									//the sensitivity for color boost
	float confidenceThreshold;							//the confidence threshold used by getSegmented();

	int segmentedPx;									//the amount of segmented pixel
	int maskPx;											//the amount of pixel of the snippet area
};


/**
 * The class binarize a grayvalue image. Only for DIBCO test reasons.
 * (getStrokeWidth, filterParams, thresholdVal are different)
 **/
class DkSegmentationDibcoTest : public DkSegmentationSuIpk {
public:
	/**
	 * Default constructor.
	**/
	DkSegmentationDibcoTest() : DkSegmentationSuIpk() {};
	/**
	 * Default constructor.
	 * @param rgb The color input image [0..255], (CV_8UC3).
	 * @param mask The mask image (CV_8UC1).
	**/
	DkSegmentationDibcoTest(Mat rgb, Mat mask) : DkSegmentationSuIpk(rgb,mask) {};
	/**
	 * Default constructor using the imagesource class.
	 * @param imgs all the input images and pre calculated images from DkImageSource class.
	**/
	DkSegmentationDibcoTest(DkImageSource *imgs) :  DkSegmentationSuIpk(imgs) {};

	virtual Mat getSegmented();
	/**
	 * Standard Destructor
	 **/ 
	virtual ~DkSegmentationDibcoTest() {};

protected:
	//virtual float contrastVal(unsigned char* maxVal, unsigned char * minVal);
	void computeSuDibcoTest(Mat &resultSegImg);
	virtual float getStrokeWidth(Mat contrastImg);
	virtual void calcFilterParams(int &filterS, int &Nm);
	virtual float thresholdVal(float *mean, float *std);
};





//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
// old class
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------
///**
// * The class binarize a RGB color image. The segmentation algorithm is based on
// * "Binarization of Historical Document Images Using Local Maximum and Minimum", Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010.
// * In addition a color segmentation is performed.
// **/
//class DkSegmentation : public DkModule  {
//
//public:
//	/**
//	 * Default constructor.
//	**/
//	DkSegmentation();
//	/**
//	 * Default constructor.
//	 * @param rgb The color input image [0..255], (CV_8UC3).
//	 * @param mask The mask image (CV_8UC1).
//	**/
//	DkSegmentation(Mat rgb, Mat mask);
//	/**
//	 * Default constructor using the imagesource class.
//	 * @param imgs all the input images and pre calculated images from DkImageSource class.
//	**/
//	DkSegmentation(DkImageSource *imgs);
//	/**
//	 * Calculates the binarized image based on "Binarization of Historical Document Images Using the Local Maximum and Minimum"
//	 * Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010 without improvement.
//	 * @return The segmented image (CV_8UC1) [0..255].
//	**/
//	Mat getSegmentedLocalMinMaxSu();
//	/**
//	 * Calculates the binarized image based on "Binarization of Historical Document Images Using the Local Maximum and Minimum"
//	 * Bolan Su, Shijian Lu and Chew Lim Tan, DAS 2010.
//	 * Due to the improvement the algorithm is less sensitive to noise
//	 * @return The segmented image (CV_8UC1) [0..255].
//	**/
//	Mat getSegmentedLocalMinMax();
//	/**
//	 * Calculates the binarized image based on B Su, S Lu and CL Tan (DAS 2010) where the grayvalue image is combined
//	 * with the Saturation channel of the IHLS color space.
//	 * In addition the Foreground-estimation as based on
//	 * an eroded and thresholded image (no contrast image is used).
//	 * Due to the improvement the algorithm is less sensitive to noise and segments also faded out ink.
//	 * !WARNING! getSegmented calculates always the segSatImg, regardless of the confidence. (getSegmented(), or filterMinMaxSat()
//	 * skips the computation if the meanContrast Value is below a certain threshold.
//	 * @return The segmented image (CV_8UC1) [0..255].
//	**/
//	Mat getSegmentedLocalMinMaxSat();
//	/**
//	 * Calculates the segmented images based on LocalMinMax and LocalMinMaxSat, and returns last one if the confidence
//	 * is higher than threshold otherwise the LocalMinMax segmented image. 
//	 * confidence threshold is initialized with 0.1 which is a good choice.
//	 * getSegmented(), or filterMinMaxSat() skips the computation if the meanContrast Value is below a certain threshold.
//	 * @return The segmented image (CV_8UC1) [0..255].
//	**/
//	Mat getSegmented();
//
//	Mat getSegmentedScaleSu();
//
//	Mat getSegmentedScaleSuFgd();
//
//	Mat getSegmentedScaleFgd();
//
//	Mat getSegmentedScaleSat();
//
//	/**
//	 * Filters the result of the approach of B Su, S Lu and CL Tan according the blob size.
//	 * @param size The maximal Size of a blob in pixel.
//	**/
//	void filterMinMax(int size);
//	/**
//	 * Filters the result of the approach of B Su, S Lu and CL Tan in combination with the saturation according the blob size.
//	 * getSegmented(), or filterMinMaxSat() skips the computation if the meanContrast Value is below a certain threshold.
//	 * @param size The maximal Size of a blob in pixel.
//	**/
//	void filterMinMaxSat(int size);
//
//	/**
//	 * Filters the result of the approach of B Su, S Lu and CL Tan in combination with the scale-space approach according to the blob size.
//	 * if getSegmentedScaleSu() was called before, the su image is filtered, otherwise the getSegmentedScaleFgd() image is filtered.
//	 * @param size The maximal Size of a blob in pixel.
//	**/
//	void filterMinMaxScale(int size);
//
//	/**
//	 * Filters the result of the approach of B Su, S Lu and CL Tan in combination with the scale-space and saturation approach according to the blob size.
//	 * @param size The maximal Size of a blob in pixel.
//	**/
//	void filterMinMaxScaleSat(int size);
//
//
//	/**
//	 * Converts all parameters and results to a string.
//	 * @return a string containing the values of the module's
//	 * results and parameters
//	 **/
//	std::string toString();
//	///**
//	// * Returns the filter size (according the blob size) of the filtered image. This size is set by
//	// * filterMinMax and filterMinMaxSat
//	// * @return filtersize, otherwise -1 if the image has not been filtered.
//	//**/
//	//int filterSize();
//	/**
//	 * Clears the class (all debug images, etc).
//	**/
//	void release();
//	/**
//	 * Returns the threshold image (debug image).
//	 * @return Mat threshold image.
//	**/
//	Mat getThrImg();
//	/**
//	 * Returns the contrast image (debug image).
//	 * @return Mat contrast image.
//	**/
//	Mat getContrastImg();
//	/**
//	 * Returns the Foreground Estimation image (debug image).
//	 * @return Mat foreground estimation image.
//	**/
//	Mat getFgdEstImg();
//	/**
//	 * Returns the eroded mask (debug image).
//	 * @return Mat eroded mask.
//	**/
//	Mat getErodedMask();
//	/**
//	 * Proves if the input has been defined.
//	 * @return True if the input has not been defined, zero otherwise.
//	**/
//	bool empty();
//
//	/**
//	 * Saves all debug images of the class.
//	 * @param path The path where the images are saved
//	 * @param filename The filename of the current image
//	**/
//	void saveDebugImg(Mat img, std::string path, std::string filename);
//
//	/**
//	 * Sets the filter size for the foreground estimation (default: 32).
//	 * @param s The filter size for the foreground estimation.
//	**/
//	void setfgdEstFilterSize(int s);
//	/**
//	 * Sets the slope of the sigmoid function to supress noise (default: 15).
//	 * @param s The filter size for the foreground estimation.
//	 **/
//	void setSigmSlope(float s);
//	/**
//	 * Sets the sensitivity for the color enhancement. A value of 0.0 means no enhancement.
//	 * A value closer to 1.0 boosts the s channel. A good choice is a value of 0.5 to 1.0.
//	 * (Smaller values can also produce more noise).
//	 * @param s The filter size for the foreground estimation.
//	 **/
//	void setSensitivity(float sens);
//	/**
//	 * Sets the threshold which is used to decide if the result ist the color enhanced or the conventional
//	 * segmented image (obtained by getSegmented). 0.1 is a good choice.
//	 * @param th The threshold value (initial 0.1f).
//	 **/
//	void setConfidenceThresh(float th);
//	/**
//	 * Calculates the confidence for the segmented image (based on LocalMinMaxSat).
//	 * The confidence is based on the fact that on a fully described sheet of paper
//	 * the text is approx. 20% of the area. The confidence is weighted with the contrast.
//	 * @return the confidence of the segmentation
//	 **/
//	float getConfidence();
//	/**
//	 * Sets the size for the erosion of the mask boundary (default = 18).
//	 * @param erodeMaskSize filter size for the erosion of the boundary to remove artefacts at the border.
//	 **/
//	void setRemoveBorderSize(int erodeMaskSize);
//	/**
//	 * Default destructor.
//	**/
//	~DkSegmentation() {};
//
//	//Mat getSatEnhImg() {return satEnhImg;};
//
//private:
//	void convertInputData();							//calculate grayvalue image and eroded mask
//	void clearDebugImg();
//	void computeLocalMinMaxSu(Mat &resultSegImg);								//computes the segmentation based on Local Maximum and Minimum (Su, Lu and Tan)
//	void computeLocalMinMaxSeg(Mat &resultSegImg);						//in Addition to Su, Lu, Tan weighted with a foreground estimation and sigmoid slope (noise)
//	void computeLocalMinMaxSatSeg(bool computeAlways=false);					//computes the segmentation based on Local Maximum and Minimum combined with Saturation
//	float getStrokeWidth(Mat contrastImg);				//estimates the stroke width
//	Mat computeContrastImg();							//computes the contrast image as return value
//	Mat computeBinaryContrastImg(Mat contrast);					//computes the binarized contrastImage
//	void computeFgdEst(Mat grayImg, Mat mask);			//computes the foreground estimation based on an erosion
//	Mat computeMeanFgdEst(Mat grayImg32F);		//computes the foreground estimation based on the mean value of the gray image
//	void computeThrImg(Mat grayImg32F, Mat binContrast, Mat &thresholdImg, Mat &thresholdContrastPxImg);					//compute threshold image
//	void computeDistHist(Mat src, std::list<int> *maxDiffList, std::list<float> *localIntensity, float gSigma); //distance histogram
//	void computeDistHist(Mat src, CDistHistList *pDHL, CDistHistList *pLocalDHL, float gSigma); //distance histogram
//	Mat computeSatImg();						//computes a white balanced saturation image
//	Mat enhanceSatImg(Mat satImg);				//enhances the satImg
//
//	Mat computeThreshScale(Mat grayImgTmp, Mat eMask, Mat parentImg, int mode=DK_SEGFGD, Mat weightImg=Mat(), Mat segFgd=Mat());		//functions for the scale space approach
//	Mat computeThrImgScale(Mat thrImg, Mat parentImg, Mat binThrImg);										//functions for the scale space approach
//	Mat computeLocalMinMaxScale(int mode);
//
//	void checkInput();
//	void init();
//
//	Mat rgbImg;											//the rgb input image [0 255]
//	Mat grayImg;										//the input img as grayvalue image [0 255]
//	Mat mask;											//the mask image [0 255]
//	Mat segImg;											//the segmented image
//	Mat segSatImg;										//the segmented image with combined saturation channel
//	Mat scaleSegImg;
//	Mat scaleSatImg;
//	//Mat satEnhImg;
//
//	int modeS;
//
//	float strokeW;										//the estimated stroke width
//	float sensitivity;									//the sensitivity for color boost
//	float confidenceThreshold;							//the confidence threshold used by getSegmented();
//
//	int fgdEstFilterSize;								//the filter size for the foreground estimation
//	int erodeMaskSize;									//size for the boundary erosion
//	//int calcBlobSizeFiltered;							//the actual minimal blob size of the filtered image
//	float sigmSlope;									//the slope of the sigmoid function
//
//	int segmentedPx;									//the amount of segmented pixel
//	int maskPx;											//the amount of pixel of the snippet area
//	float confidence;
//	Scalar meanContrast;
//	Scalar stdContrast;
//
//	//debug Images
//	Mat contrastImg;									//contrast image [0.0f 1.0f]
//	Mat binContrastImg;									//binContrastImg; [0.0f 1.0f]
//	//Mat contrastColorImg;								//contrast image with color enhancement
//	Mat fgdEstImg;										//foreground estimation image
//	Mat thrImg;											//the threshold image
//	Mat erodedMask;										//eroded mask image
//};

