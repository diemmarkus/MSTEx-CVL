/***************************************************
 *   DkIP.h
 *   
 *   Created on: 08.02.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/

#pragma once

//#include "DkCoreIncludes.h"

#include <list>
#include <limits>
#include "DkError.h"
#include "DkUtils.h"
#include "DkTimer.h"
#include "DkMath.h"

#ifdef DK_SAVE_DEBUG
	#include "opencv/highgui.h"
#endif

using namespace cv;

/**
 * The image processing unit.
 * This class DK_CORE_API provides useful image processing methods.
 **/
class DK_CORE_API DkIP {

public:
	enum DkDim {DK_S_DIM_X = 0, DK_S_DIM_Y};
	enum morph_border{DK_BORDER_ZERO = 0, DK_BORDER_FLIP};
	enum morph_shape{DK_SQUARE = 0, DK_DISK};

	// templates ---------------------------------------------------------------------
	template<typename sFmt>
	static bool isBinaryIntern(const Mat src) {

		sFmt* srcPtr = (sFmt*) src.data;
		int srcStep = (int) src.step/sizeof(sFmt);

		sFmt valA = 1;
		sFmt valB = 1;

		for (int rIdx = 0; rIdx < src.rows; rIdx++, srcPtr += srcStep) {

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				if (valA != 1 && valB != 1 && srcPtr[cIdx] != valA && srcPtr[cIdx] != valB) {
					return false;
				}
				if (valA == 1 && srcPtr[cIdx] != valB) {
					valA = srcPtr[cIdx];
				}
				else if (valB == 1 && srcPtr[cIdx] != valA) {
					valB = srcPtr[cIdx];
				}
			}
		}

		return true;
	};


	template<typename sFmt, typename mFmt>
	static void mulMaskIntern(Mat src, const Mat mask) {

		sFmt* srcPtr = (sFmt*) src.data;
		const mFmt* mPtr = (mFmt*) mask.data;

		int srcStep = (int) src.step/sizeof(sFmt);
		int mStep = (int) mask.step/sizeof(mFmt);

		for (int rIdx = 0; rIdx < src.rows; rIdx++, srcPtr += srcStep, mPtr += mStep) {

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				if (mPtr[cIdx] == 0) srcPtr[cIdx] = 0;
			}
		}
	};

	template<typename sFmt>
	static DkBox findBoundingBoxIntern(Mat &src) {

		sFmt* srcPtr = (sFmt*) src.data;

		DkVector uc(0,0);
		DkVector lc(0,0);

		// find max y
		for (int idx = 0; idx < src.rows*src.cols; idx++, srcPtr++) {

			if (*srcPtr != 0) {
				uc.y = (float)cvFloor((float)idx/(float)src.cols);
				break;
			}
		}

		// find max x
		for (int idx = 0; idx < src.rows*src.cols; idx++, srcPtr += src.cols) {

			if (*srcPtr != 0) {
				uc.x = (float)cvFloor((float)idx/(float)src.cols);
				break;
			}

			if (idx % src.rows == 0)
				srcPtr = (sFmt*)src.data + idx % src.cols;
		}

		// find new height
		for (int idx = src.rows*src.cols; idx > 0; idx--, srcPtr--) {

			if (*srcPtr != 0) {
				lc.height = (float)cvFloor((float)idx/(float)src.cols);
				break;
			}
		}

		// find new width
		for (int idx = src.rows*src.cols; idx > 0; idx--, srcPtr -= src.cols) {

			if (*srcPtr != 0) {
				lc.width = (float)cvFloor((float)idx/(float)src.cols);
				break;
			}

			if (idx % src.rows == 0)
				srcPtr = (sFmt*)src.data + idx % src.cols;
		}

		return DkBox(uc, lc-uc);

	};


		// templates ---------------------------------------------------------------------
	template<typename sFmt>
	static void setBorderConstIntern(Mat src, sFmt val) {

		sFmt* srcPtr = (sFmt*) src.data;
		sFmt* srcPtr2 = (sFmt*) src.ptr<sFmt*>(src.rows-1);
		int srcStep = (int) src.step/sizeof(sFmt);

		for (int cIdx = 0; cIdx < src.cols; cIdx++) {
			srcPtr[cIdx] = val;
			srcPtr2[cIdx] = val;
		}

		srcPtr = (sFmt*)src.data;
		for (int rIdx = 0; rIdx < src.rows; rIdx++, srcPtr += srcStep) {
			srcPtr[0] = val;
			srcPtr[src.cols-1] = val;
		}
	};

	template<typename sFmt>
	static sFmt getPixelInfoIntern(Mat src, int x, int y) {

		sFmt* srcPtr = (sFmt*) src.ptr<sFmt*>(y);

		return srcPtr[x];
	};


	// templates ---------------------------------------------------------------------

public:

	/**
	 * Erodes the image bwImg.
	 * This method erodes an image with a given structuring element.
	 * @param bwImg a grayscale image CV_8U (or CV_32F [0 1] but slower).
	 * @param seSize the structuring elemnt's size.
	 * @param shape the structuring element's shape (either DK_SQUARE or DK_DISK).
	 * @return an eroded image (CV_8U or CV_32F)
	 **/
	static Mat erodeImage(const Mat bwImg, int seSize, int shape = DK_SQUARE) {

		DkTimer dt = DkTimer();

		// nothing to do in here
		if (seSize < 3) return bwImg.clone();
		if (seSize % 2 == 0) seSize += 1;

		if (bwImg.channels() > 1)
			throw DkMatException("Gray-scale image is required.", __LINE__, __FILE__);

		Mat eImg;
		Mat imgU = bwImg;
		
		// TODO: bug if 32F & DK_DISK
		// erode is much faster (and correct) with CV_8U
		if (bwImg.depth() != CV_8U)
			bwImg.convertTo(imgU, CV_8U, 255);

		dt.stop();
		
		Mat se = createStructuringElement(seSize, shape);
		erode(imgU, eImg, se, Point(-1,-1), 1, BORDER_CONSTANT, 255);
		iout << "image erosion: " << dt << dkendl;

		imgU = eImg;
		if (bwImg.depth() != CV_8U)
			imgU.convertTo(eImg, bwImg.depth(), 1.0f/255.0f);

		iout << "(total: " << dt << ")" << dkendl;
		
		return eImg;
	}

	/**
	 * Erodes the image bwImg using integral images.
	 * This method erodes an image with a squared structuring element.
	 * It must not be applied to gray scale images.
	 * This function is generally faster if a 32F erosion needs to be performed
	 * or if the structuring element is > 10 since it is constant with respect
	 * to the structuring element's size.
	 * @param bwImg a binary image CV_32F (or CV_8U [0 255] but faster).
	 * @param seSize the structuring element's size.
	 * @return an eroded image (CV_32F or CV_8U)
	 **/
	static Mat fastErodeImage(const Mat bwImg, int seSizeX, int seSizeY = 0) {

		DkTimer dt = DkTimer();

		if (bwImg.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(bwImg.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!(bwImg.type() == CV_32FC1 || bwImg.type() == CV_8UC1)) {
			std::string msg = "the image needs to be CV_32FC1 or CV_8UC1, it is: " + 
				DkUtils::getMatInfo(bwImg);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// nothing to do in here
		if (seSizeX < 3 && seSizeY < 3) return bwImg.clone();
		if (seSizeX % 2 == 0) seSizeX += 1;
		if (seSizeY != 0 && seSizeY % 2 == 0) seSizeY += 1;

		Mat eImg;

		dt.stop();

		// different workflow for 8U and 32F
		if (bwImg.depth() == CV_8U) {

			if (seSizeX == seSizeY && seSizeX < 21)
				eImg = erodeImage(bwImg, seSizeX, DK_SQUARE);
			else {

				Mat imgInt = Mat(bwImg.rows+1, bwImg.cols+1, CV_32SC1);
				dt.stop();

				// compute a mean filter -> threshold it
				integral(bwImg, imgInt);
				eImg = morphIntegral8U(imgInt, seSizeX, seSizeY, DK_ERODE);

				DkUtils::printDebug(DK_DEBUG_INFO, "fast image erosion 8U: %s ", dt.getIvl().c_str());
			}
		}
		else {
			
			dt.stop();

			Mat imgInt = DkIP::binaryIntegralImage(bwImg);
			eImg = morphIntegral32F(imgInt, seSizeX, seSizeY, DK_ERODE);

			DkUtils::printDebug(DK_DEBUG_INFO, "fast image erosion 32F: %s ", dt.getIvl().c_str());
		}
		DkUtils::printDebug(DK_DEBUG_INFO, "(total: %s)\n", dt.getTotal().c_str());

		return eImg;
	}

	/**
	 * Dilates the image bwImg.
	 * This method dilates (morphological dilation) an image with a given structuring element.
	 * @param bwImg a grayscale image CV_8U (or CV_32F [0 1] but slower).
	 * @param seSize the structuring element's size.
	 * @param shape the structuring element's shape (either DK_SQUARE or DK_DISK).
	 * @return a dilated image (CV_8U)
	 **/
	static Mat dilateImage(const Mat bwImg, int seSize, int shape = DK_SQUARE) {

		DkTimer dt = DkTimer();

		// nothing to do in here
		if (seSize < 3) return bwImg.clone();
		if (seSize % 2 == 0) seSize += 1;

		if (bwImg.channels() > 1)
			throw DkMatException("Gray-scale image is required.", __LINE__, __FILE__);

		Mat dImg;
		// TODO: bug if 32F & DK_DISK
		// dilate is much faster (and correct) with CV_8U
		Mat imgU = bwImg;
		if (bwImg.depth() != CV_8U)
			bwImg.convertTo(imgU, CV_8U, 255);

		dt.stop();
		Mat se = createStructuringElement(seSize, shape);
		dilate(imgU, dImg, se, Point(-1,-1), 1, BORDER_CONSTANT, 0);

		DkUtils::printDebug(DK_DEBUG_INFO, "image dilation: %s", dt.getIvl().c_str());
		DkUtils::printDebug(DK_DEBUG_INFO, " (total: %s)\n", dt.getTotal().c_str());

		return dImg;
	}

	static Mat skeleton(const Mat& bwImg) {


		cv::Mat img = bwImg.clone();
		cv::Mat skel(bwImg.size(), CV_8UC1, cv::Scalar(0));
		cv::Mat temp(bwImg.size(), CV_8UC1);

		cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

		bool done;
		do
		{
			cv::morphologyEx(img, temp, cv::MORPH_OPEN, element);
			cv::bitwise_not(temp, temp);
			cv::bitwise_and(img, temp, temp);
			cv::bitwise_or(skel, temp, skel);
			cv::erode(img, img, element);

			double max;
			cv::minMaxLoc(img, 0, &max);
			done = (max == 0);
		} while (!done);

		return skel;
	}

	template<typename num>
	static Mat dilateLabelImageFast(const Mat& src) {

		Mat dist, lab;
		DkIP::dilateLabelImageFast<num>(src.t(), dist, lab);
		dist = dist.t(); lab = lab.t();
		DkIP::dilateLabelImageFast<num>(src, dist, lab);

		return lab;
	}

	template<typename num>
	static void dilateLabelImageFast(const Mat& src, Mat& idxImg, Mat& labImg) {

		bool firstPass = false;

		if (idxImg.empty()) {
			idxImg = Mat(src.size(), CV_32SC1);
			idxImg.setTo(INT_MAX);
			firstPass = true;
		}
		if (labImg.empty()) {
			labImg = src.clone();
		}

		Mat emptyRows = Mat(1, src.rows, src.depth());
		num* emptyPtr = emptyRows.ptr<num>(); 

		// label all rows
		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			const num* ptr = src.ptr<num>(rIdx);
			num* labPtr = labImg.ptr<num>(rIdx);
			unsigned int* idxPtr = idxImg.ptr<num>(rIdx);

			int nextLabelIdx = -1;
			int prevLabelIdx = -2*src.cols;

			num nextLabelVal = 0;
			num prevLabelVal = 0;

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				// don't forget the previous labels
				if (ptr[cIdx] != 0) {
					prevLabelIdx = cIdx;
					prevLabelVal = ptr[cIdx];
					idxPtr[cIdx] = 0;
					nextLabelIdx = -1;
					continue;
				}

				if (!firstPass && cIdx > 0 && idxPtr[cIdx-1]+1 < idxPtr[cIdx]) {
					idxPtr[cIdx] = idxPtr[cIdx-1]+1;
					labPtr[cIdx] = labPtr[cIdx-1];
				}

				// find the next label idx
				if (cIdx > nextLabelIdx) {

					nextLabelIdx = 2*src.cols;
					nextLabelVal = 0;

					// search next non-null label
					for (int sIdx = cIdx; sIdx < src.cols; sIdx++) {

						if (ptr[sIdx] != 0) {
							nextLabelIdx = sIdx;
							nextLabelVal = ptr[sIdx];
							break;
						}
					}
				}

				// empty line - skip
				if (prevLabelIdx == -2*src.cols && nextLabelIdx == 2*src.cols) {
					emptyPtr[rIdx] = 0;

					if (firstPass)
						break;
				}
				else 
					emptyPtr[rIdx] = 1;

				unsigned int currentDist = min(cIdx-prevLabelIdx, nextLabelIdx-cIdx);

				if (currentDist < idxPtr[cIdx]) {
					idxPtr[cIdx] = currentDist;

					if (prevLabelIdx == -2*src.cols)
						labPtr[cIdx] = nextLabelVal;
					else if (nextLabelIdx == 2*src.cols)
						labPtr[cIdx] = prevLabelVal;
					else
						labPtr[cIdx] = (cIdx - prevLabelIdx < (nextLabelIdx-prevLabelIdx)*0.5f) ? prevLabelVal : nextLabelVal;
				}
			}

			if (!firstPass) {

				// reverse label current elements
				for (int cIdx = src.cols-2; cIdx >= 0; cIdx--) {

					if (cIdx-1 < src.cols && idxPtr[cIdx] > idxPtr[cIdx+1]) {
						idxPtr[cIdx] = idxPtr[cIdx+1]+1;
						labPtr[cIdx] = labPtr[cIdx+1];
					}

				}
			}

		}

		if (!firstPass)
			return;

		int prevIdx = -1, nextIdx = -1;
		Mat tmpRow = Mat(1, src.cols, src.depth());

		for (int rIdx = 0; rIdx < emptyRows.cols; rIdx++) {


			if (emptyPtr[rIdx] > 0) {
				prevIdx = rIdx;
				continue;
			}

			// search for next non-empty row
			if (rIdx > nextIdx) {

				nextIdx = -1;

				for (int sIdx = rIdx; sIdx < emptyRows.cols; sIdx++) {

					if (emptyPtr[sIdx] == 0)
						continue;

					nextIdx = sIdx;
					break;
				}
			}

			// nothing to do
			if (prevIdx == -1 && nextIdx == -1)
				break;
			else if (nextIdx == -1) {
				labImg.row(prevIdx).copyTo(labImg.row(rIdx));
				idxImg.row(prevIdx).copyTo(idxImg.row(rIdx));
				idxImg.row(rIdx) *= rIdx-prevIdx;
			}
			else if (prevIdx == -1) {
				labImg.row(nextIdx).copyTo(labImg.row(rIdx));
				idxImg.row(nextIdx).copyTo(idxImg.row(rIdx));
				idxImg.row(rIdx) *= nextIdx-rIdx;
			}
			else {
				labImg.row((rIdx - prevIdx < (nextIdx-rIdx)*0.5f) ? prevIdx : nextIdx).copyTo(labImg.row(rIdx));
				idxImg.row((rIdx - prevIdx < (nextIdx-rIdx)*0.5f) ? prevIdx : nextIdx).copyTo(idxImg.row(rIdx));
				idxImg.row(rIdx) *= min(rIdx-prevIdx, nextIdx-rIdx);
			}

		}

	}


	template <typename num>
	static unsigned int dilateLabelImage(Mat& labelImg, const Mat mask) {

		//if (labelImg.type() != CV_8UC1)
		//	throw DkMatException("labelimg: 8U Gray-scale image is required.", __LINE__, __FILE__);

		if (mask.type() != CV_8UC1)
			throw DkMatException("mask: 8U Gray-scale image is required.", __LINE__, __FILE__);

		if (mask.size() != labelImg.size())
			throw DkMatException("mask has different dimensions compared to the label image", __LINE__, __FILE__);

		Mat tmpLabel = labelImg.clone();

		unsigned int changeBit = 0; //no change on image

		for (int row=0; row<labelImg.rows; row++) {

			num *ptrLastRow = row > 1 ? labelImg.ptr<num>(row-1) : 0;
			num *ptrCurrRow = labelImg.ptr<num>(row);
			num *ptrNextRow = row < labelImg.rows-1 ? labelImg.ptr<num>(row+1) : 0;
			const unsigned char *ptrMask = mask.ptr<unsigned char>(row);
			num maxVal = 0;

			num *ptrDilimg = tmpLabel.ptr<num>(row);

			for (int col=0; col<labelImg.cols; col++) {

				maxVal = 0;
				if (ptrMask[col] == 0) continue;
				if (ptrDilimg[col] != 1) continue;

				maxVal = ptrLastRow ? col >= 1 ? ptrLastRow[col-1] : 0 : 0;											//left upper
				maxVal = maxVal > (ptrLastRow ? ptrLastRow[col] : 0) ? maxVal : (ptrLastRow ? ptrLastRow[col] : 0);	//middle upper
				maxVal = maxVal > (ptrLastRow ? (col<labelImg.cols-1) ? ptrLastRow[col+1] : 0 : 0) ? 
								maxVal : (ptrLastRow ? (col<labelImg.cols-1) ? ptrLastRow[col+1] : 0 : 0);			//right upper

				maxVal = maxVal > (col >= 1 ? ptrCurrRow[col-1] : 0) ? maxVal : (col >= 1 ? ptrCurrRow[col-1] : 0);								//left middle
				maxVal = maxVal > (col < labelImg.cols-1 ? ptrCurrRow[col+1] : 0) ? maxVal : (col < labelImg.cols-1 ? ptrCurrRow[col+1] : 0);	//right middle

				maxVal = maxVal > (ptrNextRow ? col >= 1 ? ptrNextRow[col-1] : 0 : 0) ? maxVal : (ptrNextRow ? col >= 1 ? ptrNextRow[col-1] : 0 : 0);											//left down
				maxVal = maxVal > (ptrNextRow ? ptrNextRow[col] : 0) ? maxVal : (ptrNextRow ? ptrNextRow[col] : 0);	//middle down
				maxVal = maxVal > (ptrNextRow ? (col<labelImg.cols-1) ? ptrNextRow[col+1] : 0 : 0) ? 
								maxVal : (ptrNextRow ? (col<labelImg.cols-1) ? ptrNextRow[col+1] : 0 : 0);			//right down

				if (maxVal > 1) {
					ptrDilimg[col] = maxVal;
					changeBit = 1;
				}

			}

		}

		labelImg = tmpLabel;

		return changeBit;
	}

	/**
	 * Dilates the image bwImg using integral images.
	 * This method dilates an image with a squared structuring element.
	 * It must not be applied to gray scale images.
	 * This function is generally faster if a 32F erosion needs to be performed
	 * or if the structuring element is > 10 since it is constant with respect
	 * to the structuring element's size.
	 * @param bwImg a binary image CV_32F (or CV_8U [0 255] faster).
	 * @param seSize the structuring element's size.
	 * @return a dilated image (CV_32F or CV_8U)
	 **/
	static Mat fastDilateImage(const Mat bwImg, int seSizeX, int seSizeY = 0) {
		
		DkTimer dt = DkTimer();

		if (bwImg.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(bwImg.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!(bwImg.type() == CV_32FC1 || bwImg.type() == CV_8UC1)) {
			std::string msg = "the image needs to be CV_32FC1 or CV_8UC1, it is: " + 
				DkUtils::getMatInfo(bwImg);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// nothing to do in here
		if (seSizeX < 3 && seSizeY < 3) return bwImg.clone();
		if (seSizeX % 2 == 0) seSizeX += 1;
		if (seSizeY != 0 && seSizeY % 2 == 0) seSizeY += 1;

		Mat dImg;

		// different workflow for 8U and 32F
		if (bwImg.depth() == CV_8U) {
			
			// for small kernels, OpenCV dilation is faster...
			if (seSizeX == seSizeY && seSizeX < 21)
				dImg = dilateImage(bwImg, seSizeX, DK_SQUARE);
			else {
				Mat imgInt = Mat(bwImg.rows+1, bwImg.cols+1, CV_32SC1);
				dt.stop();

				// compute a mean filter -> threshold it
				integral(bwImg, imgInt);
				dImg = morphIntegral8U(imgInt, seSizeX, seSizeY, DK_DILATE);

				DkUtils::printDebug(DK_DEBUG_INFO, "fast image dilation 8U: %s ", dt.getIvl().c_str());
			}
		}
		else {

			dt.stop();

			Mat imgInt = DkIP::binaryIntegralImage(bwImg);
			dImg = morphIntegral32F(imgInt, seSizeX, seSizeY, DK_DILATE);

			DkUtils::printDebug(DK_DEBUG_INFO, "fast image dilation 32F: %s ", dt.getIvl().c_str());
		}

		DkUtils::printDebug(DK_DEBUG_INFO, "(total: %s)\n", dt.getTotal().c_str());

		return dImg;
	}

	/**
	 * Creates a structuring element for morphological operations.
	 * @param seSize the structuring element's size
	 * @param shape either DK_SQUARE or DK_DISK
	 * @return a Matrix containing the structuring element (CV_8UC1)
	 **/
	static Mat createStructuringElement(int seSize, int shape) {

		Mat se = Mat(seSize, seSize, CV_8U);

		switch(shape) {

		case DK_SQUARE:
			se = 1;
			break;
		case DK_DISK:

			se.setTo(0);

			int c = DkMath::halfInt(seSize);	// radius
			int r = c*c;						// center

			for (int rIdx = 0; rIdx < se.rows; rIdx++) {

				unsigned char* sePtr = se.ptr<unsigned char>(rIdx);

				for (int cIdx = 0; cIdx < se.cols; cIdx++) {

					// squared pixel distance to center
					int dist = (rIdx-c)*(rIdx-c) + (cIdx-c)*(cIdx-c);

					//printf("distance: %i, radius: %i\n", dist, r);
					if (dist < r)
						sePtr[cIdx] = 1;
				}
			}
			break;
		}

		return se;
	}

	/**
	 * Converts an RGB image to IHLS color space.
	 * @param img a 3channel rgb color image CV_32FC3 ([0 255] or [0 1]).
	 * @param mask the image mask.
	 * @return the ihls image CV_32FC3 ([0 255] or [0 1]).
	 **/
	static Mat convertRGBtoIHLS(Mat img, Mat mask) {

		if (img.type() != CV_32FC3) {
			std::string msg = "the image needs to be CV_32FC3, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}
		if (mask.type() != CV_8UC1) {
			std::string msg = "the image needs to be CV_8UC3, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		std::vector<Mat> rgbCh;
		split(img, rgbCh);

		Mat ihls = Mat(img.size(), CV_32FC3);
		std::vector<Mat> hlsCh;
		split(ihls, hlsCh);

		for(int i = 0; i < rgbCh[0].rows; i++)
		{
			float *ptrR = rgbCh[0].ptr<float>(i);
			float *ptrG = rgbCh[1].ptr<float>(i);
			float *ptrB = rgbCh[2].ptr<float>(i);
			unsigned char *ptrMask = mask.ptr<unsigned char>(i);
			float *ptrH = hlsCh[0].ptr<float>(i);
			float *ptrY = hlsCh[1].ptr<float>(i);
			float *ptrS = hlsCh[2].ptr<float>(i);

			for(int j = 0; j < rgbCh[0].cols; j++) {
				if (ptrMask[j] != 0) {
					float r = ptrR[j];
					float g = ptrG[j];
					float b = ptrB[j];
					ptrS[j] = max(max(r, g), b) - min(min(r, g), b);
					ptrY[j] = 0.2125f * r + 0.7154f * g + 0.0721f * b;

					float crx = r-(g+b)/2.0f;
					float cry = sqrt(3.0f)/2.0f*(b-g + FLT_MIN);                                //avoid zero division!!!!!!
					float cr = sqrt(crx*crx + cry*cry);
					if (cr == 0.0f)
						ptrH[j] = -1.0f; 		// undefined
					else if (cry <= 0)
						ptrH[j] = (float)acos(crx/cr);
					else
						ptrH[j] = 2.0f*(float)CV_PI - (float)acos(crx/cr);
				} else {
					ptrH[j] = -1.0f;
					ptrY[j] = -1.0f;
					ptrS[j] = -1.0f;
				}
			}
		}
		merge(hlsCh, ihls);
		return ihls;
	}

	/**
	 * Converts an RGB point to an IHLS point.
	 * @param rgb point ([0 255] or [0 1]).
	 * @return ihls point ([0 255] or [0 1]).
	 **/
	static DkVector3 convertRGBtoIHLS(DkVector3 rgb) {

		DkVector3 ihls;

		float r = rgb.r;
		float g = rgb.g;
		float b = rgb.b;

		// IHLS tranform
		ihls.s = max(max(r, g), b) - min(min(r, g), b);		// saturation
		ihls.l = 0.2125f * r + 0.7154f * g + 0.0721f * b;	// luminance
		float crx = r-(g+b)/2;
		float cry = sqrt(3.0f)/2.0f*(b-g + FLT_MIN);                                //avoid zero division!!!!!!
		float cr = sqrt(crx*crx + cry*cry);
		if (cr == 0.0f)
			ihls.h = -1.0f; 		// undefined
		else if (cry <= 0)
			ihls.h = (float)acos(crx/cr);
		else
			ihls.h = 2.0f*(float)CV_PI - (float)acos(crx/cr);

		return ihls;
	}

	/**
	 * Converts an RGB point to an XYZ point.
	 * @param rgb point ([0 1]).
	 * @return XYZ point ([0 1]).
	 **/
	static DkVector3 convertRGBtoXYZ(DkVector3 rgb) {

		DkVector3 xyz;

		// XYZ tranform
		xyz.x = 0.412453f*rgb.r + 0.357580f*rgb.g + 0.180423f*rgb.b;
		xyz.y = 0.212671f*rgb.r + 0.715160f*rgb.g + 0.072169f*rgb.b;
		xyz.z = 0.019334f*rgb.r + 0.119193f*rgb.g + 0.950227f*rgb.b;

		return xyz;
	}

	/**
	 * Converts an RGB point to an Lab point.
	 * @param rgb point ([0 1]).
	 * @return Lab point (L [0 100] a,b [-128 128]).
	 **/
	static DkVector3 convertRGBtoLab(DkVector3 rgb) {

		DkVector3 lab;

		// Lab tranform
		DkVector3 xyz = convertRGBtoXYZ(rgb);
		float xn = 0.950456f;
		float zn = 1.088754f;
		xyz.x = xyz.x/xn;
		xyz.z = xyz.z/zn;

		lab.x = xyz.y > 0.008856 ? 116.0f*std::pow(xyz.y,1.0f/3.0f)-16.0f : 903.3f*xyz.y;
		float fx = xyz.x > 0.008856 ? std::pow(xyz.x, 1.0f/3.0f) : 7.787f*xyz.x + 16.0f/116.0f;
		float fy = xyz.y > 0.008856 ? std::pow(xyz.y, 1.0f/3.0f) : 7.787f*xyz.y + 16.0f/116.0f;
		float fz = xyz.z > 0.008856 ? std::pow(xyz.z, 1.0f/3.0f) : 7.787f*xyz.z + 16.0f/116.0f;

		float delta = 0.0f;
		lab.y = 500.0f * (fx-fy) + delta;
		lab.z = 200.0f * (fy-fz) + delta;

		return lab;
	}

	/**
	 * Calculates the euclidean color distance (in Lab) of two RGB points.
	 * A distance smaller < 1.0 is supposed to be indistinguishable unless the samples are adjacent to one another.
	 * @param a RGB.
	 * @param b RGB.
	 * @return color distance .
	 **/
	static float distanceRGB(DkVector3 a, DkVector3 b) {

		DkVector3 aLab = DkIP::convertRGBtoLab(a);
		DkVector3 bLab = DkIP::convertRGBtoLab(b);

		return DkIP::distanceLab(aLab, bLab);
	}

	/**
	 * Calculates the euclidean color distance of two Lab points.
	 * A distance smaller < 1.0 is supposed to be indistinguishable unless the samples are adjacent to one another.
	 * @param a Lab.
	 * @param b Lab.
	 * @return color distance.
	 **/
	static float distanceLab(DkVector3 a, DkVector3 b) {

		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
	}

	/**
	 * Calculates the distance of two colors in rad (IHLS)
	 * @param p1 first color (IHLS)
	 * @param p2 second color (IHLS)
	 * @return color distance in rad
	 **/
	static float distanceIHLS(DkVector3 p1, DkVector3 p2) {

		// if both colors are gray
		if (p1.s == 0.0f && p2.s == 0.0f) {
			return (p1.l-p2.l)/255.0f;
		}

		double a = p1.s * cos(p1.h) + p2.s * cos(p2.h);
		double b = p1.s * sin(p1.h) + p2.s * sin(p2.h);
		
		return 1.0f-(float)sqrt(a*a + b*b)/(p1.s+p2.s+FLT_MIN);

		//return min(fabs(p1.x-p2.x),2.0f*(float)CV_PI - fabs(p1.x-p2.x));
	}

	/**
	 * Converts an IHLS point to an RGB point.
	 * @param hls point ([0 255] or [0 1]).
	 * @return rgb point ([0 255] or [0 1]).
	 **/
	static DkVector3 convertIHLStoRGB(DkVector3 hls) {

		float k = (float)cvFloor(hls.h / ((float)CV_PI/3.0f));
		float hstar = hls.h - k * (float)CV_PI / 3.0f;
		float ct = sqrt(3.0f)*hls.s;
		float cb = 2.0f * sin(-hstar + 2.0f/3.0f * (float)CV_PI);
		float c = ct/cb;

		float c1 = c * cos(hls.h);
		float c2 = -c * sin(hls.h);

		DkVector3 rgb;

		rgb.r = hls.l + 0.7875f*c1 + 0.3714f * c2;
		rgb.g = hls.l - 0.2125f*c1 - 0.2059f * c2;
		rgb.b = hls.l - 0.2125f*c1 + 0.9488f * c2;

		//rgb.clipTo();	// values are [0 255]

		return rgb;
	}

	/**
	 * Computes a 1D Gaussian filter kernel.
	 * Generates a Gaussian kernel. The kernel's size is adjusted to
	 * the standard deviation.
	 * @param sigma the standard deviation of the Gaussian.
	 * @return the gaussian kernel (CV_32FC1)
	 **/
	static Mat get1DGauss(double sigma) {

		// correct -> checked with matlab reference
		int kernelsize = cvRound(cvCeil(sigma*3)*2)+1;
		if (kernelsize < 3) kernelsize = 3;
		if ((kernelsize % 2) != 1) kernelsize+=1;

		Mat gKernel = Mat(1, kernelsize, CV_32F);
		float* kernelPtr = gKernel.ptr<float>();

		for (int idx = 0, x = -cvFloor(kernelsize/2); idx < kernelsize; idx++,x++) {

			kernelPtr[idx] = (float)(exp(-(x*x)/(2*sigma*sigma)));	// 1/(sqrt(2pi)*sigma) -> discrete normalization
		}


		if (sum(gKernel).val[0] == 0)
			throw DkIllegalArgumentException("The kernel sum is zero\n", __LINE__, __FILE__);
		else
			gKernel *= 1.0f/sum(gKernel).val[0];

		return gKernel;
	}

	/**
	 * Computes second derivative in x of a 1D Gaussian filter kernel.
	 * The kernel's size is adjusted to the standard deviation.
	 * @param sigma the standard deviation of the Gaussian.
	 * @return the second derivative in x of a gaussian kernel (CV_32FC1)
	 **/
	static Mat get1DGaussDxx(const float sigma) {

		// correct -> checked with matlab reference
		int kernelsize = cvRound((10*sigma-5)*2/3);
		if (kernelsize < 3) kernelsize = 3;
		if ((kernelsize % 2) != 1) kernelsize+=1;

		Mat gKernel = Mat(1,kernelsize, CV_32F);
		float* kernelPtr = gKernel.ptr<float>();

		for (int i = 0, x = -cvFloor(kernelsize/2); i < kernelsize; i++,x++) {
			kernelPtr[i] = 	(-1/(cvSqrt((float)2.0f*CV_PI)*sigma*sigma*sigma)*exp(-(x*x)/(2.0f*sigma*sigma))) * (1.0f-(x*x)/(sigma*sigma));
			//kernelPtr[i] = 	exp(-(x*x)/(2.0f*sigma*sigma)) * (1.0f-(x*x)/(sigma*sigma));
		}


		//float ks = (float)sum(abs(gKernel)).val[0];

		//if (ks == 0)
		//	throw DkIllegalArgumentException("The kernel sum is zero\n", __LINE__, __FILE__);
		//else
		//	gKernel *= 1/ks;


		return gKernel;
	}

	/**
	 * Detects vertical zero-crossings in an image.
	 * @param src a grayscale image CV_32FC1 [0 1]
	 * @param thresh the minimum value of a pixel to be a valid zero-crossing.
	 * @return a binary image with 1 at locations of zero-crossings.
	 **/
	static Mat findZeroCrossing(Mat src, const float thresh) {

		Mat dst = Mat(src.size(), src.type());
		dst.setTo(0);
		float* ptrsrc = src.ptr<float>(0);

		for (int row = 1; row < dst.rows-1; row++) {

			float* ptrimg = dst.ptr<float>(row);
			int pRow = (row-1)	* dst.cols;			// previous row
			int cRow = row		* dst.cols; 		// current row
			int nRow = (row+1)	* dst.cols;			// next row

			for (int col = 0; col < dst.cols; col++) {

				if (ptrsrc[cRow] > ptrsrc[pRow] && ptrsrc[cRow] >= ptrsrc[nRow] && fabs(ptrsrc[pRow]-ptrsrc[nRow]) > thresh)
					*ptrimg = 1;
				else
					*ptrimg = 0;

				pRow++; cRow++; nRow++;
				ptrimg++;
			}
		}

		return dst;
	}

	/**
	 * Computes the orientation histogram.
	 * This method returns the orientation histogram having numBinsGrad bins.
	 * @param magImg the gradient magnitude image CV_32FC1 [0 1].
	 * @param radImg the gradient orientation image CV_32FC1 [0 2pi].
	 * @param numBinsGrad the number of bins (default 180).
	 * @param thetaIvlGrad the gradient orientation interval.
	 * @return the orientation histogram.
	 **/
	static Mat computeOrHist(Mat magImg, Mat radImg, int numBinsGrad = 180, int thetaIvlGrad = 180) {

		float thetaIvlRad = thetaIvlGrad * (float) DK_DEG2RAD;
		float normBins = (float)numBinsGrad/(float)thetaIvlRad;
		int index = 0;

		Mat orHist = Mat(1, numBinsGrad, CV_32F);
		orHist.setTo(0);

		// compute gradient magnitude
		int cols = magImg.cols;
		int rows = magImg.rows;

		// speed up for accessing elements
		if(magImg.isContinuous()) {
			cols *= rows;
			rows = 1;
		}

		for (int rIdx = 0; rIdx < rows; rIdx++) {

			const float* magPtr = magImg.ptr<float>(rIdx);
			const float* radPtr = radImg.ptr<float>(rIdx);

			float* histPtr = orHist.ptr<float>();

			for (int cIdx = 0; cIdx < cols; cIdx++) {

				// skip 0 values -> speed
				if (magPtr[cIdx] == 0)
					continue;

				// convert [0 2pi] to [0 thetaIvl]
				float angle = radPtr[cIdx];
				while(angle > thetaIvlRad)
					angle -= thetaIvlRad;

				// find histogram bin for current angle
				index = cvRound(angle*normBins);
				index %= numBinsGrad;

				// accumulate gradient magnitude
				histPtr[index] += magPtr[cIdx];
			}
		}

		return orHist;
	}



	/**
	 * Convolves a histogram symmetrically.
	 * Symmetric convolution means that the convolution
	 * is flipped around at the histograms borders. This is
	 * specifically useful for orientation histograms (since
	 * 0° corresponds to 360°
	 * @param hist the histogram CV_32FC1
	 * @param kernel the convolution kernel CV_32FC1
	 * @return the convolved histogram CV_32FC1
	 **/
	static Mat convolveSymmetric(const Mat hist, const Mat kernel) {

		if (hist.channels() > 1) {
			std::string msg = "the histogram needs to have 1 channel, but it has: " + 
				DkUtils::stringify(hist.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (kernel.channels() > 1) {
			std::string msg = "the kernel needs to have 1 channel, but it has: " + 
				DkUtils::stringify(kernel.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (hist.type() != CV_32FC1) {
			std::string msg = "the histogram needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(hist);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (kernel.type() != CV_32FC1) {
			std::string msg = "the kernel needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(kernel);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		int hs = hist.rows * hist.cols;			// histogram size
		int ks = kernel.rows * kernel.cols;		// kernel size
		int halfKs = cvFloor(ks/2);				// half kernel size
		int cs = hs + ks-1;						// histogram size + kernel size

		Mat symHistSmooth;
		Mat symHist = Mat(1, cs, CV_32F);

		const float* histPtr = hist.ptr<float>();
		float* symHistPtr = symHist.ptr<float>();

		for (int nIdx = 0, oIdx = -halfKs; nIdx < cs; nIdx++, oIdx++) {

			oIdx += hs;
			oIdx %= hs;

			symHistPtr[nIdx] = histPtr[oIdx];
		}

		filter2D(symHist, symHist, -1, kernel);
		symHistSmooth = symHist.colRange(halfKs, cs-halfKs);

		return symHistSmooth.clone();	// delete values outside the range

	}

	/**
	 * Reverts a histogram (1xN Mat).
	 * Thus, the N-1st column becomes the 0th column.
	 * @param hist a CV_32F histogram
	 **/ 
	static void revertHist(Mat hist) {

		if (hist.channels() > 1) {
			std::string msg = "the histogram needs to have 1 channel, but it has: " + 
				DkUtils::stringify(hist.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}
		if (hist.type() != CV_32FC1) {
			std::string msg = "the histogram needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(hist);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		float* histPtr = hist.ptr<float>();

		for (int idx = 0, idxRev = hist.cols-1; idx < cvCeil((float)hist.cols/2.0f); idx++, idxRev--) {

			float tmp = histPtr[idx];
			histPtr[idx] = histPtr[idxRev];
			histPtr[idxRev] = tmp;
		}
	}

	/**
	 * Finds local maxima in a given histogram.
	 * The local maxima are located with sub-pixel accuracy (by means of polynomial interpolation)
	 * @param vec a 1xN histogram CV_32FC1.
	 * @param localMaxList a list where the indices of the local maxima are assigned.
	 * @param gSigma the standard deviation for pre-smoothing the histogram
	 * @param cutOffIvl a threshold which rejects weak local maxima. (the final threshold is computed
	 * by: cutOffIvl*gm where gm is the histogram's global maximum)
	 **/
	static void findLocalMaxima(const Mat vec, 
								std::list<float> *localMaxList, 
								const float gSigma = 0.0f, 
								const float cutOffIvl = 0.4f,
								const int numMax = -1) {


		if (vec.channels() > 1) {
			std::string msg = "the vector needs to have 1 channel, but it has: " + 
				DkUtils::stringify(vec.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (vec.type() != CV_32FC1) {
			std::string msg = "the vector needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(vec);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat sHist;
		if (gSigma > 0)
			sHist = convolveSymmetric(vec, get1DGauss(gSigma));
		else
			sHist = vec;

		// find the threshold for local maxima
		double sMin, sMax;
		minMaxLoc(sHist, &sMin, &sMax);
		float cutOff = (float)sMax*cutOffIvl;

		// perform first derivation
		float dxData[] = {1, -1};
		Mat dxKernel = Mat(1, 2, CV_32FC1, &dxData);
		Mat sDxHist = convolveSymmetric(sHist, dxKernel);

		float *sHistPtr = sHist.ptr<float>();
		float *sDxHistPtr = sDxHist.ptr<float>();

		// for num max
		std::list<DkVector> maxPoints;

		// find local maxima in the smoothed orientation histogram
		for (int idx = 0, nextIdx = 1; idx < sDxHist.cols; idx++, nextIdx++) {

			// flip around
			nextIdx %= sDxHist.cols;

			// search for positive zero passing in the first derivative
			if (sHistPtr[idx] > cutOff && sDxHistPtr[idx] < 0 && sDxHistPtr[nextIdx] >= 0) {

				//interpolate spline
				int splIvl [] = {idx-1, idx, idx+1};
				if (splIvl[0] <  0) splIvl[0] = sDxHist.cols-1;
				//if (splIvl[2] == sDxHist.cols) splIvl[2] = 1;
				if (splIvl[2] == sDxHist.cols) splIvl[2] = 0;			//FK 12.03.2010
				float ySpl [] = {sHistPtr[splIvl[0]], sHistPtr[splIvl[1]], sHistPtr[splIvl[2]]};
				float xMax, yMax;

				// could not compute main orientation since the orientation histogram is empty
				if (ySpl[1] == 0)
					return;

				DkMath::iplSplineEqDist((float)idx-1, ySpl, &xMax, &yMax);

				if (numMax != -1)
					maxPoints.push_back(DkVector(xMax, sHistPtr[idx]));
				else
					localMaxList->push_back(xMax);
			}
		}


		if (numMax != -1) {

			// DkVector sorts the y values if x1.y == x2.y than the x values are sorted
			maxPoints.sort();

			// vector is now sorted ascending
			std::list<DkVector>::reverse_iterator vecIter = maxPoints.rbegin();

			for (int idx = 0; idx < numMax && idx < (int)maxPoints.size(); idx++) {
				localMaxList->push_back((*vecIter).x);
				vecIter++;
			}

		}


	}

	/**
	 * Finds local maxima in a given histogram.
	 * @param vec a 1xN histogram CV_32FC1.
	 * @param localMaxList a list where the indices of the local maxima are assigned.
	 * @param gSigma the standard deviation for pre-smoothing the histogram
	 * @param cutOffIvl a threshold which rejects weak local maxima. (the final threshold is computed
	 * by: cutOffIvl*gm where gm is the histogram's global maximum)
	 **/
	static void findLocalMaxima(const Mat vec, 
								std::list<int> *localMaxList, 
								const float gSigma = 0.0f, 
								const float cutOffIvl = 0.4f,
								const int numMax = -1) {

		if (vec.channels() > 1) {
			std::string msg = "the vector needs to have 1 channel, but it has: " + 
				DkUtils::stringify(vec.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (vec.type() != CV_32FC1) {
			std::string msg = "the vector needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(vec);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat sHist;
		if (gSigma > 0)
			sHist = convolveSymmetric(vec, get1DGauss(gSigma));
		else
			sHist = vec;

		// find the threshold for local maxima
		double sMin, sMax;
		minMaxLoc(sHist, &sMin, &sMax);
		float cutOff = (float)sMax*cutOffIvl;

		// perform first derivation
		float dxData[] = {1, -1};
		Mat dxKernel = Mat(1, 2, CV_32FC1, &dxData);
		Mat sDxHist = convolveSymmetric(sHist, dxKernel);

		float *sHistPtr = sHist.ptr<float>();
		float *sDxHistPtr = sDxHist.ptr<float>();

		// for num max
		std::list<DkVector> maxPoints;

		// find local maxima in the smoothed orientation histogram
		for (int idx = 0, nextIdx = 1; idx < sDxHist.cols; idx++, nextIdx++) {

			// flip around
			nextIdx %= sDxHist.cols;

			// search for positive zero passing in the first derivative
			if (sHistPtr[idx] > cutOff && sDxHistPtr[idx] <= 0 && sDxHistPtr[nextIdx] > 0) {
				if (numMax != -1)
					maxPoints.push_back(DkVector((float)idx, sHistPtr[idx]));
				else
					localMaxList->push_back(idx);
			}

		}

		if (numMax != -1) {

			// DkVector sorts the y values if x1.y == x2.y than the x values are sorted
			maxPoints.sort();

			// vector is now sorted ascending
			std::list<DkVector>::reverse_iterator vecIter = maxPoints.rbegin();

			for (int idx = 0; idx < numMax && idx < (int)maxPoints.size(); idx++) {
				localMaxList->push_back((int)(*vecIter).x);
				vecIter++;
			}
		}

	}


	/**
	 * Finds local maxima in a given histogram.
	 * @param vec a 1xN histogram CV_32FC1.
	 * @param localMaxList a list where the indices of the local maxima are assigned.
	 * @param gSigma the standard deviation for pre-smoothing the histogram
	 * @param cutOffIvl a threshold which rejects weak local maxima. (absolute threshold value)
	 **/
	static void findLocalMaxima(Mat vec, 
								std::list<int> *localMaxList, 
								const float gSigma = 0.0f, 
								const int cutOffIvl = 20) {

		if (vec.channels() > 1) {
			std::string msg = "the vector needs to have 1 channel, but it has: " + 
				DkUtils::stringify(vec.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (vec.type() != CV_32FC1) {
			std::string msg = "the vector needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(vec);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat sHist;


		if (gSigma > 0)
			sHist = convolveSymmetric(vec, get1DGauss(gSigma));
		else
			sHist = vec;

		// find the threshold for local maxima
		float cutOff = (float)cutOffIvl;

		// perform first derivation
		float dxData[] = {1, -1};
		Mat dxKernel = Mat(1, 2, CV_32FC1, &dxData);
		Mat sDxHist = convolveSymmetric(sHist, dxKernel);
		
		float *sHistPtr = sHist.ptr<float>();
		float *sDxHistPtr = sDxHist.ptr<float>();
		//float *vecPtr = vec.ptr<float>();

		// find local maxima in the smoothed orientation histogram
		for (int idx = 0, nextIdx = 1; idx < sDxHist.cols; idx++, nextIdx++) {

			// flip around
			nextIdx %= sDxHist.cols;

			// search for positive zero passing in the first derivative
			if (sHistPtr[idx] > cutOff && sDxHistPtr[idx] <= 0 && sDxHistPtr[nextIdx] > 0) {

				//int pre, suc, max;


				//pre = idx-1 < 0 ? sDxHist.cols-1 : idx-1;
				//suc = nextIdx;

				//if (vecPtr[idx] > vecPtr[pre])
				//	max = (vecPtr[suc] > vecPtr[idx]) ? suc : idx;
				//else
				//	max = (vecPtr[suc] > vecPtr[pre]) ? suc : pre;

				//if (vecPtr[max] != 0) localMaxList->push_back(max);
				//if (vecPtr[max] != 0) printf("max: %i\n", max);
				localMaxList->push_back(idx);
			}
		}
	}

	/**
	 * Finds local maxima in a given image.
	 * @param src an image CV_32FC1.
	 * @param thr a threshold, local maxima below this threshold are rejected.
	 * @return a vector containing interest points (DkInterestPoint)
	 **/
	static vector<DkInterestPoint> findLocalMaxima2D(const Mat src, float thr) {

		if (src.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}


		vector<DkInterestPoint> ips = vector<DkInterestPoint>();

		for (int rIdx = 1; rIdx < src.rows-1; rIdx++) {

			const float* pr = src.ptr<float>(rIdx-1);		// previous row
			const float* cr = src.ptr<float>(rIdx);			// current row
			const float* nr = src.ptr<float>(rIdx+1);		// next row

			for (int cIdx = 1; cIdx < src.cols-1; cIdx++) {


				if (cr[cIdx] > thr			&&	// relevant pixel?
					cr[cIdx] > pr[cIdx-1]	&&	// upper left
					cr[cIdx] > pr[cIdx]		&&	// upper middle
					cr[cIdx] > pr[cIdx+1]	&&	// upper right
					cr[cIdx] > cr[cIdx-1]	&&	// middle left
					cr[cIdx] > cr[cIdx+1]	&&	// middle right
					cr[cIdx] > nr[cIdx-1]	&&	// lower left
					cr[cIdx] > nr[cIdx]		&&	// lower middle
					cr[cIdx] > nr[cIdx+1]) {	// lower right
					
						//interpolate spline horizontal
						int splIvlC[] = {cIdx-1, cIdx, cIdx+1};
						if (splIvlC[0] <  0 || splIvlC[2] == src.cols) continue;
						float ySplC[] = {cr[splIvlC[0]], cr[splIvlC[1]], cr[splIvlC[2]]};
						float xMaxC = 0.0f;
						float yMaxC = 0.0f;

						// empty image
						if (ySplC[1] == 0.0f)
							continue;

						DkMath::iplSplineEqDist((float)cIdx-1, ySplC, &xMaxC, &yMaxC);


						//interpolate spline vertical
						int splIvlR[] = {rIdx-1, rIdx, rIdx+1};
						if (splIvlR[0] <  0 || splIvlR[2] == src.cols) continue;
						float ySplR[] = {pr[cIdx], cr[cIdx], nr[cIdx]};
						float xMaxR = 0.0f;
						float yMaxR = 0.0f;

						// empty image
						if (ySplR[1] == 0.0f)
							continue;

						DkMath::iplSplineEqDist((float)rIdx-1, ySplR, &xMaxR, &yMaxR);
						
					ips.push_back(DkInterestPoint(xMaxC, xMaxR, cr[cIdx]));
				}
			}
		}
		return ips;
	}

	/**
	 * This method detects the histogram's global maximum with sub-pixel accuracy.
	 * If the global maximum is located at 0 or hist.cols, the interpolation is flipped around.
	 * @param hist a 1xN matrix CV_32FC1
	 * @return the global maximum's index
	 **/
	static double findIplMaximumSymmetric(const Mat hist) {


		if (hist.channels() > 1) {
			std::string msg = "the histogram needs to have 1 channel, but it has: " + 
				DkUtils::stringify(hist.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (hist.type() != CV_32FC1) {
			std::string msg = "the histogram needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(hist);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double gMin, gMax;
		Point gMinIdx, gMaxIdx;
		minMaxLoc(hist, &gMin, &gMax, &gMinIdx, &gMaxIdx);

		//interpolate spline
		const float *histPtr = hist.ptr<float>();
		int splIvl [] = {gMaxIdx.x-1, gMaxIdx.x, gMaxIdx.x+1};
		if (splIvl[0] < 0)			splIvl[0] = hist.cols-1;
		if (splIvl[2] == hist.cols) splIvl[2] = 1;
		float ySpl [] = {histPtr[splIvl[0]], histPtr[splIvl[1]], histPtr[splIvl[2]]};

		// could not compute main orientation since the orientation histogram is empty
		if (ySpl[1] == 0)
			return -1.0;

		// compute the center of mass
		// -2 since 2 is in the middle (1,2,3)
		float xMax = gMaxIdx.x-2.0f + (1.0f*ySpl[0] + 2.0f*ySpl[1] + 3.0f*ySpl[2])/(ySpl[0]+ySpl[1]+ySpl[2]);

		return xMax;
	}


	/**
	 * This method detects the histogram's global maximum with sub-pixel accuracy.
	 * @param hist a 1xN matrix CV_32FC1
	 * @return the global maximum's index
	 **/
	static double findIplMaximum(const Mat hist) {


		if (hist.channels() > 1) {
			std::string msg = "the histogram needs to have 1 channel, but it has: " + 
				DkUtils::stringify(hist.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (hist.type() != CV_32FC1) {
			std::string msg = "the histogram needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(hist);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double maxIdx;
		double gMin, gMax;
		Point gMinIdx, gMaxIdx;
		minMaxLoc(hist, &gMin, &gMax, &gMinIdx, &gMaxIdx);

		// interpolate the peak
		const float *histPtr = hist.ptr<float>();		
		if (gMaxIdx.x == 0)
			maxIdx = 0.5*histPtr[1]/(histPtr[0]+FLT_EPSILON);	// linear interpolation
		else if (gMaxIdx.x == hist.cols-1)
			maxIdx = hist.cols - 0.5*histPtr[hist.cols]/(histPtr[hist.cols-1]+FLT_EPSILON);	// linear interpolation
		else {

			//interpolate spline
			int splIvl [] = {gMaxIdx.x-1, gMaxIdx.x, gMaxIdx.x+1};
			float ySpl [] = {histPtr[splIvl[0]], histPtr[splIvl[1]], histPtr[splIvl[2]]};

			// could not compute main orientation since the orientation histogram is empty
			if (ySpl[1] == 0)
				return -1.0;

			maxIdx = gMaxIdx.x-2.0f + (1.0f*ySpl[0] + 2.0f*ySpl[1] + 3.0f*ySpl[2])/(ySpl[0]+ySpl[1]+ySpl[2]);
		}

		return maxIdx;
	}

	/**
	 * Extracts an image patch containing most background pixels.
	 * This method locates the brightest image region by means of integral convolution.
	 * Therefore the computation time is not dependent to the kernel's size newDim.
	 * @param img a grayscale image CV_32FC1.
	 * @param maskImg a grayscale (binary) image CV_32FC1.
	 * @param newDim the dimension of the patch.
	 * @return a rectangle Rect specifying the region with most background pixels.
	 **/
	static Rect extractImagePatch(const Mat img, const Mat maskImg, const int newDim) {

		if (img.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(img.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!maskImg.empty() && maskImg.channels() > 1) {
			std::string msg = "the mask needs to have 1 channel, but it has: " + 
				DkUtils::stringify(maskImg.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (img.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		DkTimer dt = DkTimer();	// stop

		// compute mean filter using fast integral image
		Mat bgImgInt;
		integral(img, bgImgInt);
		bgImgInt = convolveIntegralImage(bgImgInt, newDim, newDim, DK_BORDER_ZERO);

		DkIP::mulMask(bgImgInt, maskImg);

		// the maximum indicates the patch having the lowest content
		double min, max;
		Point minIdx, maxIdx;
		minMaxLoc(bgImgInt, &min, &max, &minIdx, &maxIdx);

		Rect patchBox;
		patchBox.x = (maxIdx.x-1) - DkMath::halfInt(newDim);	// x-1-dim/2
		patchBox.y = (maxIdx.y-1) - DkMath::halfInt(newDim);	// y-1-dim/2

		// some index correction
		if (img.cols-patchBox.x < newDim)
			patchBox.x = img.cols-newDim;
		if (img.rows-patchBox.y < newDim)
			patchBox.y = img.rows-newDim;

		if (patchBox.x < 0) patchBox.x = 0;
		if (patchBox.y < 0) patchBox.y = 0;

		patchBox.width  = (patchBox.x+newDim < img.cols) ? newDim : img.cols-patchBox.x;
		patchBox.height = (patchBox.y+newDim < img.rows) ? newDim : img.rows-patchBox.y;

		DkUtils::printDebug(DK_INFO, "Extract image patch computed in %s\n", dt.getTotal().c_str());

		return patchBox;

	}

	/**
	 * calculates the mean color of an patch, respective an corresponding mask. The mask pixels are ignored for the mean color value.
	 * @param patch image patch, where the mean color is calculated (3 channels input image).
	 * @param maskImg a grayscale (binary) image (1 channel input image).
	 * @param bgdMean the mean color value of the patch.
	 * @param bgdStd the standard deviation of the color value of the patch.
	 **/
	static void calcMeanColorPatch(const Mat patch, const Mat maskImg, Scalar& bgdMean, Scalar& bgdStd) {

		if (patch.channels() != 3) {
			std::string msg = "patch is not an rgb image, it is: " + DkUtils::getMatInfo(patch);
			throw DkMatException(msg, __LINE__, __FILE__);
		}
		if (maskImg.channels() != 1) {
			std::string msg = "the mask patch is not a single channel image, it is: " + DkUtils::getMatInfo(maskImg);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		////meanStdDev needs 8U mask
		//Mat tmpMask;
		//if (maskImg.depth != CV_8U) {

		//	maskImg.convertTo(tmpMask, CV_8U);
		//} else
		//	tmpMask = maskImg;
		//}

		Mat tmpPatch;
		if (patch.depth() != CV_32F)
			patch.convertTo(tmpPatch, CV_32F);
		else
			tmpPatch = patch;

		if (!maskImg.empty())
			meanStdDev(tmpPatch, bgdMean, bgdStd, maskImg);
		else
			meanStdDev(tmpPatch, bgdMean, bgdStd);

	}

	/**
	 * Generates a 32SC1 integral image.
	 * This method saves memory if an integral operation
	 * needs to be performed on a binary image.
	 * @param src a 32FC1 binary image (max == 1.0f).
	 * @return the corresponding 32SC1 integral image.
	 **/
	static Mat binaryIntegralImage(const Mat src) {

		if (src.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double mSum = src.rows*src.cols;
		if (mSum > UINT_MAX) {
			std::string msg = "the image is too big for our integral computation: " + 
				DkUtils::stringify(mSum) + " > " + DkUtils::stringify(FLT_MAX);
			throw DkMatException(msg, __LINE__, __FILE__);
			// TODO: do something (e.g. opencv integral)
		}

		Mat dst = Mat(src.rows+1, src.cols+1, CV_32SC1);
		
		unsigned int *lpx = dst.ptr<unsigned int>();	// last row current pixel
		unsigned int *cpx = dst.ptr<unsigned int>();		// current row current pixel
		const float *cpxSrc = src.ptr<float>();

		// first row
		for (int cIdx = 0; cIdx < dst.cols; cIdx++, cpx++)
			*cpx = 0;
		
		// compute the integral image
		for (int rIdx = 1; rIdx < dst.rows; rIdx++) {

			unsigned int sc = 0;
			*cpx = 0;	// first column
			cpx++; lpx++;
			
			// skip first column
			for (int cIdx = 1; cIdx < dst.cols; cIdx++, cpx++, lpx++, cpxSrc++) {

				sc += (unsigned int)*cpxSrc;
				*cpx = sc + *lpx;
				//printf("cpx: %i\n", *cpx);
			}
		}

		return dst;
	}

	/**
	 * Convolves an integral image by means of box filters.
	 * This functions applies box filtering. It is specifically useful for the computation
	 * of image sums, mean filtering and standard deviation with big kernel sizes.
	 * @param src an integral image CV_32FC1
	 * @param kernelSize the box filter's size
	 * @param morph DK_ERODE or DK_DILATE.
	 * @return the convolved image CV_8UC1
	 **/
	static Mat morphIntegral8U(const Mat src, int kernelSizeX, int kernelSizeY = 0, int morph = DK_ERODE) {

		if (src.empty()) {
			std::string msg = "the image assigned is empty";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32SC1) {
			std::string msg = "the image needs to be CV_32SC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat dst = Mat(src.rows-1, src.cols-1, CV_8UC1);
		
		int ksY = (kernelSizeY == 0) ? kernelSizeX : kernelSizeY;	// squared kernel		
		int halfKRows = (ksY < dst.rows) ? cvFloor((float)ksY*0.5)+1 : cvFloor((float)(dst.rows-1)*0.5)-1;
		int halfKCols = (kernelSizeX < dst.cols) ? cvFloor((float)kernelSizeX*0.5)+1 : cvFloor((float)(dst.cols-1)*0.5)-1;

		// if the image dimension (rows, cols) <= 2
		if (halfKRows <= 0 || halfKCols <= 0) {
			dst.setTo(0);
			return dst;
		}

		unsigned int maxArea = (halfKCols*2-1)*(halfKRows*2-1)*255;

		// pointer for all corners
		const unsigned int* llc = src.ptr<unsigned int>();
		const unsigned int* lrc = src.ptr<unsigned int>();
		const unsigned int* ulc = src.ptr<unsigned int>();
		const unsigned int* urc = src.ptr<unsigned int>();
		const unsigned int* origin = src.ptr<unsigned int>();
		const unsigned int* lastRow = src.ptr<unsigned int>();

		unsigned char *dstPtr = dst.ptr<unsigned char>();
		
		// initial positions
		lrc += halfKCols;
		ulc += halfKRows*src.cols;
		urc += halfKRows*src.cols+halfKCols;
		lastRow += (src.rows-1)*src.cols;
		
		DkTimer dt = DkTimer();

		for (int row = 0; row < dst.rows; row++) {

			for (int col = 0; col < dst.cols; col++) {

				// filter operation
				if (morph == DK_ERODE)
					*dstPtr = (*urc-*ulc-*lrc+*llc >= maxArea) ? 255 : 0;
				else if (morph == DK_DILATE)
					*dstPtr = (*urc-*ulc-*lrc+*llc > 0) ? 255 : 0;
				
				// do not change the left corners if we are near the left border
				if (col >= halfKCols-1) {
					llc++; ulc++;
				}

				// do not change the right corners if we are near the right border
				if (col < dst.cols-halfKCols) {
					lrc++; urc++;
				}

				dstPtr++;
			}

			// ok, flip to next row
			llc = ++lrc;
			ulc = ++urc;
			lrc += halfKCols;
			urc += halfKCols;

			if (row < halfKRows-1) {
				llc = origin;
				lrc = origin+halfKCols;
			}

			if (row >= dst.rows-halfKRows) {
				ulc = lastRow;
				urc = lastRow+halfKCols;
			}
		}
		
		DkUtils::printDebug(DK_DEBUG_INFO, "convolving integral image 32S in: %s\n", dt.getTotal().c_str());

		return dst;
	}

	/**
	 * Convolves an integral image by means of box filters.
	 * This functions applies box filtering. It is specifically useful for the computation
	 * of image sums, mean filtering and standard deviation with big kernel sizes.
	 * @param src an integral image CV_32SC1
	 * @param kernelSize the box filter's size
	 * @param morph DK_ERODE or DK_DILATE.
	 * @return the convolved image CV_32FC1
	 **/
	static Mat morphIntegral32F(const Mat src, int kernelSizeX, int kernelSizeY = 0, int morph = DK_ERODE) {

		if (src.empty()) {
			std::string msg = "the image assigned is empty";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32SC1) {
			std::string msg = "the image needs to be CV_32SC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat dst = Mat(src.rows-1, src.cols-1, CV_32FC1);
		
		int ksY = (kernelSizeY == 0) ? kernelSizeX : kernelSizeY;	// squared kernel		
		int halfKRows = (ksY < dst.rows) ? cvFloor((float)ksY*0.5)+1 : cvFloor((float)(dst.rows-1)*0.5)-1;
		int halfKCols = (kernelSizeX < dst.cols) ? cvFloor((float)kernelSizeX*0.5)+1 : cvFloor((float)(dst.cols-1)*0.5)-1;

		// if the image dimension (rows, cols) <= 2
		if (halfKRows <= 0 || halfKCols <= 0) {
			dst.setTo(0);
			return dst;
		}

		unsigned int maxArea = (halfKCols*2-1)*(halfKRows*2-1);

		// pointer for all corners
		const unsigned int* llc = src.ptr<unsigned int>();
		const unsigned int* lrc = src.ptr<unsigned int>();
		const unsigned int* ulc = src.ptr<unsigned int>();
		const unsigned int* urc = src.ptr<unsigned int>();
		const unsigned int* origin = src.ptr<unsigned int>();
		const unsigned int* lastRow = src.ptr<unsigned int>();

		float *dstPtr = dst.ptr<float>();
		
		// initial positions
		lrc += halfKCols;
		ulc += halfKRows*src.cols;
		urc += halfKRows*src.cols+halfKCols;
		lastRow += (src.rows-1)*src.cols;
		
		DkTimer dt = DkTimer();

		for (int row = 0; row < dst.rows; row++) {

			for (int col = 0; col < dst.cols; col++) {

				// filter operation
				if (morph == DK_ERODE)
					*dstPtr = (*urc-*ulc-*lrc+*llc >= maxArea) ? 1.0f : 0.0f;
				else if (morph == DK_DILATE)
					*dstPtr = (*urc-*ulc-*lrc+*llc > 0) ? 1.0f : 0.0f;
				
				// do not change the left corners if we are near the left border
				if (col >= halfKCols-1) {
					llc++; ulc++;
				}

				// do not change the right corners if we are near the right border
				if (col < dst.cols-halfKCols) {
					lrc++; urc++;
				}

				dstPtr++;
			}

			// ok, flip to next row
			llc = ++lrc;
			ulc = ++urc;
			lrc += halfKCols;
			urc += halfKCols;

			if (row < halfKRows-1) {
				llc = origin;
				lrc = origin+halfKCols;
			}

			if (row >= dst.rows-halfKRows) {
				ulc = lastRow;
				urc = lastRow+halfKCols;
			}
		}
		
		DkUtils::printDebug(DK_DEBUG_INFO, "convolving integral image 32S in: %s\n", dt.getTotal().c_str());

		return dst;
	}

	/**
	 * Convolves an integral image by means of box filters.
	 * This functions applies box filtering. It is specifically useful for the computation
	 * of image sums, mean filtering and standard deviation with big kernel sizes.
	 * @param src an integral image CV_64FC1
	 * @param kernelSize the box filter's size
	 * @param morph DK_ERODE or DK_DILATE.
	 * @return the convolved image CV_32FC1
	 **/
	static void fastMorphIntegral8U(const Mat src, Mat *dst,  int kernelSizeX, int kernelSizeY = 0, int morph = DK_ERODE) {

		if (src.channels() > 1) {
			std::string msg = "the source image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32SC1) {
			std::string msg = "the source image needs to be CV_32SC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (dst->type() != CV_8U) {
			std::string msg = "the destination image needs to be CV_8UC1, it is: " + 
				DkUtils::getMatInfo(*dst);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (DkVector(dst->size()) != DkVector(src.size())-1) {
			std::string msg = "the destination image needs to have src.rows-1 and src.cols-1, but it has src: " + 
				DkVector(src.size()).toString() + " dst: " + DkVector(dst->size()).toString();
			throw DkMatException(msg, __LINE__, __FILE__);

		}

		int ksY = (kernelSizeY == 0) ? kernelSizeX : kernelSizeY;	// squared kernel		
		int halfKRows = (ksY < dst->rows) ? cvFloor((float)ksY*0.5)+1 : cvFloor((float)(dst->rows-1)*0.5)-1;
		int halfKCols = (kernelSizeX < dst->cols) ? cvFloor((float)kernelSizeX*0.5)+1 : cvFloor((float)(dst->cols-1)*0.5)-1;
		int hC2 = halfKCols*2;
		int hC3 = halfKCols*3+1;

		// if the image dimension (rows, cols) <= 2
		if (halfKRows <= 0 || halfKCols <= 0)
			dst = 0;

		unsigned char mVal = 255;
		unsigned int maxArea = (halfKCols*2-1)*(halfKRows*2-1)*mVal;

		// pointer for all corners
		const unsigned int* llc = src.ptr<unsigned int>();
		const unsigned int* lrc = src.ptr<unsigned int>();
		const unsigned int* ulc = src.ptr<unsigned int>();
		const unsigned int* urc = src.ptr<unsigned int>();
		const unsigned int* origin = src.ptr<unsigned int>();
		const unsigned int* lastRow = src.ptr<unsigned int>();

		unsigned char *dstPtr = dst->ptr<unsigned char>();

		// initial positions
		lrc += halfKCols;
		ulc += halfKRows*src.cols;
		urc += halfKRows*src.cols+halfKCols;
		lastRow += (src.rows-1)*src.cols;

		for (int row = 0; row < dst->rows; row++) {

			for (int col = 0; col < dst->cols; col++) {

				unsigned int val = *urc-*ulc-*lrc+*llc;

				// skip the whole kernel size if all values are 0 or we leave the blob
				if (val == 0 /*|| (morph == DK_ERODE && *dstPtr == mVal && val < maxArea)*/) {

					// do we stay within the col range?
					if (col + hC3 <= dst->cols) {
						dstPtr += hC2;
						ulc = urc+1;
						llc = lrc+1;
						urc += hC2;
						lrc += hC2;
						col += hC2-1;
						continue;
					}
					// are we within the next step, at the image's border?
					else if (col + hC3 > dst->cols && col + hC2 < dst->cols){
						dstPtr += hC2;
						ulc = urc+1;
						llc = lrc+1;
						urc += dst->cols-col-halfKCols;
						lrc += dst->cols-col-halfKCols;
						col += hC2-1;
						continue;
					}
					// do we need to move to the next row?
					else if (col + halfKCols >= dst->cols) {
						dstPtr += dst->cols-col;
						ulc += dst->cols-col-halfKCols;
						llc += dst->cols-col-halfKCols;
						break;
					}
					// do we need to move to the next row?
					else {
						dstPtr += dst->cols-col;
						ulc += dst->cols-col-halfKCols;
						llc += dst->cols-col-halfKCols;
						urc += dst->cols-(col+halfKCols);
						lrc += dst->cols-(col+halfKCols);
						col += dst->cols-col-1;
						continue;
					}

				}

				// filter operation
				if (morph == DK_ERODE) {
					if (val >= maxArea) *dstPtr = mVal;
				}
				else if (morph == DK_DILATE) {
					if (val > 0) *dstPtr = mVal;
				}

				// do not change the left corners if we are near the left border
				if (col >= halfKCols-1) {
					llc++; ulc++;
				}

				// do not change the right corners if we are near the right border
				if (col < dst->cols-halfKCols) {
					lrc++; urc++;
				}

				dstPtr++;
			
			}

			// ok, flip to next row
			llc = ++lrc;
			ulc = ++urc;
			lrc += halfKCols;
			urc += halfKCols;

			if (row < halfKRows-1) {
				llc = origin;
				lrc = origin+halfKCols;
			}

			if (row >= dst->rows-halfKRows) {
				ulc = lastRow;
				urc = lastRow+halfKCols;
			}
		}
	}

	///**
	// * Convolves an integral image by means of box filters.
	// * This functions applies box filtering. It is specifically useful for the computation
	// * of image sums, mean filtering and standard deviation with big kernel sizes.
	// * @param src an integral image CV_32SC1
	// * @param kernelSize the box filter's size
	// * @return the convolved image CV_32SC1
	// **/
	//static Mat convolveIntegral8U(const Mat src, const int kernelSize) {

	//	if (src.channels() > 1) {
	//		std::string msg = "the image needs to have 1 channel, but it has: " + 
	//			DkUtils::stringify(src.channels());
	//		throw DkMatException(msg, __LINE__, __FILE__);
	//	}

	//	if (src.type() != CV_32SC1) {
	//		std::string msg = "the image needs to be CV_32SC1, it is: " + 
	//			DkUtils::getMatInfo(src);
	//		throw DkMatException(msg, __LINE__, __FILE__);
	//	}

	//	if (kernelSize < 3) DkUtils::printDebug(DK_WARNING, "[convolveIntegralImage8U] kernelsize is < 3, it is set to 3\n");

	//	Mat dst = Mat(src.rows-1, src.cols-1, CV_32SC1);
	//	
	//	int ks = (kernelSize >= 3) ? kernelSize : 3;
	//	int halfKRows = (ks < dst.rows) ? cvFloor((float)ks*0.5)+1 : cvFloor((float)(dst.rows-1)*0.5)-1;
	//	int halfKCols = (ks < dst.cols) ? cvFloor((float)ks*0.5)+1 : cvFloor((float)(dst.cols-1)*0.5)-1;


	//	// pointer for all corners
	//	const unsigned int* llc = src.ptr<unsigned int>();
	//	const unsigned int* lrc = src.ptr<unsigned int>();
	//	const unsigned int* ulc = src.ptr<unsigned int>();
	//	const unsigned int* urc = src.ptr<unsigned int>();
	//	const unsigned int* origin = src.ptr<unsigned int>();
	//	const unsigned int* lastRow = src.ptr<unsigned int>();

	//	unsigned int *dstPtr = dst.ptr<unsigned int>();
	//	
	//	// initial positions
	//	lrc += halfKCols;
	//	ulc += halfKRows*src.cols;
	//	urc += halfKRows*src.cols+halfKCols;
	//	lastRow += (src.rows-1)*src.cols;
	//
	//	DkTimer dt = DkTimer();

	//	for (int row = 0; row < dst.rows; row++) {

	//		for (int col = 0; col < dst.cols; col++) {

	//			// filter operation
	//			*dstPtr = *urc-*ulc-*lrc+*llc;
	//			
	//			// do not change the left corners if we are near the left border
	//			if (col >= halfKCols-1) {
	//				llc++; ulc++;
	//			}

	//			// do not change the right corners if we are near the right border
	//			if (col < dst.cols-halfKCols) {
	//				lrc++; urc++;
	//			}

	//			dstPtr++;
	//		}

	//		// ok, flip to next row
	//		llc = ++lrc;
	//		ulc = ++urc;
	//		lrc += halfKCols;
	//		urc += halfKCols;

	//		if (row < halfKRows-1) {
	//			llc = origin;
	//			lrc = origin+halfKCols;
	//		}

	//		if (row >= dst.rows-halfKRows) {
	//			ulc = lastRow;
	//			urc = lastRow+halfKCols;
	//		}
	//	}
	//	
	//	DkUtils::printDebug(DK_DEBUG_B, "convolving integral image 32S in: %s\n", dt.getTotal().c_str());

	//	return dst;
	//}

	/**
	 * Convolves an integral image by means of box filters.
	 * This functions applies box filtering. It is specifically useful for the computation
	 * of image sums, mean filtering and standard deviation with big kernel sizes.
	 * @param src an integral image CV_64FC1
	 * @param kernelSize the box filter's size
	 * @param norm if DK_BORDER_ZERO an image sum is computed, if DK_BORDER_FLIP a mean filtering is applied.
	 * @return the convolved image CV_32FC1
	 **/
	static Mat convolveIntegralImage(const Mat src, const int kernelSizeX, const int kernelSizeY = 0, const int norm = DK_BORDER_ZERO) {

		if (src.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_64FC1) {
			std::string msg = "the image needs to be CV_64FC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		int ksY = (kernelSizeY != 0) ? kernelSizeY : kernelSizeX;	// make squared kernel

		Mat dst = Mat(src.rows-1, src.cols-1, CV_32FC1);
		
		int halfKRows = (ksY < dst.rows) ? cvFloor((float)ksY*0.5)+1 : cvFloor((float)(dst.rows-1)*0.5)-1;
		int halfKCols = (kernelSizeX < dst.cols) ? cvFloor((float)kernelSizeX*0.5)+1 : cvFloor((float)(dst.cols-1)*0.5)-1;

		// if the image dimension (rows, cols) <= 2
		if (halfKRows <= 0 || halfKCols <= 0) {
			dst.setTo(0);
			return dst;
		}

		// pointer for all corners
		const double* llc = src.ptr<double>();
		const double* lrc = src.ptr<double>();
		const double* ulc = src.ptr<double>();
		const double* urc = src.ptr<double>();
		const double* origin = src.ptr<double>();
		const double* lastRow = src.ptr<double>();

		float *dstPtr = dst.ptr<float>();
		
		// initial positions
		lrc += halfKCols;
		ulc += halfKRows*src.cols;
		urc += halfKRows*src.cols+halfKCols;
		lastRow += (src.rows-1)*src.cols;

		// area computation
		float rs = (float)halfKRows;
		float cs = (float)halfKCols;
		float area = rs*cs;
		
		DkTimer dt = DkTimer();

		for (int row = 0; row < dst.rows; row++) {

			for (int col = 0; col < dst.cols; col++) {

				// filter operation
				if (norm == DK_BORDER_ZERO)
					*dstPtr = (float)(*urc-*ulc-*lrc+*llc);
				else if (norm == DK_BORDER_FLIP) {
					*dstPtr = (float)((*urc-*ulc-*lrc+*llc)/area);
				}
				
				// do not change the left corners if we are near the left border
				if (col >= halfKCols-1) {
					llc++; ulc++;
				}
				// but recompute the filter area near the border
				else if (norm == DK_BORDER_FLIP) {
					cs++;
					area = rs*cs;
				}

				// do not change the right corners if we are near the right border
				if (col < dst.cols-halfKCols) {
					lrc++; urc++;
				}
				else if (norm == DK_BORDER_FLIP && col != dst.cols-1) {
					cs--;
					area = rs*cs;
				}

				dstPtr++;
			}

			// ok, flip to next row
			llc = ++lrc;
			ulc = ++urc;
			lrc += halfKCols;
			urc += halfKCols;

			if (row < halfKRows-1) {
				llc = origin;
				lrc = origin+halfKCols;
				if (norm == DK_BORDER_FLIP) {
					rs++;
					area = rs*cs;
				}
			}

			if (row >= dst.rows-halfKRows) {
				ulc = lastRow;
				urc = lastRow+halfKCols;
				if (norm == DK_BORDER_FLIP) {
					rs--;
					area = rs*cs;
				}
			}
		}
		
		DkUtils::printDebug(DK_DEBUG_INFO, "convolving integral image in: %s\n", dt.getTotal().c_str());
		//normalize(dst, dst, 1, 0, NORM_MINMAX);

		return dst;
	}

	//static void imgStats(const Mat img, double *avg, double *std, const Mat mask) {

	//	DkTimer dt = DkTimer();

	//	// compute gradient magnitude
	//	int cols = img.cols;
	//	int rows = img.rows;

	//	// speed up for accessing elements
	//	if(img.isContinuous()) {
	//		cols *= rows;
	//		rows = 1;
	//	}

	//	double sum0 = 0;
	//	double sum1 = 0;
	//	double sum2 = 0;

	//	for (int rIdx = 0; rIdx < rows; rIdx++) {

	//		const float* imgPtr = img.ptr<float>(rIdx);

	//		const float* maskPtr = 0;

	//		if (!mask.empty())
	//			maskPtr = mask.ptr<float>(rIdx);

	//		for (int cIdx = 0; cIdx < cols; cIdx++) {

	//			if (mask.empty() || maskPtr[cIdx] > 0) {
	//				sum0++;
	//				sum1 += imgPtr[cIdx];
	//				sum2 += (imgPtr[cIdx]*imgPtr[cIdx]);
	//			}
	//		}
	//	}

	//	*avg = 1/sum0 * sum1;
	//	*std = 1/sum0 * sqrt(sum0*sum2 - sum1*sum1);

	//	if (DEBUG >= 3)
	//		printf(">> avg & std computed in: %s\n", dt.getTotal().c_str());
	//}

	/**
	 * Applies Otsu's threshold to the given image.
	 * @param src a grayscale image (fastest: CV_32FC1)
	 * @param mask a grayscale (binary) mask image (fastest: CV_32FC1)
	 * @param otsuThresh deprecated
	 * @return the thresholded image CV_32FC1
	 **/
	static Mat thresholdImageOtsu(const Mat src, const Mat mask = Mat()) {

		DkTimer dt = DkTimer();

		// check inputs
		if (src.channels() != 1 || !mask.empty() && mask.channels() != 1) {
			std::string msg = "The image has: " + DkUtils::stringify(src.channels()) + ", but 1 channel is required\n";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// convert the source image
		Mat imgF = src.clone();
		if(src.depth() != CV_32F)
			src.convertTo(imgF, CV_32F, 1/255.0f);
		normalize(imgF, imgF, 1, 0, NORM_MINMAX);

		// convert the mask image
		Mat maskF = mask;
		
		if (mask.empty()) {
			if (mask.depth() != CV_32F)
				mask.convertTo(maskF, CV_32F, 1/255.0f);
		}

		// compute histogram & threshold the image
		Mat hist = computeHist(imgF, maskF);
		double thresh = getThreshOtsu(hist);
		threshold(imgF, imgF, thresh/255.0f, 1.0f, THRESH_BINARY);

		DkUtils::printDebug(DK_DEBUG_INFO, "otsu threshold: %.4f computed in: %s\n", thresh/255.0f, dt.getTotal().c_str());

		return imgF;

	}

	/**
	 * Computes the image's histogram.
	 * @param img a grayscale image CV_32FC1
	 * @param mask a grayscale (binary) mask CV_32FC1
	 * @return the images histogram CV_32FC1 [0 255]
	 **/
	static Mat computeHist(const Mat img, const Mat mask=Mat()) {

		if (img.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(img.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty() && mask.channels() > 1) {
			std::string msg = "the mask needs to have 1 channel, but it has: " + 
				DkUtils::stringify(mask.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (img.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty() && !(mask.type() == CV_32FC1 || mask.type() == CV_8UC1)) {
			std::string msg = "the mask needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(mask);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// compute gradient magnitude
		int cols = img.cols;
		int rows = img.rows;

		// speed up for accessing elements
		if(img.isContinuous()) {
			cols *= rows;
			rows = 1;
		}

		Mat hist = Mat(1, 256, CV_32FC1);
		hist.setTo(0);

		for (int rIdx = 0; rIdx < rows; rIdx++) {

			const float* imgPtr = img.ptr<float>(rIdx);
			const float* maskPtr32F = 0;
			const unsigned char* maskPtr8U = 0;

			float* histPtr = hist.ptr<float>();

			if (!mask.empty() && mask.depth() == CV_32F) {
				maskPtr32F = mask.ptr<float>(rIdx);
			}
			else if (!mask.empty() && mask.depth() == CV_8U) {
				maskPtr8U = mask.ptr<unsigned char>(rIdx);
			}

			for (int cIdx = 0; cIdx < cols; cIdx++) {

				if (mask.empty() || 
					(mask.depth() == CV_32F && maskPtr32F[cIdx] > 0) ||
					(mask.depth() == CV_8U  && maskPtr8U[cIdx]	> 0)) {
					int hIdx = cvFloor(imgPtr[cIdx]*255);
					//printf("%.3f, %i\n", imgPtr[cIdx], hIdx);
					if (hIdx >= 0 && hIdx < 256)	// -> bug in normalize!
						histPtr[hIdx] ++;
				}
			}
		}

		return hist;
	}

	
	/**
	 * Computes the image's histogram - a generic histogram without limit to [0 255].
	 * @param img a grayscale image CV_32FC1
	 * @param minimum an float, giving the minimum bin
	 * @param maximum an float, giving the maximum bin 
	 * @param numBin an float, giving the number of bins
	 * @param mask a grayscale (binary) mask CV_32FC1
	 * @return the images histogram CV_32FC1 
	 **/
	static Mat computeGenericHist(Mat src, float minimum, float maximum, float numBin, const Mat mask=Mat()) {
		Mat img = src.clone();

		if (img.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(img.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty() && mask.channels() > 1) {
			std::string msg = "the mask needs to have 1 channel, but it has: " + 
				DkUtils::stringify(mask.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (img.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty() && !(mask.type() == CV_32FC1 || mask.type() == CV_8UC1)) {
			std::string msg = "the mask needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(mask);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// compute gradient magnitude
		int cols = img.cols;
		int rows = img.rows;

		// speed up for accessing elements
		if(img.isContinuous()) {
			cols *= rows;
			rows = 1;
		}

		Mat hist = Mat(1,  cvRound(numBin), CV_32FC1);
		hist.setTo(0);

		float perBin =  (maximum - minimum)/numBin;

		for (int rIdx = 0; rIdx < rows; rIdx++) {

			const float* imgPtr = img.ptr<float>(rIdx);
			const float* maskPtr32F = 0;
			const unsigned char* maskPtr8U = 0;

			float* histPtr = hist.ptr<float>();

			if (!mask.empty() && mask.depth() == CV_32F) {
				maskPtr32F = mask.ptr<float>(rIdx);
			}
			else if (!mask.empty() && mask.depth() == CV_8U) {
				maskPtr8U = mask.ptr<unsigned char>(rIdx);
			}

			for (int cIdx = 0; cIdx < cols; cIdx++) {

				if (mask.empty() || 
					(mask.depth() == CV_32F && maskPtr32F[cIdx] > 0) ||
					(mask.depth() == CV_8U  && maskPtr8U[cIdx]	> 0)) {
				    

					int hIdx = 0;
					if (imgPtr[cIdx] < FLT_MAX) 
						hIdx = cvRound(imgPtr[cIdx]/perBin - (minimum)/perBin); // TODO: 20110819 floating point invalid operation

					if (hIdx >= 0 && hIdx < (int) numBin){	// -> bug in normalize!
						//printf("%d \n", hIdx - cvFloor(minimum/perBin));
						histPtr[hIdx] ++; // shifting dot distance into histogram
					}  
				}
			}
		}

		return hist;
	}

	template <typename num>
	static void minMax(const Mat& src, Mat& minVec, Mat& maxVec, int dim = DK_S_DIM_X) {

		minVec = (dim == DK_S_DIM_X) ? Mat(1, src.cols, src.type()) : Mat(src.rows, 1, src.type());
		maxVec = (dim == DK_S_DIM_X) ? Mat(1, src.cols, src.type()) : Mat(src.rows, 1, src.type());

		minVec = std::numeric_limits<num>::max();
		maxVec = std::numeric_limits<num>::min();

		num* minPtr = minVec.ptr<num>();
		num* maxPtr = maxVec.ptr<num>();

		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			const num* srcPtr = src.ptr<num>(rIdx);

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				int cmpIdx = (dim == DK_S_DIM_X) ? cIdx : rIdx;

				if (srcPtr[cIdx] < minPtr[cmpIdx])
					minPtr[cmpIdx] = srcPtr[cIdx];
				if (srcPtr[cIdx] > maxPtr[cmpIdx])	// no else if since an element can be min & max at the same time
					maxPtr[cmpIdx] = srcPtr[cIdx];
			}
		}

	}

	//static Mat histEqualization(const Mat img, const Mat mask=Mat()) {

	//	if (img.channels() > 1) {
	//		std::string msg = "the image needs to have 1 channel, but it has: " + 
	//			DkUtils::stringify(img.channels());
	//		throw DkMatException(msg, __LINE__, __FILE__);
	//	}

	//	if (img.type() != CV_32FC1) {
	//		std::string msg = "the image needs to be CV_32FC1, it is: " + 
	//			DkUtils::getMatInfo(img);
	//		throw DkMatException(msg, __LINE__, __FILE__);
	//	}

	//	Mat hist = computeHist(img, mask);
	//	DkUtils::printMatCmd(hist, "hist");
	//	Mat cdf = hist.clone();
	//	float nVal = 0.0f;
	//	float *cpx = cdf.ptr<float>();
	//	float *lpx = &nVal;

	//	// compute the cumulative distribution function
	//	for (int cIdx = 0; cIdx < hist.cols; cIdx++, cpx++) {
	//		*cpx += *lpx;
	//		lpx = cpx;
	//	}

	//	Mat dst = img.clone();
	//	float *cdfPtr = cdf.ptr<float>();
	//	cpx = dst.ptr<float>();

	//	float s = (float)(dst.rows*dst.cols);
	//	double min, max;
	//	minMaxLoc(cdf, &min, &max);

	//	// let's compute the histogram equalization
	//	for (int rIdx = 0; rIdx < dst.rows*dst.cols; rIdx++, cpx++) {

	//		int hIdx = cvFloor(*cpx*255);
	//		if (hIdx <= 0)	hIdx = 0;
	//		if (hIdx > 255) hIdx = 255;

	//		*cpx = (cdfPtr[hIdx]-(float)min)/(s-(float)min);
	//	}

	//	return dst;
	//}


	/**
	 * Computes Otsu's threshold for a given histogram
	 * @param hist an image histogram CV_32FC1 [0 255]
	 * @param otsuThresh deprecated
	 * @return the computed threshold
	 **/
	static double getThreshOtsu(const Mat hist, double* meanLow = 0, double* meanHigh = 0) {

		if (hist.channels() > 1) {
			std::string msg = "the histogram needs to have 1 channel, but it has: " + 
				DkUtils::stringify(hist.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (hist.type() != CV_32FC1) {
			std::string msg = "the histogram needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(hist);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double max_val = 0;

		int i, count;
		const float* h;
		double sum = 0, mu = 0;
		bool uniform = false;
		double low = 0, high = 0, delta = 0;
		double mu1 = 0, mu2 = 0;
		double q1 = 0;
		double max_sigma = 0;

		count = hist.cols;
		h = hist.ptr<float>();

		low = 0;
		high = count;

		delta = (high-low)/count;
		low += delta*0.5;
		uniform = true;

		for( i = 0; i < count; i++ ) {
			sum += h[i];
			mu += (i*delta + low)*h[i];
		}

		sum = fabs(sum) > FLT_EPSILON ? 1./sum : 0;
		mu *= sum;
		q1 = 0;

		for( i = 0; i < count; i++ ) {
			double p_i, q2, val_i, sigma;
			p_i = h[i]*sum;
			mu1 *= q1;
			q1 += p_i;
			q2 = 1. - q1;

			if( MIN(q1,q2) < FLT_EPSILON || MAX(q1,q2) > 1. - FLT_EPSILON )
				continue;

			val_i = i*delta + low;

			mu1 = (mu1 + val_i*p_i)/q1;
			mu2 = (mu - q1*mu1)/q2;
			sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
			if( sigma > max_sigma ) {
				max_sigma = sigma;
				max_val = val_i;
			}
		}

		if (meanLow)
			*meanLow = mu1/255.0;
		if (meanHigh)
			*meanHigh = mu2/255.0;

		//// shift threshold if the contrast is low (no over segmentation)
		//double max_val_shift = (max_val-0.2 > 0) ? max_val-0.2 : 0;

		//return (max_sigma >= otsuThresh) ? max_val : max_val_shift; // 0.0007 (for textRectangles)
		return max_val;
	}


	/**
	 * Computes the image's power spectrum.
	 * This method transforms the image to the Fourier domain and then computes
	 * its power spectrum (sqrt(re*re + im*im)). It is particularly fast because
	 * solely two quadrants of the FFT are computed and packed complex conjugates
	 * are exploited.
	 * @param img an image to be transformed CV_32FC1 (fast if rows = 2^n & cols = 2^n)
	 * @return the image's power spectrum CV_32FC1
	 **/
	static Mat computePowerSpectrum(const Mat img) {

		if (img.channels() > 1) {
			std::string msg = "the image needs to have 1 channel, but it has: " + 
				DkUtils::stringify(img.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (img.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat fftImg;

		// append zeros
		if (!DkMath::isPowerOfTwo(img.cols) || !DkMath::isPowerOfTwo(img.rows) || img.rows != img.cols) {

			int fftM = max(img.rows, img.cols);
			fftM = DkMath::getNextPowerOfTwo(fftM);

			fftImg = Mat(fftM, fftM, img.depth());
			fftImg.setTo(0);

			Mat fftImgRoi = fftImg(Rect(0, 0, img.cols, img.rows));
			img.copyTo(fftImgRoi);
		}
		else
			fftImg = img.clone();

		dft(fftImg, fftImg, 0, img.rows);

		Mat powSpec = Mat(fftImg.rows, DkMath::halfInt(fftImg.cols)-1, fftImg.depth());
		double re = 0, im = 0;

		// compute the power spectrum of the 1st & 2nd quadrant
		for (int rIdx = 0; rIdx < powSpec.rows; rIdx++) {

			const float* fftPtr = fftImg.ptr<float>(rIdx);
			float* powPtr = powSpec.ptr<float>(rIdx);

			// ignore the first row
			for (int cIdx = 0, fIdx = 1; cIdx < powSpec.cols; cIdx++, fIdx+=2) {

				// get the real & imaginary part from the packed complex conjugates
				re = fftPtr[fIdx]+fftPtr[fIdx+1];
				im = fftPtr[fIdx+1]-fftPtr[fIdx];
				powPtr[cIdx] = (float)sqrt(re*re + im*im);
			}
		}

		// FFT shift (switch the quadrants)
		fftImg = powSpec.clone();
		Mat uq = powSpec(Rect(0, 0, powSpec.cols, DkMath::halfInt(powSpec.rows)));
		Mat lq = fftImg(Rect(0, DkMath::halfInt(powSpec.rows), powSpec.cols, DkMath::halfInt(powSpec.rows)));
		lq.copyTo(uq);
		uq = fftImg(Rect(0, 0, powSpec.cols, DkMath::halfInt(powSpec.rows)));
		lq = powSpec(Rect(0, DkMath::halfInt(powSpec.rows), powSpec.cols, DkMath::halfInt(powSpec.rows)));
		uq.copyTo(lq);

		// transpose the power spectrum -> (for polar transform)
		powSpec = powSpec.t();

		//normalize(powSpec, powSpec, 0, 1, NORM_MINMAX);

		return powSpec;
	}

	/**
	 * Applies a polar transform to the input image.
	 * @param src input image CV_32FC1.
	 * @param dst the polar transformed image CV_32FC1.
	 * @param center the center for the polar transformation.
	 * @param rmax maximal radius it is automatically selected if rmax <= 0.0
	 **/
	static void polarTransform(const Mat src, Mat dst, Point center, double rmax = 0.0) {

		if (src.channels() > 1) {
			std::string msg = "the images needs to have 1 channel, but it has: " + 
				DkUtils::stringify(src.channels());
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.type() != CV_32FC1) {
			std::string msg = "the image needs to be CV_32FC1, it is: " + 
				DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat mapx = Mat(dst.size(), dst.type());
		Mat mapy = Mat(dst.size(), dst.type());

		if (rmax <= 0) {
			double dx = (center.x > src.cols-center.x) ? center.x : src.cols-center.x;
			double dy = (center.y > src.rows-center.y) ? center.y : src.rows-center.y;
			rmax = (dx > dy) ? dx/2.0 : dy/2.0;	// x/2.0 -> scaling (high fft frequencies are neglectable)
		}

		Mat expTab = Mat(1, dst.cols, dst.depth());
		float* expPtr = expTab.ptr<float>();

		// exp table
		for(int rho = 0; rho < dst.cols; rho++)
			expPtr[rho] = (float)(rho*rmax/dst.cols);

		// polar grid mapping
		for(int phi = 0; phi < dst.rows; phi++) {

			double cp = cos(phi*CV_PI/dst.rows);
			double sp = sin(phi*CV_PI/dst.rows);
			float* mx = mapx.ptr<float>(phi);
			float* my = mapy.ptr<float>(phi);

			for(int rho = 0; rho < dst.cols; rho++) {

				double r = expPtr[rho];
				double x = r*cp + center.x;
				double y = r*sp + center.y;

				mx[rho] = (float)x;
				my[rho] = (float)y;
			}
		}

		// image transformation
		remap(src, dst, mapx, mapy, INTER_CUBIC);

	}

	/**
	 * Accumulates the image's columns.
	 * @param src the source image.
	 * @return the image's line histogram.
	 **/
	static Mat lineHistogram(const Mat src) {	

		Mat rowMul	= Mat(src.cols, 1, src.type());
		Mat hist	= Mat(src.rows, 1, src.type());

		rowMul.setTo(1);
		gemm(src, rowMul, 1, Mat(), 0, hist);

		return hist;
	}

	/**
	 * Draws the row/col histogram into the image.
	 * @param src a source image
	 * @param hist a row/col histogram where hist.cols == src.rows or hist.cols == src.cols
	 * @param col the bin color
	 * @return cv::Mat the image with the histogram visualized
	 **/ 
	static Mat drawHist(const Mat src, const Mat hist, float col = 0.3) {

		if (src.depth() != CV_32FC1) {
			std::string msg = "Mat must be CV_32FC1: " + DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		// transpose hist if it is Nx1
		Mat nHist = hist.clone();
		if (nHist.cols == 1)
			nHist = nHist.t();

		// transpose image
		Mat dst = src.clone();
		if (nHist.cols == dst.cols)
			dst = dst.t();

		if (nHist.cols != dst.rows) {
			std::string msg = "Histogram cols must be equal to src.rows or src.cols. it is: " + DkUtils::stringify(nHist.cols) 
				+ " mat dim: " + DkUtils::stringify(dst.rows) + std::string(" x ") + DkUtils::stringify(dst.cols);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		int magnitude = 500;
		if (magnitude > dst.cols) magnitude = dst.cols;

		nHist.convertTo(nHist, CV_32FC1);
		normalize(nHist, nHist, magnitude, 0.0, NORM_MINMAX);

		const float* histPtr = nHist.ptr<float>();


		for (int rIdx = 0; rIdx < dst.rows; rIdx++) {

			int val = (int)histPtr[rIdx];
			float* imgPtr = dst.ptr<float>(rIdx);

			for (int cIdx = 0; cIdx < dst.cols; cIdx++) {

				if (cIdx > val)
					break;

				imgPtr[cIdx] = col;
			}
		}

		if (nHist.cols == src.cols)
			dst = dst.t();

		return dst;
	}

	static void drawHistToImage(Mat& img, const Mat& hist, const DkBox& box, const Scalar& col) {

		if (box.getWidth() <= 0 || box.getHeight() <= 0)
			return;
		
		DkBox boxC = box; // TODO: this box might have a size < 0
		boxC.clip(img.size());

		double maxVal = 0;
		minMaxLoc(hist, 0, &maxVal);
		const float* histPtr = hist.ptr<float>();

		for (int idx = 0; idx < hist.cols; idx++) {

			float dx = boxC.uc.x+idx*boxC.size().width/hist.cols;
			DkVector p1(dx, boxC.lc.y-1);
			DkVector p2(dx, boxC.lc.y-histPtr[idx]*boxC.size().height/(float)maxVal);

			line(img, p1.getCvPoint(), p2.getCvPoint(), col);
		}
	}



	/**
	 * Rescales the histogram.
	 * @param src the source histogram.
	 * @param binFactor the number of bins that are merged
	 * @return the scaled histogram.
	 **/
	static Mat binHistogram(Mat src, int binFactor) {

		if (src.depth() != CV_32F) {
			std::string msg = "The src histogram has not the type CV_32F: " + DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if ((src.cols!=1) || (src.rows<binFactor)) {
			std::string msg = "The binFactor > rows or the number of columns is greater 1: " + DkUtils::stringify(src.cols);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		int newSize = (int)ceil((float)src.rows/(float)binFactor);

		Mat tmp = Mat(newSize, 1, CV_32FC1);
		tmp.setTo(0);

		float *srcPtr = src.ptr<float>(0);
		float *dstPtr = tmp.ptr<float>(0);
		for (int i=0; i<src.rows; i++) {
			dstPtr[i/binFactor] += srcPtr[i];
		}

		tmp /= (float)binFactor;

		return tmp;
	}

	/**
	 * Computes robust statistical moments of an image.
	 * The quantiles of an image (or median) are computed.
	 * @param src the source image CV_32FC1.
	 * @param mask the corresponding mask CV_32FC1 or CV8U_C1.
	 * @param momentValue the moment (e.g. 0.5 for median, 0.25 or 0.75 for quartiles).
	 * @param maxSamples the maximum number of samples (speed-up).
	 * @param area the mask's area (speed-up).
	 * @return the statistical moment.
	 **/
	static float statMomentMat(	const Mat src, 
								Mat mask = Mat(), 
								float momentValue = 0.5f, 
								int maxSamples = 10000,
								int area = -1) {

		// check input
		if (src.type() != CV_32FC1) {
			std::string msg = "Mat must be CV_32FC1 but it is: " + DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty()) {
			if (src.rows != mask.rows || src.cols != mask.cols) {
				std::string msg = "Matrix dimension mismatch: \n  img  (" + 
					DkUtils::stringify(src.rows) + " x " + DkUtils::stringify(src.cols) + ")\n  mask (" +
					DkUtils::stringify(mask.rows) + " x " + DkUtils::stringify(mask.cols) + ")\n";
				throw DkMatException(msg, __LINE__, __FILE__);
			}

			if (mask.depth() != CV_32F && mask.depth() != CV_8U) {
				std::string msg = "The mask obtained is neither of type CV_32F nor CV_8U: " + DkUtils::getMatInfo(mask);
				throw DkMatException(msg, __LINE__, __FILE__);
			}
		}

		// init output list
		std::list<float> samples = std::list<float>();

		// assign the step size
		if (mask.empty()) {
			int step = cvRound((src.cols*src.rows)/(float)maxSamples);
			if (step <= 0) step = 1;

			for (int rIdx = 0; rIdx < src.rows; rIdx += step) {

				const float* srcPtr = src.ptr<float>(rIdx);

				for (int cIdx = 0; cIdx < src.cols; cIdx += step) {
					samples.push_back(srcPtr[cIdx]);
				}
			}
		}
		else {

			if (area == -1)
				area = countNonZero(mask);

			int step = cvRound((float)area/(float)maxSamples);
			int cStep = 0;

			const void *maskPtr;
						
			if (step <= 0) step = 1;

			for (int rIdx = 0; rIdx < src.rows; rIdx++) {

				const float* srcPtr = src.ptr<float>(rIdx);
				if (mask.depth() == CV_32F) 
					maskPtr = mask.ptr<float>(rIdx);
				else 
					maskPtr = mask.ptr<uchar>(rIdx);
				//maskPtr = (mask.depth() == CV_32F) ? mask.ptr<float>(rIdx) : maskPtr = mask.ptr<uchar>(rIdx);

				for (int cIdx = 0; cIdx < src.cols; cIdx++) {
					
					// skip mask pixel
					if (mask.depth() == CV_32FC1	&& ((float*)maskPtr)[cIdx] != 0.0f ||
						mask.depth() == CV_8U		&& ((uchar*)maskPtr)[cIdx] != 0) {

						if (cStep >= step) {
							samples.push_back(srcPtr[cIdx]);
							cStep = 0;
						}
						else
							cStep++;
					}
				}
			}
		}
		
		return (float)DkMath::statMoment(&samples, momentValue);
	}

	template <typename num>
	static int countPixel(const Mat& img, num max = std::numeric_limits<num>::epsilon, num min = 0) {

		int numVals = 0;
		for (int rIdx = 0; rIdx < img.rows; rIdx++) {

			const num* imgPtr = img.ptr<num>(rIdx);

			for (int cIdx = 0; cIdx < img.cols; cIdx++) {

				if (imgPtr[cIdx] >= min && imgPtr[cIdx] <= max)
					numVals++;
			}
		}

		return numVals;
	}

	///**
	// * Computes robust statistical moments of an image.
	// * The quantiles of an image (or median) are computed.
	// * @param src the source image CV_32FC1.
	// * @param mask the corresponding mask CV_32FC1 or CV8U_C1.
	// * @param momentValue the moment (e.g. 0.5 for median, 0.25 or 0.75 for quartiles).
	// * @param area the mask's area (speed-up).
	// * @return the statistical moment.
	// **/
	//static float statMomentMatMask(	const Mat src, 
	//							Mat mask, 
	//							float momentValue = 0.5f, 
	//							int area = -1) {

	//	// check input
	//	if (src.type() != CV_32FC1) {
	//		std::string msg = "Mat must be CV_32FC1 but it is: " + DkUtils::getMatInfo(src);
	//		throw DkMatException(msg, __LINE__, __FILE__);
	//	}

	//	if (!mask.empty()) {
	//		if (src.rows != mask.rows || src.cols != mask.cols) {
	//			std::string msg = "Matrix dimension mismatch: \n  img  (" + 
	//				DkUtils::stringify(src.rows) + " x " + DkUtils::stringify(src.cols) + ")\n  mask (" +
	//				DkUtils::stringify(mask.rows) + " x " + DkUtils::stringify(mask.cols) + ")\n";
	//			throw DkMatException(msg, __LINE__, __FILE__);
	//		}

	//		if (mask.depth() != CV_32F && mask.depth() != CV_8U) {
	//			std::string msg = "The mask obtained is neither of type CV_32F nor CV_8U: " + DkUtils::getMatInfo(mask);
	//			throw DkMatException(msg, __LINE__, __FILE__);
	//		}
	//	}

	//	// init output list
	//	std::list<float> samples = std::list<float>();

	//	// assign the step size
	//	if (area == -1)
	//		area = countNonZero(mask);

	//	const void *maskPtr;
	//		

	//	for (int rIdx = 0; rIdx < src.rows; rIdx++) {

	//		const float* srcPtr = src.ptr<float>(rIdx);
	//		maskPtr = (mask.depth() == CV_32F) ? mask.ptr<float>(rIdx) : maskPtr = mask.ptr<uchar>(rIdx);

	//		for (int cIdx = 0; cIdx < src.cols; cIdx++) {
	//			
	//			// skip mask pixel
	//			if (mask.depth() == CV_32FC1	&& ((float*)maskPtr)[cIdx] != 0.0f ||
	//				mask.depth() == CV_8U		&& ((uchar*)maskPtr)[cIdx] != 0) {

	//				samples.push_back(srcPtr[cIdx]);
	//			}
	//		}
	//	}

	//	
	//	return (float)DkMath::statMoment(&samples, momentValue);
	//}


	/**
	 * Computes the distance between successive local maxima.
	 *
	 * @param src 1xN Mat, a line histogram CV_32FC1.
	 * @param maxDiffList a list containing the successive peak distances computed.
	 * @param gSigma sigma for the 1D gaussian smoothing.
	 * @param cutOffIvl	threshold which rejects low local maxima.
	 * 
	 **/
	static void lineHistPeakDist(Mat src, std::list<float> *maxDiffList, float gSigma, float cutOffIvl) {
		
		// remove low frequencies
		Mat g = DkIP::get1DGauss(9);
		Mat vec;
		filter2D(src, vec, -1, g);
		scaleAdd(src, -1, vec, vec);
		threshold(vec, vec, 0, 1, THRESH_TRUNC);
		vec *= -1;
		normalize(vec, vec, 0, 1, NORM_MINMAX);

		std::list<float> localMaxList;
		std::list<float>::iterator localMaxIter;

		DkIP::findLocalMaxima(vec, &localMaxList, gSigma, cutOffIvl);
		if (localMaxList.empty() || localMaxList.size() <= 1)	
			return;	// at least two local maxima present?

		localMaxIter = localMaxList.begin();
		float cIdx = *localMaxIter;
		++localMaxIter;	// skip the first element

		// compute the distance between peaks
		float lIdx = -1;
		while (localMaxIter != localMaxList.end()) {

			lIdx = cIdx;
			cIdx = *localMaxIter;

			maxDiffList->push_back(cIdx-lIdx);
			++localMaxIter;
		}
	}

	/**
	 * Computes the distance between successive local maxima.
	 *
	 * @param src 1xN Mat, a line histogram CV_32FC1.
	 * @param gSigma sigma for the 1D gaussian smoothing.
	 * @param cutOffIvl	threshold which rejects low local maxima.
	 * 
	 * @return a list containing the successive peak distances computed.
	 **/
	static std::list<float> lineHistPeakDist(Mat src, float gSigma, float cutOffIvl) {

		std::list<float> maxDiffList;
		DkIP::lineHistPeakDist(src, &maxDiffList, gSigma, cutOffIvl);

		return maxDiffList;
	}

	static DkVector calcRotationSize(double angleRad, DkVector srcSize) {

		DkVector nSl = srcSize;
		DkVector nSr = srcSize;

		// compute
		nSl.rotate(angleRad);
		nSl.abs();

		nSr.swap();
		nSr.rotate(angleRad);
		nSr.abs();
		nSr.swap();

		return nSl.getMaxVec(nSr);
	}
	
	/** 
	 * Rotates an image according to the angle obtained.
	 * The new image bounds are minimized with respect to
	 * the angle obtained.
	 * @param src an image which will be rotated
	 * @param angleRad the rotation angle in radians
	 * @return the rotated image
	 **/
	static Mat rotateImg(Mat src, double angleRad, int interpolation = INTER_CUBIC, Scalar borderValue = Scalar(0)) {
		
		// check inputs
		if (src.empty()) {
			std::string msg = "[DkIP::rotateImg] I cannot rotate an empty image";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (angleRad == 0.0f)
			return src.clone();
		//if (angleRad == CV_PI*0.5f)
		//	return src.clone().t();

		DkVector srcSize(src.size());
		DkVector nSl = calcRotationSize(angleRad, srcSize);
		DkVector center = nSl * 0.5f;

		Mat rotMat = getRotationMatrix2D(center.getCvPoint32f(), DK_RAD2DEG*angleRad, 1.0);

		// add a shift towards new center
		DkVector cDiff = center - (srcSize * 0.5f);
		cDiff.rotate(angleRad);

		double *transl = rotMat.ptr<double>();
		transl[2] += (double)cDiff.x;
		transl[5] += (double)cDiff.y;

		// img in wrapAffine must not be overwritten
		Mat rImg = Mat(nSl.getCvSize(), src.type());
		warpAffine(src, rImg, rotMat, rImg.size(), interpolation, BORDER_CONSTANT, borderValue);


		//if (minBoundary) {

		//	DkTimer tt = DkTimer();
		//	DkBox b = findBoundingBox(rImg);
		//	DkUtils::printDebug(DK_MODULE, "box found in: %s\n", tt.getTotal().c_str());

		//	
		//	
		//	b.clip(rImg.size());
		//	b.size -= 1;

		//	printf("new image border: %s\nc size: %s\n", b.toString().c_str(), DkVector(rImg.size()).toString().c_str());
		//	rImg = rImg(b.getCvRect()).clone();
		//}


		return rImg;
	}

	static DkBox findBoundingBox(Mat &src) {

		DkBox box;

		if (src.depth() == CV_32F)
			box = findBoundingBoxIntern<float>(src);
		else if (src.depth() == CV_8U)
			box = findBoundingBoxIntern<unsigned char>(src);
		else
			box = DkBox(DkVector(0,0), DkVector(src.size()));

		return box;
	}

	/** 
	 * Rotates an (rotated) image back
	 * The new image bounds are the previous bounds given by newSize.
	 * @param src an image which will be rotated
	 * @param angle the rotation angle in radians
	 * @param newSize the size of the rotated image as Size
	 * @return the rotated image
	 **/
	static Mat rotateInvImg(Mat src, double angle, Size newSize, int interpolation = INTER_CUBIC) {


		// check inputs
		if (src.empty()) {
			std::string msg = "[DkIP::rotateInvImg] I cannot rotate an empty image";
			throw DkMatException(msg, __LINE__, __FILE__);
		}


		DkVector oSz = DkVector(src.size());	// *0.5f?
		DkVector nSz = DkVector(newSize);

		DkVector center = nSz * 0.5f;

		Mat rotMat = getRotationMatrix2D(center.getCvPoint32f(), DK_RAD2DEG*angle, 1.0);

		// add a shift towards new center
		DkVector cDiff = center - (oSz * 0.5f);
		cDiff.rotate(angle);

		double *transl = rotMat.ptr<double>();
		transl[2] += (double)cDiff.x;
		transl[5] += (double)cDiff.y;

		// img in wrapAffine must not be overwritten
		Mat rImg = Mat(newSize, src.type());
		warpAffine(src, rImg, rotMat, rImg.size(), interpolation);

		return rImg;
	}

	/**
	 * Non-linear contrast enhancement.
	 * @param src a grayscale image CV_32FC1.
	 * @param mask a grayscale (binary) mask (CV_32FC1 or CV_8UFC1).
	 * @param slope the Sigmoids slope.
	 * @param mean the Sigmoids mean.
	 * @return grayscale image CV_32FC1
	 **/
	static Mat weightImageSigmoid(const Mat src, const Mat mask, float slope = 0.06f, float mean = -1.0f) {

		// check inputs
		if (src.type() != CV_32FC1) {
			std::string msg = "The image obtained is not of type CV_32FC1: " + DkUtils::getMatInfo(src);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (!mask.empty()) {
			if (src.rows != mask.rows || src.cols != mask.cols) {
				std::string msg = "Matrix dimension mismatch: \n  img  (" + 
					DkUtils::stringify(src.rows) + " x " + DkUtils::stringify(src.cols) + ")\n  mask (" +
					DkUtils::stringify(mask.rows) + " x " + DkUtils::stringify(mask.cols) + ")\n";
				throw DkMatException(msg, __LINE__, __FILE__);
			}

			if (mask.depth() != CV_32F && mask.depth() != CV_8U) {
				std::string msg = "The mask obtained is neither of type CV_32F nor CV_8U: " + DkUtils::getMatInfo(mask);
				throw DkMatException(msg, __LINE__, __FILE__);
			}
		}

		if (mean == -1.0f) {
			Mat hist = DkIP::computeHist(src, mask);				//weight gray values with sigmoid function according		
			mean = (float)DkIP::getThreshOtsu(hist);		//sigmoid slope, centered at l according text estimation
			mean = mean/255.0f;
			DkUtils::printDebug(DK_DEBUG_INFO, "mean: %.3f\n", mean);
		}

		const float *maskPtrF;
		const uchar *maskPtrU;
		Mat dst = Mat(src.size(), src.type());
		
		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			const float *srcPtr = src.ptr<float>(rIdx);
			float *dstPtr = dst.ptr<float>(rIdx);

			if (!mask.empty() && mask.depth() == CV_32F)
				maskPtrF = mask.ptr<float>(rIdx);
			else if (!mask.empty() && mask.depth() == CV_8U)
				maskPtrU = mask.ptr<uchar>(rIdx);

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				// skip non mask values
				if (!mask.empty() &&
					((mask.depth() == CV_32FC1	&& maskPtrF[cIdx] == 0.0f) ||
					(mask.depth() == CV_8U		&& maskPtrU[cIdx] == 0))) {
					
					dstPtr[cIdx] = 0.0f;
					continue;
				}

				// the sigmoid function
				float val = -(srcPtr[cIdx] - mean)/slope;
				dstPtr[cIdx] = 1.0f/(1.0f + expf(val));
				//dstPtr[cIdx] = srcPtr[cIdx];
			}
		}

		return dst;
	}

	/**
	 * Swaps the channels of an RGB image.
	 * This function is especially useful if an RGB
	 * image needs to be transformed to a BGR image
	 * and vice versa.
	 * @param img a 3 channel color image.
	 * @return the image where the R and B channels are swapped.
	 **/
	static Mat swapChannels(Mat img) {

		if (img.channels() != 3) {
			std::string msg = "A 3 channel image is required, the image is: " + DkUtils::getMatInfo(img) + "\n";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		std::vector<Mat> rgbCh;
		split(img, rgbCh);
		Mat tmp;
		tmp = rgbCh[0];
		rgbCh[0] = rgbCh[2];
		rgbCh[2] = tmp;
		merge(rgbCh, img); 

		return img;
	}

	static Mat normDims(const Mat &img, DkDim dim) {
		Mat m;
		return normDims(img, dim, m);
	}

	/**
	 * Normalizes the matrix along the dimension dim.
	 * This method is especially useful if one has features that need to be
	 * normalized along their feature dimensions.
	 * @param img a 32FC1 input image
	 * @param dim the dimension along which the matrix is normalized
	 * @param maxDim the maxima along the dimensions, if !maxDim.empty() the maxima are not calculated
	 * @return cv::Mat the normalized matrix
	 **/ 
	static Mat normDims(const Mat &img, DkDim dim, Mat& maxDim) {

		// just allow 32FC1 mats
		if (img.type() != CV_32FC1) {
			std::string msg = "The image obtained is not of type CV_32FC1: " + DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (maxDim.empty()) {
			int mCols = (dim == DK_S_DIM_X) ? img.cols : img.rows;
			
			maxDim = Mat(1, mCols, img.type());
			minMaxDim<float>(img, &maxDim);
		}

		Mat rImg = img.clone();
		const float* maxPtr = maxDim.ptr<float>();

		for (int cRow = 0; cRow < rImg.rows; cRow++) {

			float* rPtr = rImg.ptr<float>(cRow);

			//// skip the row if the maximum is one
			//if (dim == DK_S_DIM_Y && maxPtr[cRow] < FLT_EPSILON || fabs(maxPtr[cRow]-1.0f) < FLT_EPSILON)
			//	continue;

			for (int cCol = 0; cCol < rImg.cols; cCol++) {

				// skip col if max is one (speed up)
				if (dim == DK_S_DIM_X && maxPtr[cCol] < FLT_EPSILON || fabs(maxPtr[cCol]-1.0f) < FLT_EPSILON)
					continue;

				rPtr[cCol] = (dim == DK_S_DIM_X) ? rPtr[cCol]/maxPtr[cCol] : rPtr[cCol]/maxPtr[cRow];
			}
		}

		return rImg;
	}

	/**
	 * Returns the minimum and maximum of each column or row (depending on dim)
	 * @param img	a 32FC1 input mat
	 * @param dim	the dimension along which the maxima and minima should be computed
	 * @param maxDim a pointer to the mat which stores the maxima (if 0 the maxima will not be computed)
	 * @param minDim a pointer to the mat which stores the minima (if 0 the minima will not be computed)
	 **/ 
	template <typename num>
	static void minMaxDim(const Mat &img, Mat *maxDim, Mat *minDim = 0) {

		// just allow 32FC1 mats
		if (img.type() != CV_32FC1) {
			std::string msg = "The image obtained is not of type CV_32FC1: " + DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (img.empty() || !maxDim && !minDim) 
			return;

		if (maxDim && minDim && maxDim->cols != minDim->cols) {
			std::string msg = "[minMaxDim] min.cols != max.cols: " + DkUtils::stringify(minDim->cols) + " != " + DkUtils::stringify(maxDim->cols);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (maxDim && maxDim->rows != 1 || minDim && minDim->rows != 1) {
			std::string msg = "[minMaxDim] min and max must be a 1xN Matrix (minDim: " + 
				DkUtils::stringify(minDim->rows) + "x" + DkUtils::stringify(minDim->cols) + ", maxDim: " +
				DkUtils::stringify(maxDim->rows) + "x" + DkUtils::stringify(maxDim->cols) + ")";
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (maxDim && maxDim->cols != img.cols && maxDim->cols != img.rows ||
			minDim && minDim->cols != img.cols && minDim->cols != img.rows) {
				std::string msg = "[minMaxDim] min and max Matrix must either equal the image rows or the image cols";
				throw DkMatException(msg, __LINE__, __FILE__);
		}


		DkDim dim = (maxDim && maxDim->cols == img.cols || minDim && minDim->cols == img.cols) ? DK_S_DIM_X : DK_S_DIM_Y;

		// initialize min and max arrays
		if (maxDim != 0)
			*maxDim = std::numeric_limits<num>::min();
		if (minDim != 0)
			*minDim = std::numeric_limits<num>::max();

		num* minPtr = (minDim) ? minDim->ptr<num>() : 0;
		num* maxPtr = (maxDim) ? maxDim->ptr<num>() : 0;

		for (int cRow = 0; cRow < img.rows; cRow++) {

			const float* ptr = img.ptr<float>(cRow);

			for (int cCol = 0; cCol < img.cols; cCol++) {

				int idx = (dim == DK_S_DIM_X) ? cCol : cRow;

				if (minPtr && ptr[cCol] < minPtr[idx])
					minPtr[idx] = ptr[cCol];
				if (maxPtr && ptr[cCol] > maxPtr[idx])
					maxPtr[idx] = ptr[cCol];
			}
		}
	}

	/**
	 * Normalizes a [-1 1] image.
	 * This normalization does not change around 0.
	 * Thus, the negative part is normalized with no respect to the positive part.
	 * @param img the image which needs to be normalized [-1 1], CV_32FC1
	 **/
	static void normDiffImage(Mat img) {

		if (img.type() != CV_32FC1) {
			std::string msg = "The image obtained is not of type CV_32FC1: " + DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double minV, maxV;
		minMaxLoc(img, &minV, &maxV);
		float minPx = (float)abs(minV);
		float maxPx = (float)maxV;

		// nothing to do in here...
		if (maxV == minV)
			return;

		float* srcPtr = img.ptr<float>();

		for (int rIdx = 0; rIdx < img.rows; rIdx++) {

			for (int cIdx = 0; cIdx < img.cols; cIdx++, srcPtr++) {

				if (maxPx != 0.0f && *srcPtr > 0)
					*srcPtr /= maxPx;
				else if (minPx != 0.0f && *srcPtr < 0)
					*srcPtr /= minPx;
			}
		}
	}


	/**
	 * Inverts an Image (must be CV_32FC1 [0 1], or CV_8UC1)
	 * @param img to be converted as CV_32FC1 [0 1] or CV_8UC1.
	 * @param mask the optional mask
	 * @return the inverted image (CV_32FC1 [0 1] or CV_8UC1).
	 **/
	static void invertImg(Mat img, Mat mask=Mat()) {

		if (img.depth() == CV_32F) {
			
			int rows = img.rows;
			int cols = img.cols;
			for(int i = 0; i < rows; i++) {
				float *ptrImg = img.ptr<float>(i);
				for(int j = 0; j < cols; j++) {
					ptrImg[j] = ptrImg[j] * -1.0f + 1.0f;
				}
			}
			DkIP::mulMask(img, mask);

		} else if (img.depth() == CV_8U) {

			bitwise_not(img, img);
			DkIP::mulMask(img, mask);

		} else {
			std::string msg = "[DkIP::invertImg] the input image's depth must be 32F or 8U, it is:\n";
			msg += DkUtils::getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

	}

	static Mat createNoise(const Mat mask) {

		// check inputs
		if (mask.type() != CV_8UC1) {
			std::string msg = "The image obtained is not of type CV_8UC1: " + DkUtils::getMatInfo(mask);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		Mat dst = Mat(mask.size(), CV_32FC1);
		dst.setTo(0);
		srand((unsigned int)DkTimer::getTime());

		for (int row = 0; row < mask.rows; row++) {

			const unsigned char* srcPtr = mask.ptr<unsigned char>(row);
			float* dstPtr = dst.ptr<float>(row);

			for (int col = 0; col < mask.cols; col++) {

				if (srcPtr[col] > 0)
					dstPtr[col] = (float)rand()/RAND_MAX;

			}
		}

		return dst;
	}

	/**
	 * Calculates the projecion profile of an image
	 * @param img a 32F grayvalue image
	 * @param kernelSize the kernelsize.
	 * @param dim ...
	 * @return the projection profile
	 **/
	static Mat localProjectionProfile(const Mat img, int kernelSize, int dim = DK_S_DIM_X) {

		if (img.type() != CV_32FC1) {
			std::string msg = "localProjectionProfile: img  must be 32FC1, it is: " + DkUtils::getMatInfo(img) + "\n";
			throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
		}

		Mat imgCpy = img;


		if (dim == DK_S_DIM_Y) {
			imgCpy = imgCpy.t();
		}

		Mat dst = Mat(imgCpy.size(), imgCpy.type());

		int halfKS = cvFloor((float)kernelSize*0.5);
		float kernel = 0.0f;
		const float*ptrSrc = imgCpy.ptr<float>(0);

		for (int row=0; row < dst.rows; row++) {
			
			float* ptrDst = dst.ptr<float>(row);

			kernel = 0.0f;
			int lBound = row*dst.cols;
			int rBound = (row+1)*dst.cols-1;
			int kl = lBound;													// left kernel index
			int kr = (dst.cols < halfKS) ? rBound : row*dst.cols+halfKS;		// right kernel index
			// init kernel
			for (int iIdx = kl; iIdx < kr; iIdx++)
				kernel += ptrSrc[iIdx];

			for (int col=0; col < dst.cols; col++) {

				if (col > halfKS) {
					kernel -= ptrSrc[kl];
					kl++;
				}
				if (col+halfKS < dst.cols) {
					kernel += ptrSrc[kr];
					kr++;
				}
				*ptrDst = kernel;
				ptrDst++;

			}
		}

		if (dim == DK_S_DIM_Y) {
			dst = dst.t();
		}

		return dst;
	}

	static bool isBinary(const Mat src) {

		// do nothing if the mask is empty
		if (!src.empty()) {

			if (src.depth() == CV_32F)
				return isBinaryIntern<float>(src);
			else if (src.depth() == CV_8U)
				return isBinaryIntern<unsigned char>(src);
			else {
				std::string msg = "The source image must be [CV_8U or CV_32F], but it is (img: " +
					DkUtils::getMatInfo(src) + ")";
				DkMatException(msg, __LINE__, __FILE__);
			}
		}

		return false;
		//else {
		//	DkUtils::printDebug(DK_WARNING, "empty mask...\n");
		//}
	}


	static void mulMask(Mat src, const Mat mask) {

		// do nothing if the mask is empty
		if (!mask.empty()) {

			if (src.depth() == CV_32F && mask.depth() == CV_32F)
				mulMaskIntern<float, float>(src, mask);
			else if (src.depth() == CV_32F && mask.depth() == CV_8U)
				mulMaskIntern<float, unsigned char>(src, mask);
			else if (src.depth() == CV_8U && mask.depth() == CV_32F)
				mulMaskIntern<unsigned char, float>(src, mask);
			else if (src.depth() == CV_8U && mask.depth() == CV_8U)
				mulMaskIntern<unsigned char, unsigned char>(src, mask);
			else if (src.depth() == CV_32S && mask.depth() == CV_8U)
				mulMaskIntern<int, unsigned char>(src, mask);
			else {
				std::string msg = "The source image and the mask must be [CV_8U or CV_32F], but they are (img: " +
					DkUtils::getMatInfo(src) + " mask: " + DkUtils::getMatInfo(mask) + ")";
				DkMatException(msg, __LINE__, __FILE__);
			}
		}
		//else {
		//	DkUtils::printDebug(DK_WARNING, "empty mask...\n");
		//}
	}

	/**
	 * Sets the border of an image to a defined value (1 pixel width)
	 * @param src a 32F or 8U image
	 * @param val the border value (default: 0)
	 **/
	static void setBorderConst(Mat src, float val=0.0f) {

		// do nothing if the mask is empty
		if (!src.empty()) {
			if (src.depth() == CV_32F)
				setBorderConstIntern<float>(src, val);
			else if (src.depth() == CV_8U)
				setBorderConstIntern<unsigned char>(src, (unsigned char)val);
			else {
				std::string msg = "The source image and the mask must be [CV_8U or CV_32F], but they are (img: " +
					DkUtils::getMatInfo(src) +  ")";
				DkMatException(msg, __LINE__, __FILE__);
			}
		}
	}

	static Mat convexHull(const Mat& bw, cv::Rect& bbox) {

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		findContours(bw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		// Find the convex hull object for each contour
		vector<vector<Point> > hull(contours.size());
		for(int i = 0; i < contours.size(); i++) { 
			cv::convexHull(Mat(contours[i]), hull[i], false); 
		}

		// Draw contours + hull results
		Mat hullImg(bw.size(), CV_8UC1, Scalar(0));

		for(int i = 0; i< contours.size(); i++ ) {
			Scalar color = Scalar(255);
			//drawContours(hullImg, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			//drawContours(hullImg, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point(), CV_);
			drawContours(hullImg, hull, i, color, CV_FILLED);
		}

		bbox = DkBox::contour2BBox(contours).getCvRect();

		return hullImg;
	}

	/**
	 * Performs a bitwise and of each pixel with the byte specified (default: 128).
	 * @param src a CV_8UC1 image
	 * @param mask a CV_8UC1 mask or an empty mat
	 * @param byte a byte
	 **/ 
	static void bitwiseAnd(Mat src, const Mat mask=Mat(), unsigned char byte=0x0F) {

		if (src.empty()) {
			std::string msg = "The source image is empty";
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (src.type() != CV_8UC1) {
			std::string msg = "The image must be CV_8UC1, but it is: " + DkUtils::getMatInfo(src);
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (mask.type() != CV_8UC1) {
			std::string msg = "The image must be CV_8UC1, but it is: " + DkUtils::getMatInfo(mask);
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (!mask.empty() && src.size() != mask.size()) {
			std::string msg = "the source and the mask have not the same size.";
			DkMatException(msg, __LINE__, __FILE__);
		}

		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			unsigned char* srcPtr = src.ptr<unsigned char>(rIdx);
			const unsigned char* maskPtr = (!mask.empty()) ? mask.ptr<unsigned char>(rIdx) : 0; 

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				// if mask is empty or mask is non-zero
				if (maskPtr == 0 || maskPtr[cIdx] != 0)
					srcPtr[cIdx] = srcPtr[cIdx] & byte;
			}
		}

	}


	/**
	 * Performs a bitwise or of each pixel with the byte specified (default: 128).
	 * @param src a CV_8UC1 image
	 * @param mask a CV_8UC1 mask or an empty mat
	 * @param byte a byte
	 **/ 
	static void bitwiseOr(Mat src, const Mat mask=Mat(), unsigned char byte=0x0F) {

		if (src.empty()) {
			std::string msg = "The source image is empty";
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (src.type() != CV_8UC1) {
			std::string msg = "The image must be CV_8UC1, but it is: " + DkUtils::getMatInfo(src);
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (mask.type() != CV_8UC1) {
			std::string msg = "The image must be CV_8UC1, but it is: " + DkUtils::getMatInfo(mask);
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (!mask.empty() && src.size() != mask.size()) {
			std::string msg = "the source and the mask have not the same size.";
			DkMatException(msg, __LINE__, __FILE__);
		}

		// for all pixels
		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			unsigned char* srcPtr = src.ptr<unsigned char>(rIdx);
			const unsigned char* maskPtr = (!mask.empty()) ? mask.ptr<unsigned char>(rIdx) : 0; 

			for (int cIdx = 0; cIdx < src.cols; cIdx++) {

				// if mask is empty or mask is non-zero
				if (maskPtr == 0 || maskPtr[cIdx] != 0)
					srcPtr[cIdx] = srcPtr[cIdx] | byte;
			}
		}

	}



	/**
	 * Returns the image value at position (x,y).
	 * @param src a CV_32FC1 or CV_8UC1 image
	 * @param x the image column (x-coordinate)
	 * @param y the image row (y-coordinate)
	 * @return float the image value at the specified position
	 **/ 
	static float getPixelInfo(Mat src, int x, int y) {
		float pixelVal = -1.0f;
		if (src.empty()) {
			std::string msg = "The source image is empty";
			DkMatException(msg, __LINE__, __FILE__);
		}
		if ((y > src.rows-1) || (x > src.cols-1)) {
			std::string msg = "rows < y or cols < x";
			DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.depth() == CV_32F)
			pixelVal = getPixelInfoIntern<float>(src, x, y);
		else if (src.depth() == CV_8U)
			pixelVal = (float)getPixelInfoIntern<unsigned char>(src, x, y);
		else {
			std::string msg = "The source image and the mask must be [CV_8U or CV_32F], but they are (img: " +
				DkUtils::getMatInfo(src) +  ")";
			DkMatException(msg, __LINE__, __FILE__);
		}

		return pixelVal;
	}

	/**
	 * Prints the image value at position (x,y).
	 * @param src a CV_32FC1 or CV_8UC1 image
	 * @param x the image column (x-coordinate)
	 * @param y the image row (y-coordinate)
	 * @param varname the image name
	 **/ 
	static void getPixelInfo(Mat src, int x, int y, std::string varname) {
		float pixelVal = -1.0f;
		if (src.empty()) {
			std::string msg = "The source image is empty";
			DkMatException(msg, __LINE__, __FILE__);
		}
		if ((y > src.rows-1) || (x > src.cols-1)) {
			std::string msg = "rows < y or cols < x";
			DkMatException(msg, __LINE__, __FILE__);
		}

		if (src.depth() == CV_32F)
			pixelVal = getPixelInfoIntern<float>(src, x, y);
		else if (src.depth() == CV_8U)
			pixelVal = (float)(getPixelInfoIntern<unsigned char>(src, x, y));
		else {
			std::string msg = "The source image and the mask must be [CV_8U or CV_32F], but they are (img: " +
				DkUtils::getMatInfo(src) +  ")";
			DkMatException(msg, __LINE__, __FILE__);
		}

		printf("%s @ (%i,%i) = %f\n", varname.c_str(), x, y, pixelVal);
	}

	static Mat mulMaskClone(const Mat src, const Mat mask) {

		Mat dst = src.clone();

		if (!mask.empty()) {
			if (src.depth() == CV_32F && mask.depth() == CV_32F)
				mulMaskIntern<float, float>(dst, mask);
			else if (src.depth() == CV_32F && mask.depth() == CV_8U)
				mulMaskIntern<float, unsigned char>(dst, mask);
			else if (src.depth() == CV_8U && mask.depth() == CV_32F)
				mulMaskIntern<unsigned char, float>(dst, mask);
			else if (src.depth() == CV_8U && mask.depth() == CV_8U)
				mulMaskIntern<unsigned char, unsigned char>(dst, mask);
			else {
				std::string msg = "The source image and the mask must be [CV_8U or CV_32F], but they are (img: " +
					DkUtils::getMatInfo(src) + " mask: " + DkUtils::getMatInfo(mask) + ")";
				DkMatException(msg, __LINE__, __FILE__);
			}
		}

		return dst;
	}

	static float corrHistograms(Mat histTemplate, Mat histTest, int method, Point &shift, float thresh) {

		if (histTemplate.depth() != CV_32F) {
			std::string msg = "A CV_8U image is required, but it is: " + DkUtils::getMatInfo(histTemplate);
			throw DkMatException(msg, __LINE__, __FILE__);
		}
		if (histTest.depth() != CV_32F) {
			std::string msg = "A CV_8U image is required, but it is: " + DkUtils::getMatInfo(histTest);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		if (histTemplate.cols!=1 || histTest.cols!=1) {
			std::string msg = "columns size must be 1, but it is:  " + DkUtils::getMatInfo(histTemplate) + DkUtils::getMatInfo(histTest);
			throw DkMatException(msg, __LINE__, __FILE__);
		}

		double minV, maxV;
		Point minP, maxP;

		//histTemplate is always smaller then the histogram to test (pages can be copied on a larger page but not vice versa
		float sizeTemplate = (float)histTemplate.rows;
		float sizeTest = (float)histTest.rows;

		if (sizeTest*2 < sizeTemplate) { 
			shift = minP;
			if (method>=CV_TM_CCORR)
				return 0.0f;
			else
				return 1.0f;
		}
		if (sizeTest < sizeTemplate*thresh) {
			shift = minP;
			if (method>=CV_TM_CCORR)
				return 0.0f;
			else
				return 1.0f;
		}

		Mat tmpCorr = Mat((int)sizeTest*2, 1, CV_32FC1);
		tmpCorr.setTo(0);
		//"enlarge" the bigger featureVec - equivalen to zero padding of the borders of the template
		Mat tmpRange = 	tmpCorr.rowRange((int)(sizeTest/2.0f-1.0f), (int)(sizeTest/2.0f-1.0f + sizeTest));// = hist1.rows < hist2.rows ? hist2 : hist1;
		histTest.copyTo(tmpRange);

		//float scaleF = sizeTest/sizeTemplate;
		//tmpRange *= scaleF;

		Mat result;
		matchTemplate(tmpRange, histTemplate, result, method);

		minMaxLoc(result, &minV, &maxV, &minP, &maxP);

		//correct the index according original histogram (without zero padding)
		minP -= Point(0, (int)sizeTest/2-1);
		maxP -= Point(0, (int)sizeTest/2-1);

		if (minP.y > histTest.rows-1) minP.y = histTest.rows-1;
		if (maxP.y > histTest.rows-1) maxP.y = histTest.rows-1;

		//printf("correlation idx:  %i\n", maxP);

		if (method==CV_TM_CCORR || method==CV_TM_CCORR_NORMED || method==CV_TM_CCOEFF || method==CV_TM_CCOEFF_NORMED) {
			shift = maxP;
			return (float)maxV;
		} else {
			shift = minP;
			return (float)minV;
		}
	}

	static Mat whiteRunMask(const Mat& src, double angle = 0.0) {

		if (src.empty()) {
			std::string msg = "The source image is empty";
			DkMatException(msg, __LINE__, __FILE__);
		}
		if (src.type() != CV_32FC1) {
			std::string msg = "The image must be CV_32FC1, but it is: " + DkUtils::getMatInfo(src);
			DkMatException(msg, __LINE__, __FILE__);
		}


		Mat rImg = DkIP::rotateImg(src, angle);

		Mat rowMul	= Mat(1, rImg.rows, rImg.type());
		Mat hist	= Mat(1, rImg.cols, rImg.type());

		// sum all columns
		rowMul.setTo(1);
		gemm(rowMul, rImg, 1, Mat(), 0, hist);

		// smooth col hist
		hist = DkIP::convolveSymmetric(hist, DkIP::get1DGauss(15));

		// compute weight image
		Mat weightImg = Mat(rImg.size(), rImg.type());
		gemm(rowMul.t(), hist, 1, Mat(), 0, weightImg);

		DkIP::invertImg(weightImg);
		//DkIP::mulMask(weightImg, imgs.getMask());
		normalize(weightImg, weightImg, 0.0, 1.0, NORM_MINMAX);


		weightImg = DkIP::rotateInvImg(weightImg, -angle, src.size());

		return weightImg;
	}

//	static void drawRectangle(Mat& img, DkRectCorners &rect, Scalar col = Scalar(255), float opacity = 1.0f) { original
	static void drawRectangle(Mat& img, DkRectCorners rect, Scalar col = Scalar(255), float opacity = 1.0f) {

		if (opacity <= 0.0f)
			return;

		vector<Point> rPts = rect.getPoints();
		const Point* rPtsA = &rPts[0];
		int n = (int)rPts.size();
		
		Mat imgCpy;
		
		if (opacity < 1.0f)
			imgCpy = img.clone();
			
		
		fillPoly(img, &rPtsA, &n, 1, col);

		if (opacity < 1.0f)
			img = img*opacity + imgCpy*(1.0f-opacity);
	}

	//static void drawRectangleContour(Mat& img, const DkRectCorners &rect, Scalar col = Scalar(255,255,255), int lineType = 8) {

	//	line(img, rect.a.getCvPoint32f(), rect.b.getCvPoint32f(), Scalar(col), 2, lineType);
	//	line(img, rect.b.getCvPoint32f(), rect.c.getCvPoint32f(), Scalar(col), 2, lineType);
	//	line(img, rect.c.getCvPoint32f(), rect.d.getCvPoint32f(), Scalar(col), 2, lineType);
	//	line(img, rect.d.getCvPoint32f(), rect.a.getCvPoint32f(), Scalar(col), 2, lineType);
	//}

	static void drawRectangleContour(Mat& img, const DkRectCorners &rect, Scalar col = Scalar(255,255,255), int lineType = 8, int linewidth = 2) {

		line(img, rect.a.getCvPoint32f(), rect.b.getCvPoint32f(), Scalar(col), linewidth, lineType);
		line(img, rect.b.getCvPoint32f(), rect.c.getCvPoint32f(), Scalar(col), linewidth, lineType);
		line(img, rect.c.getCvPoint32f(), rect.d.getCvPoint32f(), Scalar(col), linewidth, lineType);
		line(img, rect.d.getCvPoint32f(), rect.a.getCvPoint32f(), Scalar(col), linewidth, lineType);
	}

	static void drawDots(Mat& img, std::vector<DkVector> &dots, Scalar col=Scalar(0), int radius = 3) {

		// now find the k nearest
		for (unsigned int idx = 0; idx < dots.size(); idx++) {

			if (radius > 0)
				circle(img, dots[idx].getCvPoint32f(), radius*2, Scalar(0, 0, 0), -1);
			circle(img, dots[idx].getCvPoint32f(), radius, col, -1);
		}
	}


	//input angle [rad]
	//angleDiff in [deg]
	static std::vector<DkLine> filterLineAngle(std::vector<DkLine>& inputLines, double angle, float angleDiff=5.0f) {

		std::vector<DkLine> tmp = inputLines;
		std::vector<DkLine>::iterator lineIter = tmp.begin();

		//dout << class DK_CORE_APIName << " filterLineAngle" << dkendl;

		if (angle == 361.0f) {
			//DkUtils::printDebug(DK_WARNING, "[DkLines] no page angle set\n");
			return tmp;
		}

		while (lineIter != tmp.end()) {

			if (angle != 361.0f) {

				float a = DkMath::normAngleRad((float)angle, 0.0f, (float)CV_PI);
				float angleNewLine = DkMath::normAngleRad((float)lineIter->getAngle(),0.0f,(float)CV_PI);

				float diffangle = (float)min(fabs(DkMath::normAngleRad(a, 0, (float)CV_PI)-DkMath::normAngleRad(angleNewLine, 0, (float)CV_PI))
					, (float)CV_PI-fabs(DkMath::normAngleRad(a, 0, (float)CV_PI)-DkMath::normAngleRad(angleNewLine, 0, (float)CV_PI)));

				if (diffangle > angleDiff/180.0f*(float)CV_PI) {
					lineIter = tmp.erase(lineIter);
					continue;
				}
			}

			lineIter++;
		}

		return tmp;
	}

	static void filterBorderLines(std::vector<DkLine> &lineVectorH, std::vector<DkLine> &lineVectorV, Size imgS, int borderThres) {

		std::vector<DkLine>::iterator lineIter = lineVectorH.begin();

		//test horizontal lines if they are located within the borderThreshold
		while (lineIter != lineVectorH.end()) {
			if (((lineIter->getYS() - borderThres) <= 0 && (lineIter->getYE() - borderThres) <= 0) ||
			   ((lineIter->getYS() + borderThres) >= imgS.height && (lineIter->getYE() + borderThres) >= imgS.height)) {
				lineIter = lineVectorH.erase(lineIter);
				continue;
			}
			lineIter++;
		}

		//test vertical lines if they are located within the borderThreshold
		lineIter = lineVectorV.begin();
		while (lineIter != lineVectorV.end()) {
			if (((lineIter->getXS() - borderThres) <= 0 && (lineIter->getXE() - borderThres) <= 0) ||
				((lineIter->getXS() + borderThres) >= imgS.width && (lineIter->getXE() + borderThres) >= imgS.width)) {
					lineIter = lineVectorV.erase(lineIter);
					continue;
			}
			lineIter++;
		}

	}

	static void drawLines(Mat& img, const std::vector<DkLine> &lineVector, Scalar col=Scalar(0,0,255), int thickness=4, int circles=1) {
		
		if (img.empty())
			return;

		for (int i=0; i<(int)lineVector.size(); i++) {
			cv::line(img, lineVector[i].getStartPoint(), lineVector[i].getEndPoint(),  col, thickness);//(int)lineVector[i].getThickness());
			
			//DkUtils::getMatInfo(drawImg, "drawImg");
			
			if (circles) {
				circle(img, lineVector[i].getStartPoint(), 6, Scalar(0, 0, 0), -1);
				circle(img, lineVector[i].getStartPoint(), 3, Scalar(255, 255, 255), -1);

				circle(img, lineVector[i].getEndPoint(), 6, Scalar(255, 255, 255), -1);
				circle(img, lineVector[i].getEndPoint(), 3, Scalar(0, 0, 0), -1);
			}
		}
	}

	static Mat drawLinesClone(const Mat img, const std::vector<DkLine> &lineVector, Scalar col=Scalar(0,0,255), int thickness=4, int circles=1) {

		Mat linImg = img.clone();
		drawLines(linImg, lineVector, col, thickness, circles);

		return linImg;
	}

	static Mat drawLinesClone(Mat img, const std::vector<DkLineExt> &lineVector, Scalar col=Scalar(0,0,255), int thickness=4, int circles=1) {

		return drawLinesClone(img, std::vector<DkLine>(lineVector.begin(), lineVector.end()), col, thickness, circles);

	}

	static Mat drawLinesClone(Mat img, const std::vector<DkLineDotted> &lineVector, Scalar col=Scalar(0,0,255), int thickness=4, int circles=1) {

		return drawLinesClone(img, std::vector<DkLine>(lineVector.begin(), lineVector.end()), col, thickness, circles);

	}

	static Mat drawLinesClone(Mat img, std::list<DkLine> lineVector, Scalar col=Scalar(0,0,255), int thickness=4, int circles=1) {

		if (img.empty())
			return Mat();

		Mat drawImg = img.clone();

		std::list<DkLine>::iterator it;

		for (it = lineVector.begin(); it != lineVector.end(); it++) {
			cv::line(drawImg, it->getStartPoint(),it->getEndPoint(),  col, thickness, 8, 0);//(int)lineVector[i].getThickness());

			if(circles)
			{
				circle(drawImg, it->getStartPoint(), 6, Scalar(0, 0, 0), -1);
				circle(drawImg, it->getStartPoint(), 3, Scalar(255, 255, 255), -1);

				circle(drawImg, it->getEndPoint(), 6, Scalar(255, 255, 255), -1);
				circle(drawImg, it->getEndPoint(), 3, Scalar(0, 0, 0), -1);
			}

		}

		return drawImg;
	}


	static void drawSingleContour(Mat &img, vector<Point> *contour, Scalar col = Scalar(255)) {
		
		// is anything to do in here?
		if (!contour)
			return;

		if (img.empty()) {
			throw DkMatException("drawSingleContour: image should not be empty\n", __LINE__, __FILE__);
		}

		if (img.type() != CV_8UC1) {
			std::string msg = "drawSingleContour: img  must be 8UC1, it is: " + DkUtils::getMatInfo(img) + "\n";
			throw DkIllegalArgumentException(msg, __LINE__, __FILE__);
		}

		vector<vector<Point> > contours;
		contours.push_back(*contour);
				
		drawContours(img, contours, 0, col, CV_FILLED, 8);
	}

	static void writeTextToImage(Mat img, std::string text, DkVector coords = DkVector(), int txtSize = 1) {

		if (img.empty())
			return;

		int baseline;
		Size s = getTextSize(text, CV_FONT_HERSHEY_PLAIN, txtSize, 11, &baseline);

		CvPoint tC = Point((int)(coords.x-(float)s.width*0.5f), (int)(coords.y+s.height*0.5f));
		putText(img, text, tC, CV_FONT_HERSHEY_PLAIN, txtSize, Scalar(255, 255, 255), 11, CV_AA);
		putText(img, text, tC, CV_FONT_HERSHEY_PLAIN, txtSize, Scalar(0, 0, 0), 2, CV_AA);
	}

	/**
	 * calculates the cosine distance of two histograms
	 * @param hist1 the first histogram
	 * @param hist2 the second histogram
	 * @return double; the cosine distance of the two histograms, 0 is equal 2 is orthogonal, DBL_MAX is dimension mismatch
	 **/ 
	static double cosineDistance(Mat hist1, Mat hist2) {
		if (hist1.rows != 1 || hist2.rows != 1 || hist1.cols != hist2.cols)
			return DBL_MAX;
		return 1-hist1.dot(hist2)/((norm(hist1)*norm(hist2)) + DBL_EPSILON); // 1-dist ... 0 is equal 2 is orthogonal
	}

	static void appendMatToBuffer(char** stream, size_t& length, const Mat& img) {

		size_t matLength = img.rows*img.cols*img.elemSize();
		size_t newBufferLength = length + matLength + 4*sizeof(int);

		char* newStream = new char[newBufferLength];

		if (*stream) {
			// copy old stream & clean it
			memcpy(newStream, *stream, length);
			delete *stream;
		}

		*stream = newStream;		// update output

		int iPos = 0;
		int* newIntStream = (int*)(newStream+length);

		newIntStream[iPos] = img.rows; iPos++;
		newIntStream[iPos] = img.cols; iPos++;
		newIntStream[iPos] = img.depth(); iPos++;
		newIntStream[iPos] = img.channels(); iPos++;

		newStream = (char*)(newIntStream+iPos);

		memcpy(newStream, img.data, matLength);
		length = newBufferLength;	// update output
	}

	static const char* getMatFromBuffer(const char* buffer, Mat& img) {

		if (!buffer) {
			wout << "[getMatFromBuffer] cannot read from an empty buffer!" << dkendl;
			return buffer;
		}

		int iPos = 0;
		const int* iBuffer = (const int*)buffer;

		int rows, cols, depth, channels;
		rows = iBuffer[iPos]; iPos++;
		cols = iBuffer[iPos]; iPos++;
		depth = iBuffer[iPos]; iPos++;
		channels = iBuffer[iPos]; iPos++;

		//mout << "initializing image: " << rows << " x " << cols << dkendl;

		const char* matBuf = (const char*)(iBuffer+iPos);

		if (!rows || !cols)	// empty image?
			return matBuf;

		// the const cast seems to be evil - but we clone the image in the next line to guarantee no operations on the buffer
		img = Mat(rows, cols, CV_MAT_DEPTH(depth) | CV_MAT_CN(channels), const_cast<char *>(matBuf));	
		img = img.clone();	// copy the buffer

		buffer = matBuf + img.rows*img.cols*img.elemSize();

		return buffer;
	}



#ifdef DK_SAVE_DEBUG
	static bool imwrite(std::string path, Mat img, bool norm = false) {

		
		Mat img1C = (img.channels() > 1) ? img.reshape(1) : img;
		
		double minV, maxV;
		minMaxLoc(img1C, &minV, &maxV);

		Mat imgW = img.clone();

		if (norm)
			normalize(img, imgW, 0, 255.0f, NORM_MINMAX);

		else if (img.depth() != CV_8U && maxV <= 1.0) {
			imgW = img.clone();
			imgW *= 255.0f;
		}

		return cv::imwrite(path, imgW);
	}
#else
#pragma warning(disable: 4100)
	static void imwrite(std::string path, Mat img, bool norm = false) { return false; };
#pragma warning(default: 4100)
#endif

};
