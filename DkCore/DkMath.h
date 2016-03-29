/*******************************************************************************************************
 DkMath.h
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

#include <list>

#include "DkUtils.h"

#ifdef max
	#undef max
#endif
#ifdef min
	#undef min
#endif

using namespace cv;

#define DK_DEG2RAD	0.017453292519943
#define DK_RAD2DEG 	57.295779513082323
#define DK_EULER	2.718281828459045

class DK_CORE_API DkVector;

/** 
 * Provides useful mathematical functions.
 **/
class DK_CORE_API DkMath {

public:
	
	/** 
	 * Divides the integer by 2.
	 * @param val the integer value.
	 * @return the half integer (floor(val)).
	 **/
	static int halfInt(int val) {
		return (val >> 1);
	}

	static unsigned char cropToUChar(float x) {

		return (x < 0) ? 0 : (x > 255) ? 255 : (unsigned char)x;
	}


	/**
	 * Computes a fast square root.
	 * @param val the value for which the root will be computed.
	 * @return the approximated square root of the value val.
	 **/
	static float fastSqrt(const float val) {

		long sqrtVal = *(long *) &val;

		//sqrtVal -= 1L<<23;	// Remove IEEE bias from exponent (-2^23)
		sqrtVal -= 127L<<23;
		// sqrtVal is now an approximation to logbase2(val)
		sqrtVal = sqrtVal>>1; // divide by 2
		//sqrtVal += 1L<<23;	// restore the IEEE bias from the exponent (+2^23)
		sqrtVal += 127L<<23;

		return *(float *) &sqrtVal;
	}

	/**
	 * Computes a fast inverse square root.
	 * @param x the value to be computed.
	 * @return the inverse square root of x.
	 **/
	static float invSqrt (float x) {
		float xhalf = 0.5f*x;
		int i = *(int*)&x;
		i = 0x5f3759df - (i>>1);
		x = *(float*)&i;
		x = x*(1.5f - xhalf*x*x);
		return x;
	}

	/**
	 * Computes the normalized angle in radians.
	 * The normalized angle is in this case defined as the
	 * corresponding angle within [0 pi].
	 * @param angle an angle in radians.
	 * @return the normalized angle in radians within [0 pi].
	 **/
	static double normAngleRad(double angle) {

		// this could be a bottleneck
		if (abs(angle) > 1000)
			return angle;

		while (angle < 0)
			angle += 2*CV_PI;
		while (angle >= 2*CV_PI)
			angle -= 2*CV_PI;

		return angle;
	}

	/**
	 * Computes the normalized angle within startIvl and endIvl.
	 * @param angle the angle in radians.
	 * @param startIvl the interval's lower bound.
	 * @param endIvl the interval's upper bound.
	 * @return the angle within [startIvl endIvl)
	 **/
	static double normAngleRad(double angle, double startIvl, double endIvl) {

		// this could be a bottleneck
		if (abs(angle) > 1000)
			return angle;

		while(angle <= startIvl)
			angle += endIvl-startIvl;
		while(angle > endIvl)
			angle -= endIvl-startIvl;

		return angle;
	}

	/**
	 * Computes the normalized angle within startIvl and endIvl.
	 * @param angle the angle in radians.
	 * @param startIvl the interval's lower bound.
	 * @param endIvl the interval's upper bound.
	 * @return the angle within [startIvl endIvl)
	 **/
	static float normAngleRad(float angle, float startIvl, float endIvl) {

		// this could be a bottleneck
		if (abs(angle) > 1000)
			return angle;

		while(angle <= startIvl)
			angle += endIvl-startIvl;
		while(angle > endIvl)
			angle -= endIvl-startIvl;

		return angle;
	}

	/**
	 * Computes the normalized angle in radians.
	 * The normalized angle is in this case defined as the
	 * corresponding angle within [0 pi].
	 * @param angle an angle in radians.
	 * @return the normalized angle in radians within [0 pi].
	 **/
	static float normAngleRad(float angle) {

		// this could be a bottleneck
		if (abs(angle) > 1000)
			return angle;

		while (angle < 0)
			angle += 2*(float)CV_PI;
		while (angle >= 2.0*CV_PI)
			angle -= 2*(float)CV_PI;

		return angle;
	}

	static double diffAngle(const double angle1, const double angle2, double maxAngle = 2*CV_PI) {

		double nAngle1 = normAngleRad(angle1, 0, maxAngle);
		double nAngle2 = normAngleRad(angle2, 0, maxAngle);

		double angle = nAngle1 - nAngle2;

		return (angle > maxAngle*0.5) ? maxAngle - angle : angle;
	}

	static double distAngle(const double angle1, const double angle2, double maxAngle = 2*CV_PI) {

		double nAngle1 = normAngleRad(angle1, 0, maxAngle);
		double nAngle2 = normAngleRad(angle2, 0, maxAngle);

		double angle = abs(nAngle1 - nAngle2);

		return (angle > maxAngle*0.5) ? maxAngle - angle : angle;
	}

	/**
	 * Check if a number is a power of two.
	 * @param ps a positive integer
	 * @return true if ps is a power of two.
	 **/
	static bool isPowerOfTwo(unsigned int ps) {

		// count the bit set, see: http://tekpool.wordpress.com/category/bit-count/
		unsigned int bC;

		bC = ps - ((ps >> 1) & 033333333333) - ((ps >> 2) & 011111111111);
		bC = ((bC + (bC >> 3)) & 030707070707) % 63;

		return bC == 1;
	}

	/**
	 * Returns the next power of two.
	 * @param val a number for which the next power of two needs to be computed.
	 * @return the next power of two for val.
	 **/
	static int getNextPowerOfTwo(int val) {

		int pt = 1;
		while (val > pt)
			pt = pt << 1;	// *2

		return pt;
	}

	/**
	 * Returns the value of f(x,sigma) where f is a gaussian.
	 * @param sigma of the gaussian distribution.
	 * @param x param of the gaussian.
	 * @return f(x,sigma) .
	 **/
	static float getGaussian(float sigma, float x) {

		return 1/sqrt(2*(float)CV_PI*sigma*sigma) * exp(-(x*x)/(2*sigma*sigma));
	}

	static bool isLocalExtrema8(float cpx, float* ptr, int cols, int extrema) {

		bool isExtrema = false;

		if (extrema > 0)
			isExtrema = (	cpx >  *(ptr-cols-1)	&&	// upper left
							cpx >  *(ptr-cols)		&&	// upper middle
							cpx >  *(ptr-cols+1)	&&	// upper right
							cpx >  *(ptr-1)			&&	// left
							cpx >  *(ptr+1)			&&	// right
							cpx >  *(ptr+cols-1)	&&	// lower left
							cpx >  *(ptr+cols)		&&	// lower middle
							cpx >  *(ptr+cols+1));
		else if (extrema < 0)
			isExtrema = (	cpx <  *(ptr-cols-1)	&&	// upper left
							cpx <  *(ptr-cols)		&&	// upper middle
							cpx <  *(ptr-cols+1)	&&	// upper right
							cpx <  *(ptr-1)			&&	// left
							cpx <  *(ptr+1)			&&	// right
							cpx <  *(ptr+cols-1)	&&	// lower left
							cpx <  *(ptr+cols)		&&	// lower middle
							cpx <  *(ptr+cols+1));
		else {
			cpx = fabs(cpx);		
			isExtrema = (	cpx >  fabs(*(ptr-cols-1))	&&	// upper left
							cpx >  fabs(*(ptr-cols))	&&	// upper middle
							cpx >  fabs(*(ptr-cols+1))	&&	// upper right
							cpx >  fabs(*(ptr-1))		&&	// left
							cpx >  fabs(*(ptr+1))		&&	// right
							cpx >  fabs(*(ptr+cols-1))	&&	// lower left
							cpx >  fabs(*(ptr+cols))	&&	// lower middle
							cpx >  fabs(*(ptr+cols+1)));
		}

		return isExtrema;
	}

	static bool isLocalExtrema9(float cpx, float* ptr, int cols, int extrema) {
		
		bool isExtrema = false;

		if (extrema > 0)
			isExtrema = (	cpx >  *(ptr-cols-1)	&&	// upper left
							cpx >  *(ptr-cols)		&&	// upper middle
							cpx >  *(ptr-cols+1)	&&	// upper right
							cpx >  *(ptr-1)			&&	// left
							cpx >= *(ptr)			&&	// middle
							cpx >  *(ptr+1)			&&	// right
							cpx >  *(ptr+cols-1)	&&	// lower left
							cpx >  *(ptr+cols)		&&	// lower middle
							cpx >  *(ptr+cols+1));
		else if (extrema < 0)
			isExtrema = (	cpx <  *(ptr-cols-1)	&&	// upper left
							cpx <  *(ptr-cols)		&&	// upper middle
							cpx <  *(ptr-cols+1)	&&	// upper right
							cpx <  *(ptr-1)			&&	// left
							cpx <= *(ptr)			&&	// middle
							cpx <  *(ptr+1)			&&	// right
							cpx <  *(ptr+cols-1)	&&	// lower left
							cpx <  *(ptr+cols)		&&	// lower middle
							cpx <  *(ptr+cols+1));
		else {
			cpx = fabs(cpx);		
			isExtrema = (	cpx >  fabs(*(ptr-cols-1))	&&	// upper left
							cpx >  fabs(*(ptr-cols))	&&	// upper middle
							cpx >  fabs(*(ptr-cols+1))	&&	// upper right
							cpx >  fabs(*(ptr-1))		&&	// left
							cpx >= fabs(*(ptr))			&&	// middle
							cpx >  fabs(*(ptr+1))		&&	// right
							cpx >  fabs(*(ptr+cols-1))	&&	// lower left
							cpx >  fabs(*(ptr+cols))	&&	// lower middle
							cpx >  fabs(*(ptr+cols+1)));
		}

		return isExtrema;
	}

	/**
	 * Fits a spline into the given values.
	 * @param xValues an array consisting of 3 x-values.
	 * @param yValues an array consisting of 3 y-values.
	 * @param xMax a float containing the spline's maximum index.
	 * @param yMax a float containing the spline's maximum value.
	 **/
	static void iplSpline(const float *xValues, float *yValues, float *xMax, float *yMax) {

		float A [9];
		int validEq = 0;

		// (x1^2 x2^2 x3^2; x1 x2 x3; 1 1 1)
		A[0] = xValues[0]*xValues[0];   A[3] = xValues[1]*xValues[1];    A[6] = xValues[2]*xValues[2];
		A[1] = xValues[0];              A[4] = xValues[1];               A[7] = xValues[2];
		A[2] = 1;                       A[5] = 1;                        A[8] = 1;

		Mat matA = Mat(3, 3, CV_32FC1, &A);
		Mat matY = Mat(3, 1, CV_32FC1, yValues);
		Mat result = Mat(3, 1, CV_32FC1);

		validEq = solve(matA, matY, result, CV_LU);

		if (validEq == 1) {
			// system is linear (e.g. all yValues are 0) so return the middle
			float *rp = result.ptr<float>();
			*xMax = (rp[0] != 0.0f) ? -rp[1]/(2*rp[0]) : xValues[1];
			*yMax = *xMax * *xMax * rp[0] + *xMax*rp[1] + rp[2];
		}
		else {
			std::cout << "I could not solve the system..." << std::endl;
			return;
		}

		printf("old spline xMax: %.6f\n", *xMax);

		// lineares Gleichungssystem
		  /*
			  f(x) =  A*x^2 + B * x + C
			  splIvl für x substituieren, C=1 (damit mat nicht singulär werden kann)
			  -> Gleichungssystem
			  (x1^2   x1   1)     (A)    (y1)
			  (x2^2   x2   1)  *  (B) =  (y2)     bzw.   (matA) * (result) = (matyneu)
			  (x3^2   x3   1)     (C)    (y3)
			  -> nach (A,B,C) auflösen, rücksubstituieren und max durch ableitung bestimmen:
			  f(x)' = 2*A*x + B    ->  0 setzen (maxima bestimmen)
			  x = -B/(2*A)
		  */
	}


	/**
	 * Fits a spline into the given values. (x values are set to 1,2,3 and
	 * finally shifted towards firstXVal. This improves the functions stability if
	 * large x values need to be computed.
	 * @param firstXVal an array consisting of 3 x-values.
	 * @param yValues an array consisting of 3 y-values.
	 * @param xMax a float containing the spline's maximum index.
	 * @param yMax a float containing the spline's maximum value.
	 **/
	static void iplSplineEqDist(const float firstXVal, float *yValues, float *xMax, float *yMax) {

		float A [9];
		int validEq = 0;

		// (x1^2 x2^2 x3^2; x1 x2 x3; 1 1 1)
		A[0] = 1.0f;	A[3] = 4.0f;	A[6] = 9.0f;
		A[1] = 1.0f;	A[4] = 2.0f;	A[7] = 3.0f;
		A[2] = 1.0f;	A[5] = 1.0f;	A[8] = 1.0f;

		Mat matA = Mat(3, 3, CV_32FC1, &A);
		Mat matY = Mat(3, 1, CV_32FC1, yValues);
		Mat result = Mat(3, 1, CV_32FC1);

		validEq = solve(matA, matY, result, CV_LU);

		if (validEq == 1) {
			// system is linear (e.g. all yValues are 0) so return the middle
			float *rp = result.ptr<float>();
			*xMax = (rp[0] != 0.0f) ? -rp[1]/(2*rp[0]) : firstXVal+1.0f;
			*xMax += firstXVal-1.0f;
			*yMax = *xMax * *xMax * rp[0] + *xMax*rp[1] + rp[2];
		}
		else {
			std::cout << "I could not solve the system...";
			return;
		}
	}

	static double computePcaAngle(const std::vector<DkVector>& points);
	static double computePcaAngle(const Mat& points);

	template <typename numFmt>
	static numFmt sq(numFmt x) {
		
		return x*x;
	}

	template <typename numFmt>
	static double log2(numFmt x) {

		return log((double)x)/log(2.0);
	}


	/**
	 * Computes robust statistical moments (quantiles).
	 * @param values the statistical set (samples).
	 * @param momentValue the statistical moment value (0.5 = median, 0.25 and 0.75 = quartiles)
	 * @param interpolated is a flag if the value should be interpolated if the length of the list is even
	 * @return the statistical moment.
	 **/
	template <typename numFmt>
	static double statMoment(std::list<numFmt> *values, float momentValue, int interpolated=1) {

		values->sort();
		typename std::list<numFmt>::iterator valIter = values->begin();

		size_t lSize = values->size();
		double moment = -1;
		unsigned int momIdx = cvCeil(lSize*momentValue);
		unsigned int idx = 1;

		// find the statistical moment
		while (valIter != values->end()) {

			// skip
			if (idx < momIdx) {
				valIter++;
				idx++;
				continue;
			}
			if (lSize % 2 == 0 && momIdx < lSize && interpolated==1)
				// compute mean between this and the next element
				moment = ((double)*valIter + *++valIter)*0.5;
			else
				moment = (double)*valIter;
			break;
		}

		return moment;
	}

};

/**
 * A simple 2D vector class DK_CORE_API.
 */
class DK_CORE_API DkVector {

public:
	
	union {
		float x;		/**< the vector's x-coordinate*/
		float width;	/**< the vector's x-coordinate*/
		float r;		/**< radius for log-polar coordinates or red channel*/
		float h;		/**< hue channel*/
	};
	
	union {
		float y;		/**< the vector's y-coordinate*/
		float height;	/**< the vector's y-coordinate*/
		float theta;	/**< angle for log-polar coordinates*/
		float g;		/**< green channel*/
		float l;		/**< luminance channel*/
	};
	
	/** 
	 * Default constructor.
	 **/
	DkVector() : x(0), y(0) {
		empty = true;
	};

	/** 
	 * Initializes an object.
	 * @param x the vector's x-coordinate.
	 * @param y the vector's y-coordinate.
	 **/
	DkVector(float x, float y) {
		this->x = x;
		this->y = y;
		empty = false;
	};

	/**
	 * Initializes an object by means of the OpenCV size.
	 * @param s the size.
	 **/
	DkVector(Size s) {
		this->width  = (float)s.width;
		this->height = (float)s.height;
		empty = false;
	};

	/**
	 * Initializes a Vector by means of a OpenCV Point.
	 * @param p the point
	 **/
	DkVector(Point2f p) {
		this->x = p.x;
		this->y = p.y;
		empty = false;
	};

	/**
	 * Initializes a Vector by means of a OpenCV Point.
	 * @param p the point
	 **/
	DkVector(Point p) {
		this->x = (float)p.x;
		this->y = (float)p.y;
		empty = false;
	};


	/** 
	 * Default destructor.
	 **/
	virtual ~DkVector() {};

	/**
	 * Compares two vectors.
	 * @return true if both vectors have the same coordinates
	 */
	virtual bool operator== (const DkVector &vec) const {

		return (this->x == vec.x && this->y == vec.y);
	};

	/**
	 * Compares two vectors.
	 * @return true if both either the x or y coordinates of both
	 * vectors are not the same.
	 */
	virtual bool operator!= (const DkVector &vec) const {

		return (this->x != vec.x || this->y != vec.y);
	};
	
	/**
	 * Decides which vector is smaller.
	 * If y is < vec.y the function returns true.
	 * Solely if y == vec.y the x coordinates are compared.
	 * @param vec the vector to compare this instance with.
	 * @return true if y < vec.y or y == vec.y && x < vec.y.
	 **/
	virtual bool operator< (const DkVector &vec) const {

		if (y != vec.y)
			return y < vec.y;
		else
			return x < vec.x;
	};

	/**  
	 * Adds vector to the current vector.
	 * @param vec the vector to be added
	 */
	virtual void operator+= (const DkVector &vec) {

		this->x += vec.x;
		this->y += vec.y;
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added
	 */
	virtual void operator+= (const float &scalar) {

		this->x += scalar;
		this->y += scalar;
	};

	/** 
	 * Computes the direction vector between this vector and vec.
	 * Computes the direction vector pointing to the current vector
	 * and replacing it.
	 */
	virtual void operator-= (const DkVector &vec) {

		this->x -= vec.x;
		this->y -= vec.y;
	};
	
	/** 
	 * Subtracts a scalar from the current vector.
	 * @param scalar the scalar which should be subtracted.
	 */
	virtual void operator-= (const float &scalar) {
		
		this->x -= scalar;
		this->y -= scalar;
	};

	/** 
	 * Scalar product.
	 * @param vec a vector which should be considered for the scalar product.
	 * @return the scalar product of vec and the current vector.
	 */ 
	virtual float operator* (const DkVector &vec) const {

		return this->x*vec.x + this->y*vec.y;
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 */
	virtual void operator*= (const float scalar) {
		
		this->x *= scalar;
		this->y *= scalar;
	};

	/** 
	 * Scalar division.
	 * @param scalar a scalar.
	 */
	virtual void operator/= (const float scalar) {
		this->x /= scalar;
		this->y /= scalar;
	};

	// friends ----------------

	/** 
	 * Adds a vector to the current vector.
	 * @param vec the vector which should be added
	 * @return the addition of the current and the given vector.
	 */
	friend DkVector operator+ (const DkVector &vec, const DkVector &vec2) {

		return DkVector(vec.x+vec2.x, vec.y+vec2.y);
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added
	 * @return the addition of the current vector and the scalar given.
	 */
	friend DkVector operator+ (const DkVector &vec, const float &scalar) {

		return DkVector(vec.x+scalar, vec.y+scalar);
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added
	 * @return the addition of the current vector and the scalar given.
	 */
	friend DkVector operator+ (const float &scalar, const DkVector &vec) {

		return DkVector(vec.x+scalar, vec.y+scalar);
	};

	/** 
	 * Computes the direction vector between the given vector and vec.
	 * The direction vector C is computed by means of: C = B-A
	 * where B is the current vector.
	 * @param vec the basis vector A.
	 * @return a direction vector that points from @param vec to the 
	 * current vector.
	 */
	friend DkVector operator- (const DkVector &vec, const DkVector &vec2) {

		return DkVector(vec.x-vec2.x, vec.y-vec2.y);
	};

	/** 
	 * Subtracts a scalar from the current vector.
	 * @param scalar the scalar which should be subtracted.
	 * @return the subtraction of the current vector and the scalar given.
	 */
	friend DkVector operator- (const DkVector vec, const float &scalar) {

		return DkVector(vec.x-scalar, vec.y-scalar);
	};

	/** 
	 * Subtracts the vector from a scalar.
	 * @param scalar the scalar which should be subtracted.
	 * @return the subtraction of the current vector and the scalar given.
	 */
	friend DkVector operator- (const float &scalar, const DkVector vec) {

		return DkVector(scalar-vec.x, scalar-vec.y);
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 * @return the current vector multiplied by a scalar.
	 */
	friend DkVector operator* (const DkVector& vec, const float scalar) {

		return DkVector(vec.x*scalar, vec.y*scalar);
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 * @return the current vector multiplied by a scalar.
	 */
	friend DkVector operator* (const float &scalar, const DkVector& vec) {

		return DkVector(vec.x*scalar, vec.y*scalar);
	};

	/** 
	 * Scalar division.
	 * @param vec a vector which shall be divided.
	 * @param scalar a scalar.
	 * @return the current vector divided by a scalar.
	 */
	friend DkVector operator/ (const DkVector &vec, const float &scalar) {

		return DkVector(vec.x/scalar, vec.y/scalar);
	};

	/** 
	 * Scalar division.
	 * @param scalar a scalar.
	 * @param vec a vector which shall be divided.
	 * @return the current vector divided by a scalar.
	 */
	friend DkVector operator/ (const float &scalar, const DkVector &vec) {

		return DkVector(scalar/vec.x, scalar/vec.y);
	};

	/** 
	 * Vector division.
	 * @param vec1 the numerator.
	 * @param vec2 the denominator.
	 * @return vec1 divided by vec2.
	 */
	friend DkVector operator/ (const DkVector &vec1, const DkVector &vec2) {

		return DkVector(vec1.x/vec2.x, vec1.y/vec2.y);
	};

	/**
	 * Writes the vector r coordinates to the outputstream s.
	 * @param s the outputstream
	 * @param r the vector
	 * @return friend std::ostream& the modified outputstream
	 **/ 
	friend std::ostream& operator<<(std::ostream& s, const DkVector& r) {

		return r.put(s);
	};

	/**
	 * Writes the vector coordinates to the stream s.
	 * @param s the output stream
	 * @return std::ostream& the output stream with the coordinates.
	 **/ 
	virtual std::ostream& put(std::ostream& s) const {

		return s << "[" << x << ", " << y << "]";
	};


	bool isEmpty() const {

		return empty && x == 0 && y == 0;
	};

	/**
	 * Returns the largest coordinate.
	 * @return float the largest coordinate
	 **/ 
	virtual float maxCoord() {

		return max(x, y);
	};

	/**
	 * Returns the largest coordinate.
	 * @return float the largest coordinate.
	 **/ 
	virtual float minCoord() {

		return min(x, y);
	};

	/**
	 * Creates a new vector having the
	 * maximum coordinates of both vectors.
	 * Thus: n.x = max(this.x, vec.x).
	 * @param vec the second vector.
	 * @return a vector having the maximum 
	 * coordinates of both vectors.
	 **/
	virtual DkVector getMaxVec(const DkVector& vec) const {

		return DkVector(max(x, vec.x), max(y, vec.y));
	}

	/**
	 * Creates a new vector having the
	 * minimum coordinates of both vectors.
	 * Thus: n.x = min(this.x, vec.x).
	 * @param vec the second vector.
	 * @return a vector having the minimum
	 * coordinates of both vectors.
	 **/
	virtual DkVector getMinVec(const DkVector& vec) const{

		return DkVector(min(x, vec.x), min(y, vec.y));
	}


	/**
	 * Creates a new vector having the
	 * maximum coordinates of both vectors.
	 * Thus: n.x = max(this.x, vec.x).
	 * @param vec the second vector.
	 **/
	virtual void maxVec(const DkVector& vec) {

		x = max(x, vec.x);
		y = max(y, vec.y);
	}

	/**
	 * Creates a new vector having the
	 * minimum coordinates of both vectors.
	 * Thus: n.x = min(this.x, vec.x).
	 * @param vec the second vector.
	 **/
	virtual void minVec(const DkVector vec) {

		x = min(x, vec.x);
		y = min(y, vec.y);
	}

	/**
	 * Swaps the coordinates of a vector.
	 **/
	void swap() {
		float xtmp = x;
		x = y;
		y = xtmp;
	}

	/**
	 * Returns the vector's angle in radians.
	 * The angle is computed by: atan2(y,x).
	 * @return the vector's angle in radians.
	 **/
	double angle() {
		return atan2(y, x);
	};

	/**
	 * Rotates the vector by a specified angle in radians.
	 * The rotation matrix is: R(-theta) = [cos sin; -sin cos]
	 * @param angle the rotation angle in radians.
	 **/
	void rotate(double angle) {
		
		float xtmp = x;
		x = (float) ( xtmp*cos(angle)+y*sin(angle));
		y = (float) (-xtmp*sin(angle)+y*cos(angle));
	};


	void computeTransformed(DkVector centerR, DkVector centerO, double angle) {

		*this -= centerR;
		rotate(angle);
		*this += centerO;
	};


	/**
	 * Computes the absolute value of both coordinates.
	 **/
	virtual void abs() {

		x = fabs(x);
		y = fabs(y);
	};

	virtual void floor() {
		x = (float)cvFloor(x);
		y = (float)cvFloor(y);
	};

	virtual void ceil() {
		x = (float)cvCeil(x);
		y = (float)cvCeil(y);
	};

	virtual void round() {
		x = (float)cvRound(x);
		y = (float)cvRound(y);
	};

	/**
	 * Clips the vector's coordinates to the bounds given.
	 * @param maxBound the maximum bound.
	 * @param minBound the minimum bound.
	 **/
	virtual void clipTo(float maxBound = 1.0f, float minBound = 0.0f) {

		if (minBound > maxBound) {
			wout << "[DkVector] maxBound < minBound: " << maxBound <<  " < " << minBound << dkendl;
			return;
		}

		if (x > maxBound)		x = maxBound;
		else if (x < minBound)	x = minBound;
		if (y > maxBound)		y = maxBound;
		else if (y < minBound)	y = minBound;
	};

	/**
	 * Clips the vector's coordinates to the bounds given.
	 * @param maxBound the maximum bound.
	 * @param minBound the minimum bound.
	 **/
	virtual void clipTo(const DkVector& maxBound) {

		if (maxBound.x < 0  || maxBound.y < 0) {
			
			DkVector nonConst = maxBound;
			mout << "[WARNING] clipTo maxBound < 0: " << nonConst << dkendl;
			return;
		}

		maxVec(DkVector(0.0f,0.0f));
		minVec(maxBound);
	};

	/**
	 * Convert DkVector to cv::Point.
	 * @return a cv::Point having the vector's coordinates.
	 **/
	virtual Point getCvPoint32f() const {

		return Point_<float>(x, y);
	};

	/**
	 * Convert DkVector to cv::Point.
	 * The vectors coordinates are rounded.
	 * @return a cv::Point having the vector's coordinates.
	 **/
	virtual Point getCvPoint() const {

		return Point(cvRound(x), cvRound(y));
	};

	/**
	 * Convert DkVector to cv::Size.
	 * The vector coordinates are rounded.
	 * @return a cv::Size having the vector's coordinates.
	 **/
	Size getCvSize() const {

		return Size(cvRound(width), cvRound(height));
	}

	
	/** 
	 * Normal vector.
	 * @return a vector which is normal to the current vector
	 * (rotated by 90° counter clockwise).
	 */
	DkVector normalVec() const {

		return DkVector(-y, x);
	};

	/** 
	 * The vector norm.
	 * @return the vector norm of the current vector.
	 */
	virtual float norm() const {
		
		return sqrt(this->x*this->x + this->y*this->y);
	}

	/**
	 * Convenience function.
	 * @return float the vector norm (norm()).
	 **/ 
	virtual float length() const {
		return norm();
	}

	/** 
	 * Normalizes the vector.
	 * After normalization the vector's magnitude is |v| = 1
	 */
	virtual void normalize() {
		float n = norm();
		x /= n; 
		y /= n;
	};

	///** 
	// * Returns the normalized vector.
	// * After normalization the vector's magnitude is |v| = 1
	// * @return the normalized vector.
	// */
	//virtual DkVector getNormalized() const {
	//	float n = norm();

	//	return DkVector(x/n, y/n);
	//};

		
	/** 
	 * Returns the angle between two vectors
	 *  @param vec vector
	 *  @return the angle between two vectors
	 */
	double angle(const DkVector &vec) const {
		return acos(cosv(vec));
	};

	double cosv(const DkVector& vec) const {
		return (this->x*vec.x + this->y*vec.y) / (sqrt(this->x*this->x + this->y*this->y)*sqrt(vec.x*vec.x + vec.y*vec.y));
	};

	/** Returns euclidean distance between two vectors
	 *  @param vec vector
	 *  @return the euclidean distance
	 */
	virtual float euclideanDistance(const DkVector &vec) const {
		return sqrt((this->x - vec.x)*(this->x - vec.x) + (this->y - vec.y)*(this->y - vec.y));
	};
	

	/** 
	 * Scalar product.
	 * @param vec a vector which should be considered for the scalar product.
	 * @return the scalar product of vec and the current vector.
	 */ 
	virtual float scalarProduct(const DkVector& vec) const {

		return this->x*vec.x + this->y*vec.y;
	};

	virtual float vectorProduct(const DkVector& vec) const {

		return x*vec.y - y*vec.x;
	};

	/**
	 * Multiplies thee coordinates.
	 * @param vec a vector to multiply this vector with.
	 * @return DkVector the multiplied vector
	 **/ 
	virtual DkVector mul(const DkVector& vec) const {

		return DkVector(x*vec.x, y*vec.y);
	}

	/** 
	 * String containing the vector's values.
	 * @return a String representing the vector's coordinates: <x, y>
	 */
	virtual std::string toString();
	/** 
	 * Slope of a line connecting two vectors. 
	 * start point is the actual vector, end point the parameter vector
	 * @param vec a vector which should be considered for the slope.
	 * @return the slope between the two points.
	 */ 
	float slope(DkVector vec) {
		return (vec.x - this->x) != 0 ? (vec.y - this->y) / (vec.x - this->x) : FLT_MAX;
	}

private:
	bool empty;

};


/**
 * A simple 3D vector class DK_CORE_API.
 */
 class DK_CORE_API DkVector3 : public DkVector{

public:
	union {
		float z;	/**< z coordinate>*/
		float b;	/**< blue channel>*/
		float s;	/**< saturation channel>*/
	};

	/** 
	 * Default constructor.
	 **/
	DkVector3() : DkVector(), z(0.0f) {};

	/** 
	 * Initializes an object.
	 * @param x the vector's x-coordinate.
	 * @param y the vector's y-coordinate.
	 * @param z the vector's z-coordinate.
	 **/
	DkVector3(float x, float y, float z) : DkVector(x,y) {
		this->z = z;
	};

	/**
	 * Initializes a Vector by means of a OpenCV Point3f.
	 * @param p the point
	 **/
	DkVector3(Point3f p) {
		this->x = p.x;
		this->y = p.y;
		this->z = p.z;
	};


	/**
	 * Initializes a Vector by means of a 2D Vector and a z component.
	 * @param vec a 2D Vector
	 * @param z the vector's z-component
	 **/
	DkVector3(DkVector vec, float z) : DkVector(vec) {
		this->z = z;
	};

	/**
	 * Initializes a Vector by means of an array.
	 * NOTE: the array must be a 3 element array.
	 * @param vec a 3 element array.
	 **/
	DkVector3(float* vec) {

		this->x = vec[0];
		this->y = vec[1];
		this->z = vec[2];

	};

	/** 
	 * Default destructor.
	 **/
	~DkVector3(){};

	/**
	 * Compares two vectors.
	 * @return true if both vectors have the same coordinates
	 */
	bool operator== (const DkVector3 &vec) const {

		return (DkVector::operator== (vec) && this->z == vec.z);
	};

	/**
	 * Compares two vectors.
	 * @return true if all - either x, y or z - of both
	 * vectors are not the same.
	 */
	bool operator!= (const DkVector3 &vec) const {

		return (DkVector::operator!= (vec) || this->z != vec.z);
	};

	/**
	 * Decides which vector is smaller.
	 * If y is < vec.y the function returns true.
	 * Solely if y == vec.y the x coordinates are compared.
	 * @param vec the vector to compare this instance with.
	 * @return true if y < vec.y or y == vec.y && x < vec.y or
	 * y == vec.y && x == vec.x && z < vec.z
	 **/
	bool operator< (const DkVector3 &vec) const {
		
		return (DkVector::operator< (vec) || this->z < vec.z);
	};

	/**  
	 * Adds vector to the current vector.
	 * @param vec the vector to be added
	 */
	void operator+= (const DkVector3 &vec) {

		DkVector::operator+= (vec);
		z += vec.z;
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added.
	 */
	void operator+= (const float &scalar) {

		DkVector::operator+= (scalar);
		z += scalar;
	};

	/** 
	 * Computes the direction vector between this vector and vec.
	 * Computes the direction vector pointing to the current vector
	 * and replacing it.
	 */
	void operator-= (const DkVector3 &vec) {

		DkVector::operator-= (vec);
		z -= vec.z;
	};

	/** 
	 * Subtracts a scalar from the current vector.
	 * @param scalar the scalar which should be subtracted.
	 */
	void operator-= (const float &scalar) {

		DkVector::operator-= (scalar);
		z -= scalar;
	};

	/**
	 * Scalar product (or dot product).
	 * @param vec a vector which should be considered for the scalar product.
	 * @return the scalar product of vec and the current vector.
	 **/
	float operator* (const DkVector3 &vec) const {

		return DkVector::operator* (vec) + z * vec.z;
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 */
	void operator*= (const float &scalar) {

		DkVector::operator*= (scalar);
		z *= scalar;
	};

	/** 
	 * Scalar division.
	 * @param scalar a scalar.
	 */
	void operator/= (const float scalar) {
		
		DkVector::operator/= (scalar);
		z /= scalar;
	};

	// friends ----------------

	/** 
	 * Adds a vector to the current vector.
	 * @param vec the vector which should be added
	 * @return the addition of the current and the given vector.
	 */
	friend DkVector3 operator+ (const DkVector3 &vec, const DkVector3 &vec2) {

		return DkVector3(vec.x+vec2.x, vec.y+vec2.y, vec2.z+vec.z);
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added
	 * @return the addition of the current vector and the scalar given.
	 */
	friend DkVector3 operator+ (const DkVector3 &vec, const float &scalar) {

		return DkVector3(vec.x+scalar, vec.y+scalar, vec.z+scalar);
	};

	/** 
	 * Adds a scalar to the current vector.
	 * @param scalar the scalar which should be added
	 * @return the addition of the current vector and the scalar given.
	 */
	friend DkVector3 operator+ (const float &scalar, const DkVector3 &vec) {

		return DkVector3(vec.x+scalar, vec.y+scalar, vec.z+scalar);
	};

	/** 
	 * Computes the direction vector between the given vector and vec.
	 * The direction vector C is computed by means of: C = B-A
	 * where B is the current vector.
	 * @param vec the basis vector A.
	 * @return a direction vector that points from @param vec to the 
	 * current vector.
	 */
	friend DkVector3 operator- (const DkVector3 &vec, const DkVector3 &vec2) {

		return DkVector3(vec.x-vec2.x, vec.y-vec2.y, vec.z-vec2.z);
	};

	/** 
	 * Subtracts a scalar from the current vector.
	 * @param scalar the scalar which should be subtracted.
	 * @return the subtraction of the current vector and the scalar given.
	 */
	friend DkVector3 operator- (const DkVector3 &vec, const float &scalar) {

		return DkVector3(vec.x-scalar, vec.y-scalar, vec.z-scalar);
	};

	/** 
	 * Subtracts a scalar from the current vector.
	 * @param scalar the scalar which should be subtracted.
	 * @return the subtraction of the current vector and the scalar given.
	 */
	friend DkVector3 operator- (const float &scalar, const DkVector3 &vec) {

		return DkVector3(scalar-vec.x, scalar-vec.y, scalar-vec.z);
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 * @return the current vector multiplied by a scalar.
	 */
	friend DkVector3 operator* (const DkVector3 &vec, const float &scalar) {

		return DkVector3(vec.x*scalar, vec.y*scalar, vec.z*scalar);
	};

	/** 
	 * Scalar multiplication.
	 * @param scalar a scalar.
	 * @return the current vector multiplied by a scalar.
	 */
	friend DkVector3 operator* (const float &scalar, const DkVector3 &vec) {

		return DkVector3(vec.x*scalar, vec.y*scalar, vec.z*scalar);
	};

	/** 
	 * Scalar division.
	 * @param vec a vector which shall be devided.
	 * @param scalar a scalar.
	 * @return the current vector multiplied by a scalar.
	 */
	friend DkVector3 operator/ (const DkVector3 &vec, const float &scalar) {

		return DkVector3(vec.x/scalar, vec.y/scalar, vec.z/scalar);
	};


	/**
	 * Writes the vector coordinates to the stream s.
	 * @param s the output stream
	 * @return std::ostream& the output stream with the coordinates.
	 **/ 
	virtual std::ostream& put(std::ostream& s) {

		return s << "[" << x << ", " << y << ", " << z << "]";
	};



	/**
	 * The vector's cross product.
	 * @param vec a vector with which the cross product should be computed.
	 * @return the normal vector to the plane of both vectors.
	 **/
	DkVector3 crossProduct(const DkVector3 &vec) {

		DkVector3 cp;
		cp.x = y*vec.z - z*vec.y;
		cp.y = z*vec.x - x*vec.z;
		cp.z = x*vec.y - y*vec.x;

		return cp;
	};

	/**
	 * Creates a new vector having the
	 * maximum coordinates of both vectors.
	 * Thus: n.x = max(this.x, vec.x).
	 * @param vec the second vector.
	 * @return a vector having the maximum 
	 * coordinates of both vectors.
	 **/
	DkVector3 getMaxVec(const DkVector3 vec) const {

		return DkVector3(DkVector::getMaxVec(vec), max(z, vec.z));
	};

	/**
	 * Creates a new vector having the
	 * minimum coordinates of both vectors.
	 * Thus: n.x = min(this.x, vec.x).
	 * @param vec the second vector.
	 * @return a vector having the minimum
	 * coordinates of both vectors.
	 **/
	DkVector3 getMinVec(const DkVector3 vec) const {

		return DkVector3(DkVector::getMinVec(vec), min(z, vec.z));
	};

	/**
	 * Computes the absolute value of both coordinates.
	 **/
	void abs() {

		DkVector::abs();
		z = fabs(z);
	};

	/**
	 * Clips the vector's coordinates to the bounds given.
	 * @param maxBound the maximum bound.
	 * @param minBound the minimum bound.
	 **/
	void clipTo(float maxBound = 1.0f, float minBound = 0.0f) {

		if (minBound > maxBound) {
			wout << "[DkVector3] maxBound < minBound: " << maxBound << " < " << minBound << dkendl;
			return;
		}

		DkVector::clipTo(maxBound, minBound);
		if (z > maxBound)		z = maxBound;
		else if (z < minBound)	z = minBound;
	};

	/**
	 * Convert DkVector to cv::Point3f.
	 * @return a cv::Point3f having the vector's coordinates.
	 **/
	Point3f getCvPoint3f() const {

		return Point3_<float>(x, y, z);
	};

	/**
	 * Convert DkVector to cv::Point3i.
	 * The vectors coordinates are rounded.
	 * @return a cv::Point3i having the vector's coordinates.
	 **/
	Point3i getCvPoint3i() const {
		
		return Point3_<int>(cvRound(x), cvRound(y), cvRound(z));
	};

	/**
	 * Converts the vector to a float array.
	 * @param vec the float array. It must be a float[3] array.
	 **/ 
	void toArray(float* vec) {

		vec[0] = x;
		vec[1] = y;
		vec[2] = z;
	};

	/**
	 * Computes the vector's norm.
	 * @return the vector's norm (sqrt(x*x+y*y+z*z)).
	 **/
	float norm() const {

		return sqrt(x*x + y*y + z*z);
	};

	/** 
	 * Normalizes the vector.
	 * After normalization the vector's magnitude is |v| = 1
	 */
	void normalize() {

		float n = norm();
		x /= n;
		y /= n;
		z /= n;
	};

	/** 
	 * Returns euclidean distance between two vectors
	 *  @param vec vector
	 *  @return the euclidean distance
	 */
	float euclideanDistance(const DkVector3 &vec) {

		return sqrt((x-vec.x)*(x-vec.x) + (y-vec.y)*(y-vec.y) + (z-vec.z)*(z-vec.z));
	};
	
	/** 
	 * String containing the vector's values.
	 * @return a String representing the vector's coordinates: <x, y>
	 */
	std::string toString();


	double angle(const DkVector3 &vec) {

		return (DkVector3::operator*(vec))/(this->norm()*vec.norm());
	};



};

/**
 * The class DK_CORE_API represents a line within an image and its properties.
 **/
class DK_CORE_API DkLine {

public:
	/**
	 * Default constructor.
	 * All values are initialized with zero.
	**/
	DkLine();

	/**
	 * Creates a line with the defined starting and end point.
	 * @param start the vector which represents the line's start.
	 * @param end the vector which represents the line's end.
	**/
	DkLine(DkVector start, DkVector end);

	/**
	 * Creates a line with the defined starting and end point.
	 * @param xStart The starting column value of the line.
	 * @param yStart The starting row value of the line.
	 * @param xEnd The ending column value of the line.
	 * @param yEnd The ending row value of the line.
	**/
	DkLine(const float xStart, const float yStart, const float xEnd, const float yEnd);
	/**
	* Creates a line with the defined starting and end point.
	* @param start The starting point of the line.
	* @param end The ending point of the line.
	**/
	DkLine(const Point2f start, const Point2f end);
	/**
	 * Default destructor.
	**/
	~DkLine() {};


	bool operator< (const DkLine &line) const {

		if (line.getCenter().y == this->getCenter().y)
			return this->getCenter().x < line.getCenter().x;
		else
			return this->getCenter().y < line.getCenter().y;

	};

	bool empty() {
		return start.isEmpty() && end.isEmpty();
	};

	/**
	* Computes the squared Euclidean distance between the end point of the class DK_CORE_API and the starting point of line l.
	* @param l	The line to which the distance is calculated.
	**/
	float euclidianDistanceSq(const DkLine& l);

	void computeTransformed(DkVector centerR, DkVector centerO, double angle) {

		// transform rotated rect
		start.computeTransformed(centerR, centerO, angle);
		end.computeTransformed(centerR, centerO, angle);

		// start might be < end
		DkLine l = DkLine(start, end);	// why is there no function for sorting?
		start = l.start;
		end = l.end;
	};

	/**
	* Rotates the line about the center..
	* @param center the rotation center
	* @param angle the rotation angle
	**/
	void rotateLine(DkVector center, double angle);

	void moveLine(DkVector translation) {

		start += translation;
		end += translation;
	}

	/**
	 * Computes the Euclidean distance between the end point of the class DK_CORE_API and the starting point of line l.
	 * @param l	The line to which the distance is calculated.
	 **/
	float euclidianDistance(const DkLine& l);

	virtual DkLine merge(const DkLine& l) {


		Mat dist = Mat(1, 4, CV_32FC1);
		float* ptr = dist.ptr<float>();

		ptr[0] = start.euclideanDistance(l.getStartVector());
		ptr[1] = start.euclideanDistance(l.getEndVector());
		ptr[2] = end.euclideanDistance(l.getStartVector());
		ptr[3] = end.euclideanDistance(l.getEndVector());

		Point maxIdxP;
		minMaxLoc(dist, 0, 0, 0, &maxIdxP);
		int maxIdx = maxIdxP.x;

		DkLine mergedLine;
		switch(maxIdx) {
			case 0: mergedLine = DkLine(start, l.getStartVector()); break;
			case 1: mergedLine = DkLine(start, l.getEndVector());	break;
			case 2: mergedLine = DkLine(end, l.getStartVector());	break;
			case 3: mergedLine = DkLine(end, l.getEndVector());		break;
		}

		return mergedLine;		
	}

	virtual DkLine createGapLine(const DkLine& l) {


		Mat dist = Mat(1, 4, CV_32FC1);
		float* ptr = dist.ptr<float>();

		ptr[0] = start.euclideanDistance(l.getStartVector());
		ptr[1] = start.euclideanDistance(l.getEndVector());
		ptr[2] = end.euclideanDistance(l.getStartVector());
		ptr[3] = end.euclideanDistance(l.getEndVector());

		Point minIdxP;
		minMaxLoc(dist, 0, 0, &minIdxP);
		int minIdx = minIdxP.x;

		DkLine mergedLine;
		switch(minIdx) {
		case 0: mergedLine = DkLine(start, l.getStartVector());	break;
		case 1: mergedLine = DkLine(start, l.getEndVector());	break;
		case 2: mergedLine = DkLine(end, l.getStartVector());	break;
		case 3: mergedLine = DkLine(end, l.getEndVector());		break;
		}

		return mergedLine;		
	}


	/**
	 * Returns the minimal distance between the two lines.
	 * @param l the second line to compute the distance with.
	 * @return the minimal distance
	 **/ 
	float minDistance(const DkLine& l) {

		float dist1 = start.euclideanDistance(l.getStartVector());
		float dist2 = start.euclideanDistance(l.getEndVector());
		dist1 = (dist1 < dist2) ? dist1 : dist2;
		dist2 = end.euclideanDistance(l.getStartVector());
		dist1 = (dist1 < dist2) ? dist1 : dist2;
		dist2 = end.euclideanDistance(l.getEndVector());
		dist1 = (dist1 < dist2) ? dist1 : dist2;

		return dist1;
	}

	/**
	* Computes the len of a line
	* @return the line len
	**/
	float getLen();

	double getAngle() {

		DkVector line = end - start;

		return line.angle();
	};

	DkVector getCenter() const;

	/**
	* Sets the start point of a line.
	* @param x The column value of the start point of the line.
	* @param y The row value of the start point of the line.
	**/
	void setStart(const float x, const float y);
	/**
	* Sets the end point of a line.
	* @param x The column value of the end point of the line.
	* @param y The row value of the end point of the line.
	**/
	void setEnd(const float x, const float y);
	/**
	* Returns the column value (xStart) of the start point of the line.
	* @return The column value (xStart) of the start point of the line.
	**/
	float getXS() const {return start.x;};
	/**
	* Returns the row value (yStart) of the start point of the line.
	* @return The row value (yStart) of the start point of the line.
	**/
	float getYS() const {return start.y;};
	/**
	* Returns the column value (xEnd) of the end point of the line.
	* @return The column value (xEnd) of the end point of the line.
	**/
	float getXE() const {return end.x;};
	/**
	* Returns the row value (yEnd) of the end point of the line.
	* @return The row value (yEnd) of the end point of the line.
	**/
	float getYE() const {return end.y;};
	/**
	* Returns the start point of the line as OpenCV Point.
	* @return start point of the linr.
	**/
	Point getStartPoint() const;
	/**
	* Returns the end point of the line as OpenCV Point.
	* @return end point of the line.
	**/
	Point getEndPoint() const;
	/**
	* Returns the start point of the line as an vector.
	* @return start point of the line as DkVector.
	**/
	DkVector getStartVector() const;
	/**
	* Returns the end point of the line as an vector.
	* @return end point of the line as DkVector.
	**/
	DkVector getEndVector() const;
	/**
	* Returns the slope of a line defined as dy/dx.
	* @return The slope value of the line.
	**/
	float getSlope() const {return slope;};
	/**
	* Returns the column value (xStart) of the start point of the line casted to int.
	* @return The column value (xStart) of the start point of the line.
	**/
	int intXS() const {return (int)cvRound(start.x);};
	/**
	* Returns the row value (yStart) of the start point of the line casted to int.
	* @return The row value (yStart) of the start point of the line.
	**/
	int intYS() const {return (int)cvRound(start.y);};
	/**
	* Returns the column value (xEnd) of the end point of the line casted to int.
	* @return The column value (xEnd) of the end point of the line.
	**/
	int intXE() const {return (int)cvRound(end.x);};
	/**
	* Returns the row value (yEnd) of the end point of the line casted to int.
	* @return The row value (yEnd) of the end point of the line.
	**/
	int intYE() const {return (int)cvRound(end.y);};

	/**
	 * Calculates the normal distance to the point given.
	 * @param p a point.
	 * @return the normal distance to the point p.
	 **/
	float distance(DkVector p) {

		DkVector n = DkVector(end-start).normalVec();
		return abs((n*DkVector(p-end))/(FLT_EPSILON+n.norm()));
	};

	/**
	 * Calculates the normal distance to the point given.
	 * @param p a point.
	 * @return the normal distance to the point p. if > 0 -> counter clockwise, if < 0 clockwise
	 **/
	float signedDistance(DkVector p) const {

		DkVector n = DkVector(end-start).normalVec();
		return (n*DkVector(p-end))/(FLT_EPSILON+n.norm());
	};

	/**
	 * Decides whether the point is clockwise, counter clockwise or on the line.
	 * @param p a point whose relative position to the line should be computed.
	 * @return a value > 0 if the point is clockwise, a value < 0 if it is counter clockwise
	 * a value == 0 if it is on the line.
	 **/
	float relativePosition(DkVector p) const {
		
		DkVector g = end-start;	// position vector line
		g = g.normalVec();

		return -1.0f*g*p;				// if g*p > 0 -> counter clockwise, g*p < 0 clockwise, g*p == 0 point is element of the line
	};

	void sampleLine(std::vector<DkVector>& dots, float ivl = 20.0f) {

			dots.push_back(start);
			
			DkVector rv = end-start;
			float length = rv.norm();
			rv.normalize();
			rv *= ivl;	// tick
			DkVector base = start;

			for (int idx = 0; idx < length/20.0f; idx++) {
				dots.push_back(base+rv);
				base = base+rv;
			}
			
			dots.push_back(end);
	}

	/**
	* Decides whether a point is inside the two normals at the end of the line (at any distance)
	* @param p a point whose position with the two normals should be computer
	* @return true if the point lies within the two normals at the endpoints of the line (or the point is on the line), false if not
	*/
	bool within(const DkVector& p) const {
		 return (end-start)*(p-end) * (end-start)*(p-start) < 0;
	};

	/**
	* Decides whether a line given is covered by this line. 
	* Covered means that either one (or both) endpoints of the line given is inside the normals at the end or the line is fully covered
	* @param l the line which should be determined whether it is covered or not
	* @return true if the line is covered (at least one endpoint is inside the normals at the end, or the line is fully covered), false if the line is lying outside
	*/
	bool covers(DkLine l) {
		float sEnd =(end-start)*(l.getStartVector()-end);
		float sStart = (end-start)*(l.getStartVector()-start);
		float eEnd =(end-start)*(l.getEndVector()-end);
		float eStart =  (end-start)*(l.getEndVector()-start);

		return (sEnd*sStart < 0) ||(eEnd*eStart < 0) ||(sEnd * eEnd < 0); // l.start left start and l.end right of end

		
	}

	/**
	* Extends a Vector within a BB defined by leftUp and rightDn to the borders.
	* Vector must be within the defined BB.
	* @param leftUp the left upper corner of the BB.
	* @param rightDn the right down corner of the BB.
	* @return the extended line.
	*/
	DkLine extendBorder(DkVector leftUp, DkVector rightDn);
	
	DkVector getLineInterSect(const DkLine& line);

	/**
	* untested!!!
	* Cuts the line with a horizontal or vertical line.
	* @param line the line
	* @return cut point.
	*/
	DkVector getLineInterSect(DkVector line);
	/**
	 * Swaps start and end point.
	 **/
	void swap();

	friend std::ostream& operator<<(std::ostream& s, DkLine& b){

		// this makes the operator<< virtual (stroustrup)
		return s << b.toString();
	};

	virtual std::string toString();

protected:

	DkVector start;	/**< start point**/
	DkVector end;	/**< end point***/

	float slope;	/**< slope of the line defined as dy/dx. stored to save computing time **/
};

/**
 * The extended line class DK_CORE_API represents a line within an image and its properties. In addition to its base class DK_CORE_API DkLine it stores the moment based orientation
 * and the thickness of a line.
 **/
class DK_CORE_API DkLineExt : public DkLine {

public:
	/**
	 * Default constructor.
	 * All values are initialized with zero.
	**/
	DkLineExt();
	/**
	 * Creates a line with the defined starting and end point, an orientation and thickness.
	 * @param xStart The starting column value of the line.
	 * @param yStart The starting row value of the line.
	 * @param xEnd The ending column value of the line.
	 * @param yEnd The ending row value of the line.
	 * @param orientation The moment based orientation.
	 * @param thickness The thickness of a line in pixel.
	**/
	DkLineExt(const float xStart, const float yStart, const float xEnd, const float yEnd, const float orientation, const float thickness);
	/**
	* Creates a line with the defined starting and end point.
	* @param start The starting point of the line.
	* @param end The ending point of the line.
	* @param orientation The moment based orientation.
	* @param thickness The thickness of a line in pixel.
	**/
	DkLineExt(const Point2f start, const Point2f end, const float orientation, const float thickness);

	/**
	* Creates a line with the defined starting and end point.
	* @param start The starting point of the line.
	* @param end The ending point of the line.
	* @param orientation The moment based orientation.
	* @param thickness The thickness of a line in pixel.
	**/
	DkLineExt(const DkVector start, const DkVector end, const float orientation, const float thickness);

	/**
	 * Default destructor.
	**/
	~DkLineExt() {};
	/**
	* Sets the orientation of a line.
	* @param o The moment based orientation
	**/
	void setOrientation(float o);
	/**
	* Sets the thickness of a line.
	* @param t The thickness of a line in pixel
	**/
	void setThickness(float t);
	/**
	* Returns the orientation of a line.
	* @return The moment based orientation
	**/
	float getOrientation() const {return orientation;};
	/**
	* Returns the thickness of a line.
	* @return The thickness of a line in pixel
	**/
	float getThickness() const {return thickness;};

	void setLineWeight(float w);
	float getLineWeight() {return lineWeight;};

	DkLineExt merge(const DkLineExt& l) {


		Mat dist = Mat(1, 4, CV_32FC1);
		float* ptr = dist.ptr<float>();

		ptr[0] = start.euclideanDistance(l.getStartVector());
		ptr[1] = start.euclideanDistance(l.getEndVector());
		ptr[2] = end.euclideanDistance(l.getStartVector());
		ptr[3] = end.euclideanDistance(l.getEndVector());

		Point maxIdxP;
		minMaxLoc(dist, 0, 0, 0, &maxIdxP);
		int maxIdx = maxIdxP.x;

		float thickness = this->thickness < l.getThickness() ? this->thickness : l.getThickness();

		DkLineExt mergedLine;
		switch(maxIdx) {
		case 0: mergedLine = DkLineExt(start, l.getStartVector(), 0, thickness);	break;
		case 1: mergedLine = DkLineExt(start, l.getEndVector(), 0, thickness);		break;
		case 2: mergedLine = DkLineExt(end, l.getStartVector(), 0, thickness);		break;
		case 3: mergedLine = DkLineExt(end, l.getEndVector(), 0, thickness);		break;
		}

		return mergedLine;		
	}

	DkLineExt createGapLine(const DkLineExt& l) {


		Mat dist = Mat(1, 4, CV_32FC1);
		float* ptr = dist.ptr<float>();

		ptr[0] = start.euclideanDistance(l.getStartVector());
		ptr[1] = start.euclideanDistance(l.getEndVector());
		ptr[2] = end.euclideanDistance(l.getStartVector());
		ptr[3] = end.euclideanDistance(l.getEndVector());

		Point minIdxP;
		minMaxLoc(dist, 0, 0, &minIdxP);
		int minIdx = minIdxP.x;

		float thickness = this->thickness < l.getThickness() ? this->thickness : l.getThickness();

		DkLineExt mergedLine;
		switch(minIdx) {
		case 0: mergedLine = DkLineExt(start, l.getStartVector(), 0, thickness);	break;
		case 1: mergedLine = DkLineExt(start, l.getEndVector(), 0, thickness);		break;
		case 2: mergedLine = DkLineExt(end, l.getStartVector(), 0, thickness);		break;
		case 3: mergedLine = DkLineExt(end, l.getEndVector(), 0, thickness);		break;
		}

		return mergedLine;		
	}

	virtual std::string toString();


private:
	float orientation;		//moment based orientation.
	float thickness;		//thickness of the line.
	float lineWeight;
};

class DK_CORE_API DkRectCorners;

/**
 * Box class DK_CORE_API, defines a non-skewed rectangle e.g. Bounding Box
 **/
class DK_CORE_API DkBox {

public:

	/**
	 * Default constructor.
	 * All values are initialized with zero.
	**/
	DkBox() : uc(), lc() {};

	/**
	 * Constructor.
	 * @param uc the upper left corner of the box.
	 * @param size the size of the box.
	**/
	DkBox(DkVector uc, DkVector size) {

		this->uc = uc;
		this->lc = uc+size;

		if (size.width < 0 || size.height < 0)
			wout << "the size is < 0: " << size << dkendl;
	};
	/**
	 * Constructor.
	 * @param x value of the upper left corner.
	 * @param y value of the upper left corner.
	 * @param width of the box.
	 * @param height of the box.
	**/
	DkBox(float x, float y, float width, float height) {

		DkVector size = DkVector(width, height);

		uc = DkVector(x,y);
		lc = uc+size;

		if (size.width < 0 || size.height < 0)
			wout << "the size is < 0: " << size << dkendl;

	};
	/**
	 * Constructor.
	 * @param r box as rect with upperleft corner and width and height.
	**/
	DkBox(Rect r) {
		
		DkVector size((float)r.width, (float)r.height);

		uc.x = (float)r.x;
		uc.y = (float)r.y;

		lc = uc+size;

		if (size.width < 0 || size.height < 0)
			wout << "the size is < 0: " << size << dkendl;

	};
	/**
	 * Constructor.
	 * @param r box as rect with upperleft corner and width and height.
	 **/
	DkBox(DkRectCorners &r);

	/**
	 * Constructor.
	 * @param b box as DkBox.
	**/
	DkBox(const DkBox &b) {

		this->uc = b.uc;
 		this->lc = b.uc + b.size();

		if (size().width < 0 || size().height < 0)
			wout << "the size is < 0: " << size() << dkendl;
	}
	/**
	 * Default destructor.
	**/
	~DkBox(){};

	void getStorageBuffer(char** buffer, size_t& length) const {


		size_t newBufferLength = length + 4*sizeof(float);
		char* newStream = new char[newBufferLength];

		if (*buffer) {

			// copy old stream & clean it
			memcpy(newStream, *buffer, length);
			delete *buffer;
		}

		float* newFStream = (float*)newStream;

		int pos = 0;
		newFStream[pos] = uc.x; pos++;
		newFStream[pos] = uc.y; pos++;
		newFStream[pos] = lc.x; pos++;
		newFStream[pos] = lc.y; pos++;

		*buffer = newStream;
		length = newBufferLength;
	}

	const char* setSorageBuffer(const char* buffer) {

		const float* fBuffer = (const float*)buffer;
		int pos = 0;
		uc.x = fBuffer[pos]; pos++;
		uc.y = fBuffer[pos]; pos++;
		lc.x = fBuffer[pos]; pos++;
		lc.y = fBuffer[pos]; pos++;

		return buffer+sizeof(float)*pos;	// update buffer position
	}

	//friend std::ostream& operator<<(std::ostream& s, DkBox& b) - original
	friend std::ostream& operator<<(std::ostream& s, DkBox b)
	{

		// this makes the operator<< virtual (stroustrup)
		return s << b.toString();
	};
	
	void moveBy(const DkVector& dxy) {

		uc += dxy;
		lc += dxy;
	};

	bool isEmpty() const {


		return uc.isEmpty() && lc.isEmpty();
	}

	/**
	 * Returns the box as opencv Rect.
	 * @return a box as opencv Rect.
	**/
	Rect getCvRect() const {

		return Rect(cvRound(uc.x), cvRound(uc.y), cvRound(size().width), cvRound(size().height));
	}

	static DkBox contour2BBox(const std::vector<std::vector<cv::Point> >& pts) {
		
		if (pts.empty())
			return DkBox();

		// TODO: write this in dk style
		int ux = INT_MAX, uy = INT_MAX;
		int lx = 0, ly = 0;

		for (int cIdx = 0; cIdx < pts.size(); cIdx++) {

			const std::vector<Point>& cont = pts[cIdx];

			for (int idx = 0; idx < cont.size(); idx++) {

				Point p = cont[idx];

				if (p.x < ux)
					ux = p.x;
				if (p.x > lx)
					lx = p.x;
				if (p.y < uy)
					uy = p.y;
				if (p.y > ly)
					ly = p.y;
			}
		}
		DkBox rect((float)ux, (float)uy, (float)lx-ux, (float)ly-uy);

		return rect;
	}
	
	/**
	 * Enlarges the box by the given offset, and the upperleft corner is recalculated.
	 * @param offset by which the box is expanded.
	**/
	void expand(float offset) {

		uc -= (offset*0.5f);
	}
	
	/**
	 * Clips the box according the vector s (the box is only clipped but not expanded).
	 * @param s the clip vector.
	**/
	void clip(DkVector s) {
		
		uc.round();
		lc.round();
		
		uc.clipTo(s);
		lc.clipTo(s);

		if (lc.x > s.x || lc.y > s.y)
			mout << "I did not clip..." << dkendl;
	};

	bool within(const DkVector& p) const {

		return (p.x >= uc.x && p.x < lc.x && 
				p.y >= uc.y && p.y < lc.y);
	};

	DkVector center() const {
		return uc + size() * 0.5f;
	};

	void scaleAboutCenter(float s) {

		DkVector c = center();

		uc = DkVector(uc-c)*s+c;
		lc = DkVector(lc-c)*s+c;
	};

	/**
	 * Returns the x value of the upper left corner.
	 * @return x value in pixel of the upperleft corner.
	**/
	int getX() const {
		return cvRound(uc.x);
	};
	/**
	 * Returns the y value of the upper left corner.
	 * @return y value in pixel of the upperleft corner.
	**/
	int getY() const {
		return cvRound(uc.y);
	};
	/**
	 * Returns the width of the box.
	 * @return the width in pixel of the box.
	**/
	int getWidth() const {
		return cvRound(lc.x-uc.x);
	};
	/**
	 * Returns the width of the box.
	 * @return float the width in pixel fo the box.
	 **/ 
	float getWidthF() const {
		return lc.x-uc.x;
	};
	/**
	 * Returns the height of the box.
	 * @return the height in pixel of the box.
	**/
	int getHeight() const {
		return cvRound(lc.y-uc.y);
	};
	/**
	 * Returns the height of the box as float
	 * @return float height in pixel of the box.
	 **/ 
	float getHeightF() const {
		return lc.y-uc.y;
	};
	/**
	 * Returns the size of the box.
	 * @return size of the box as opencv Size.
	**/
	Size getSize() const {
		return Size(getWidth(), getHeight());
	};

	DkVector size() const {
		
		return lc-uc;
	};

	void setSize(DkVector size) {

		lc = uc+size;
	};

	float area() const {

		DkVector s = size();
		return s.width*s.height;
	};

	float intersectArea(const DkBox& box) const {

		DkVector tmp1 = lc.getMaxVec(box.lc);
		DkVector tmp2 = uc.getMaxVec(box.uc);
		
		// no intersection?
		if (lc.x < uc.x || lc.y < lc.y)
			return 0;

		tmp1 = tmp2-tmp1;
		
		return tmp1.width*tmp1.height;
	};



	//const DkVector& ucNew() const {
	//	return uc;
	//};

	//DkVector& ucNew() {
	//	return const_cast<DkVector&>(static_cast<const DkBox*>(this)->ucNew());
	//};

	std::string toString() {

		std::string msg =	"\n upper corner: " + uc.toString();
		msg +=				"\n size:         " + size().toString();

		return msg;
	}

//protected:


	DkVector uc;		/**< upper left corner of the box **/
	DkVector lc;		/**< lower right corner of the box **/
};

/**
 * rectangle class DK_CORE_API is an arbitrary rectangle defined by center, size and skew angle.
 * NOTE: 0° is the horizontal axis
 **/
class DK_CORE_API DkRect {

public:

	DkVector center;	/**< the center of the rectangle */
	DkVector size;		/**< the size of the rectangle */
	double angle;		/**< the skew angle of the rectangle in radians */

	/**
	 * Default constructor.
	 * All values are initialized with zero.
	**/
	DkRect() : center(), size(), angle(0) {};

	/**
	 * Constructor, initializes DkRect with an rectangle defined by an opencv Rect, the angle is set to zero.
	 * @param r an opencv Rect.
	**/
	DkRect(Rect &r) {

		this->size = DkVector((float)r.width, (float)r.height);
		this->center = DkVector((float)r.x, (float)r.y) + (this->size*0.5f);
		this->angle = 0.0;

		checkDims();
	}

	DkRect(DkVector &size, DkVector &center, double &angle) {

		this->size = size;
		this->center = center;
		this->angle = angle;

		checkDims();
	}

	/**
	 * Constructor, initializes DkRect with a DkRect.
	 * @param r a DkRect.
	**/
	DkRect(const DkRect &r) {

		if (&r == 0) 
			return;

		this->center = r.center;
		this->size = r.size;
		this->angle = r.angle;

		checkDims();
	};

	/**
	 * Constructor, initializes DkRect with an opencv RotatedRect.
	 * @param cvRect an opencv RotatedRect.
	**/
	DkRect(RotatedRect cvRect) {

		this->center = DkVector(cvRect.center.x, cvRect.center.y);
		this->size = DkVector(cvRect.size.width, cvRect.size.height);
		this->angle = cvRect.angle;

		checkDims();
	};

	DkRect(const DkBox &box) {

		this->center = box.uc+(box.size()*0.5f);
		this->size = box.size();
		this->angle = 0.0;

		checkDims();
	};

	bool operator== (const DkRect &r) const {

		return (this->center == r.center && this->size == r.size && this->angle == r.angle);
	};

	/**
	 * Writes the DkRect values to the outputstream s.
	 * @param s the outputstream
	 * @param r the vector
	 * @return friend std::ostream& the modified outputstream
	 **/ 
	friend std::ostream& operator<<(std::ostream& s, DkRect& r){

		return s << r.toString();
	};

	/**
	 * Check the dimensions of a rectangle. If height > width the angle is corrected by 90°.
	**/
	void checkDims() {

		if (size.height > size.width) {
			size = DkVector(size.height, size.width);
			angle -= CV_PI*0.5;
		}

	};

	bool isEmpty() {
		return center.isEmpty() && size.isEmpty() && angle == 0;
	};

	/**
	 * Returns the dimension, the position and the skew of the rectangle as string.
	 * @return the rectangle as string.
	**/
	std::string toString();

	/**
	 * Converts the DkRect to an opencv RotatedRect.
	 * @return the rectangle as an opencv RotatedRect.
	**/
	RotatedRect getRotatedRect() {

		RotatedRect r;
		r.center.x = center.x;
		r.center.y = center.y;

		r.size.width = size.width;
		r.size.height = size.height;

		r.angle = (float)angle;

		return r;
	};

	double area() {

		return size.width * size.height;
	};

	~DkRect() {};

};

/**
 * rectangle class DK_CORE_API defines an arbitrary rectangle defined by the corners of the rectangle.
 **/
class DK_CORE_API DkRectCorners {

public:

	DkVector a;		/**< corner points of the rectangle, specified clockwise (a,b,c,d) */
	DkVector b;		/**< corner points of the rectangle, specified clockwise (a,b,c,d) */
	DkVector c;		/**< corner points of the rectangle, specified clockwise (a,b,c,d) */
	DkVector d;		/**< corner points of the rectangle, specified clockwise (a,b,c,d) */

	/**
	 * Default constructor.
	 * All values are initialized with zero.
	**/
	DkRectCorners () : a(), b(), c(), d() {};

	/**
	 * Constructor, whereas the corner points of the rectangle are specified clockwise by a, b, c, d.
	 * @param a  a corner point.
	 * @param b  a corner point.
	 * @param c  a corner point.
	 * @param d  a corner point.
	**/
	DkRectCorners(DkVector a, DkVector b, DkVector c, DkVector d) {
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}
	/**
	 * Constructor, the rectangle is specified by a DkRectCorners.
	 * @param rect DkRectCorners rectangle.
	**/
	DkRectCorners(const DkRectCorners &rect) {

		this->a = rect.a;
		this->b = rect.b;
		this->c = rect.c;
		this->d = rect.d;
	};
	/**
	 * Constructor, the rectangle is specified by a DkRect.
	 * @param rect DkRect rectangle.
	**/
	DkRectCorners(const DkRect& rect) {
		
		//// rotate: x' = x*cos(a) - y*sin(a) -> y = 0;
		////         y' = x*sin(a) + y*cos(a) -> y = 0;
		//DkVector xVec;
		//xVec.x = (float)(rect.size.height*0.5*cos(-rect.angle));
		//xVec.y = (float)(rect.size.height*0.5*sin(-rect.angle));

		//// rotate: x' = x*cos(a) - y*sin(a) -> x = 0;
		////         y' = x*sin(a) + y*cos(a) -> x = 0;
		//DkVector yVec;
		//yVec.x = (float)(-rect.size.width*0.5*sin(-rect.angle));
		//yVec.y = (float)( rect.size.width*0.5*cos(-rect.angle));

		DkVector xVec = DkVector(rect.size.width * 0.5f, 0);
		xVec.rotate(-rect.angle);

		DkVector yVec = DkVector(0, rect.size.height * 0.5f);
		yVec.rotate(-rect.angle);

		a = rect.center - xVec - yVec;
		b = rect.center + xVec - yVec;
		c = rect.center + xVec + yVec;
		d = rect.center - xVec + yVec;
		// TODO bug wenn von CV::Rect umgewandelt wird ... werte werden negativ
	};

	// -> old

	///**
	// * Constructor, the rectangle is specified by a RotatedRect.
	// * @param rect RotatedRect rectangle.
	//**/
	//DkRectCorners(RotatedRect rect) {

	//	// rotate: x' = x*cos(a) - y*sin(a) -> y = 0;
	//	//         y' = x*sin(a) + y*cos(a) -> y = 0;
	//	DkVector xVec;
	//	xVec.x = (float)(rect.size.height*0.5*cos(-rect.angle));
	//	xVec.y = (float)(rect.size.height*0.5*sin(-rect.angle));

	//	// rotate: x' = x*cos(a) - y*sin(a) -> x = 0;
	//	//         y' = x*sin(a) + y*cos(a) -> x = 0;
	//	DkVector yVec;
	//	yVec.x = (float)(-rect.size.width*0.5*sin(-rect.angle));
	//	yVec.y = (float)( rect.size.width*0.5*cos(-rect.angle));

	//	
	//	a.x = rect.center.x - xVec.x + yVec.x;
	//	a.y = rect.center.y - xVec.y + yVec.y;

	//	b.x = rect.center.x + xVec.x + yVec.x;
	//	b.y = rect.center.y + xVec.y + yVec.y;

	//	c.x = rect.center.x + xVec.x - yVec.x;
	//	c.y = rect.center.y + xVec.y - yVec.y;

	//	d.x = rect.center.x - xVec.x - yVec.x;
	//	d.y = rect.center.y - xVec.y - yVec.y;

	//};

	/**
	 * Converts the rectangle to an opencv RotatedRect.
	 * @return opencvRotatedRect.
	 **/
	RotatedRect getRotatedRect() {
		
		RotatedRect rect;

		DkVector xVec = this->a - this->b;
		DkVector yVec = this->a - this->d;

		// compute width & height
		float nX = xVec.norm();
		float nY = yVec.norm();

		// compute center
		DkVector center = (this->c + (xVec*0.5f));
		center += (yVec*0.5f);

		rect.size.height = (nX > nY) ? nX : nY;
		rect.size.width  = (nX > nY) ? nY : nX;

		rect.center.x = center.x;
		rect.center.y = center.y;

		rect.angle = (nX > nY) ? (float)yVec.angle() : (float)xVec.angle();

		return rect;

	};

	/**
	 * Writes the rectangles r coordinates to the outputstream s.
	 * @param s the outputstream
	 * @param r the vector
	 * @return friend std::ostream& the modified outputstream
	 **/ 
	friend std::ostream& operator<<(std::ostream& s, DkRectCorners& r) {

		return s << r.toString();
	};

	DkVector getLeftMiddle(double angle = 0.0) {

		if (angle == DBL_MAX)
			angle = 0.0;

		// tested
		DkVector ab = b-a;
		double abAngle = DkMath::normAngleRad(ab.angle())-CV_PI*0.25;
		double gAngle = DkMath::normAngleRad(angle);
		double angleDist = DkMath::normAngleRad(gAngle-abAngle);

		if (angleDist <= CV_PI*0.5)
			return a + (d - a) * 0.5f;
		else if (angleDist <= CV_PI)
			return b + (a - b) * 0.5f;
		else if (angleDist < CV_PI*1.5)
			return c + (b - c) * 0.5f;
		else
			return d + (c - d) * 0.5f;
	}

	DkVector getRightMiddle(double angle = 0.0) {

		if (angle == DBL_MAX)
			angle = 0.0;
		// tested
		DkVector ab = b-a;
		double abAngle = DkMath::normAngleRad(ab.angle())-CV_PI*1.25;
		double gAngle = DkMath::normAngleRad(angle);
		double angleDist = DkMath::normAngleRad(gAngle-abAngle);

		if (angleDist <= CV_PI*0.5)
			return a + (d - a) * 0.5f;
		else if (angleDist <= CV_PI)
			return b + (a - b) * 0.5f;
		else if (angleDist < CV_PI*1.5)
			return c + (b - c) * 0.5f;
		else
			return d + (c - d) * 0.5f;
	}


	void getUpperPoints(DkVector &leftUppr, DkVector &rightUppr, const double angle = 0.0) {
		// untested
		DkVector ab = b-a;
		double abAngle = DkMath::normAngleRad(ab.angle())-CV_PI*0.25;
		double gAngle = DkMath::normAngleRad(angle);
		double angleDist = DkMath::normAngleRad(gAngle-abAngle);

		if (angleDist <= CV_PI*0.5) {
			 leftUppr = a;
			 rightUppr = b;
		} else if (angleDist <= CV_PI) {
			leftUppr = b;
			rightUppr = c;
		} else if (angleDist < CV_PI*1.5) {
			leftUppr = c;
			rightUppr = d;
		} else {
			leftUppr = d;
			rightUppr = a;
		}
	}

	void getLowerPoints(DkVector &leftLower, DkVector &rightLower, const double angle = 0.0) {
		// untested
		DkVector ab = b-a;
		double abAngle = DkMath::normAngleRad(ab.angle())-CV_PI*0.25;
		double gAngle = DkMath::normAngleRad(angle);
		double angleDist = DkMath::normAngleRad(gAngle-abAngle);

		if (angleDist <= CV_PI*0.5) {
			leftLower = d;
			rightLower = c;
		} else if (angleDist <= CV_PI) {
			leftLower = a;
			rightLower = d;
		} else if (angleDist < CV_PI*1.5) {
			leftLower = b;
			rightLower = a;
		} else {
			leftLower = c;
			rightLower = b;
		}
	}

	/**
	 * Converts the rectangle to a DkRect.
	 * @return DkRect Rectangle.
	**/
	DkRect getDkRect() {
		

		DkVector xVec = a - b;
		DkVector yVec = a - d;

		DkVector center = (c + (xVec*0.5f)) + (yVec*0.5f);
		DkVector size = DkVector(xVec.norm(), yVec.norm());
		double angle = xVec.angle();

		DkRect rect = DkRect(size, center, angle);

		return rect;

	};

	/**
	 * Calculated a bounding rectangle for the DkRectCorners rectangle.
	 * @return Bounding box as DkBox.
	**/
	DkBox getBoundingRect() const {

		// find the lower left corner
		float minAB = min(a.x, b.x);
		float minCD = min(c.x, d.x);
		float lcx = min(minAB, minCD);
		if (lcx < 0) lcx = 0.0f;

		minAB = min(a.y, b.y);
		minCD = min(c.y, d.y);
		float lcy = min(minAB, minCD);
		if (lcy < 0) lcy = 0.0f;

		// find the upper right corner
		float maxAB = max(a.x, b.x);
		float maxCD = max(c.x, d.x);
		float ucx = max(maxAB, maxCD);

		maxAB = max(a.y, b.y);
		maxCD = max(c.y, d.y);
		float ucy = max(maxAB, maxCD);

		
		return DkBox(DkVector((float)cvRound(lcx), (float)cvRound(lcy)), DkVector((float)cvRound(ucx-lcx), (float)cvRound(ucy-lcy)));
	};

	/**
	 * Returns the corner points of the DkRectCorners as a vector, whereas the corners are stored as opencv Point2f (for cv::minAreaRect).
	 * @return corner points clockwise as a std::vector<Point2f>.
	**/
	std::vector<Point2f> getPoints32f() const {

		std::vector<Point2f> pts;
		pts.push_back(a.getCvPoint32f());
		pts.push_back(b.getCvPoint32f());
		pts.push_back(c.getCvPoint32f());
		pts.push_back(d.getCvPoint32f());

		return pts;
	};

	std::vector<Point> getPoints() const {

		std::vector<Point> pts;
		pts.push_back(a.getCvPoint());
		pts.push_back(b.getCvPoint());
		pts.push_back(c.getCvPoint());
		pts.push_back(d.getCvPoint());

		return pts;
	};

	std::vector<DkVector> getCorners() const {

		std::vector<DkVector> crn;
		crn.push_back(a);
		crn.push_back(b);
		crn.push_back(c);
		crn.push_back(d);

		return crn;
	}

	/**
	 * Tests if a point specified by DkVector lies within the specified rectangle or not.
	 * @return true if the point is within the rectangle.
	**/
	bool within(DkVector point) {
		
		DkVector e = b-a;
		DkVector f = d-a;

		if ((point.x-a.x)*e.x + (point.y-a.y)*e.y < 0)	return false;
		if ((point.x-b.x)*e.x + (point.y-b.y)*e.y > 0)	return false;
		if ((point.x-a.x)*f.x + (point.y-a.y)*f.y < 0)	return false;
		if ((point.x-d.x)*f.x + (point.y-d.y)*f.y > 0)	return false;

		return true;
	};


	
	/**
	 * Rotate the rectangle around the coordinate origin.
	 * @param angle the angle by which the rectangle is rotated
	 **/ 
	void rotateAroundOrigin(double angle) {

		a.rotate(angle);
		b.rotate(angle);
		c.rotate(angle);
		d.rotate(angle);
	};

	double intersectArea(DkRectCorners *r);

	/**
	 * Converts the rectangle to a string.
	 * @return a string containing all corner coordinates.
	 **/
	std::string toString() {
		
		std::string msg = "a: " + a.toString() + " b: " + b.toString() +
			" c: " + c.toString() + " d: " + d.toString() + "\n";

		return msg;
	};

	double area() {
		
		return getDkRect().area();
	};

	/**
	 * Default Destructor.
	**/
	~DkRectCorners() {};

};

/**
 * An Interest Point.
 * This class DK_CORE_API stores interest points. An interest point is a pixel which
 * is a local maximum with respect to the method computed. In this case
 * interest points are extracted by means of the DoG.
 **/
class DK_CORE_API DkInterestPoint {

public:
	
	DkVector vec;	/**< the interest point's coordinates*/
	float val;		/**< the pixel value at the interest point's location*/
	int idx;		/**< the interest point's index (with respect to the distance map)*/

	/** 
	 * Default constructor.
	 **/
	DkInterestPoint() : vec(0, 0), val(-1), idx(-1) {};

	/**
	 * Creates an interest point.
	 * The coordinate origin is at the images top left corner.
	 * @param x the x coordinate
	 * @param y the y coordinate (it increases from top to bottom)
	 * @param val the interest point's value
	 **/
	DkInterestPoint(float x, float y, float val) : vec(0, 0), val(-1), idx(-1) {
		this->vec.x = x;
		this->vec.y = y;
		this->val = val;
	};

	/**
	 * Creates an interest point.
	 * The coordinate origin is at the images top left corner.
	 * @param x the x coordinate
	 * @param y the y coordinate (it increases from top to bottom)
	 * @param val the interest point's value
	 * @param idx the interest point's idx within the distance map
	 **/
	DkInterestPoint(float x, float y, float val, int idx) : vec(0, 0), val(-1), idx(-1) {
		this->vec.x = x;
		this->vec.y = y;
		this->val = val;
		this->idx = idx;
	};

	/**
	 * Default destructor.
	 **/
	~DkInterestPoint() {};
	
	/**
	 * Checks if two interest points have the same values.
	 * @return true if both interest points have the same values (except for their index)
	 **/
	bool operator== (const DkInterestPoint &o) {
		return (this->vec == o.vec && this->val == o.val);
	};

	/**
	 * Checks if two interest points have different values.
	 * @return true if both interest points have different values (except for their index)
	 **/
	bool operator!= (const DkInterestPoint &o) {
		return (this->vec != o.vec || this->val != o.val);
	};

	/**
	 * Compares the value of two interest points.
	 * This method is needed in order to sort a std::vector< of interest points.
	 * @param o an other interest point to compare the current with.
	 * @return true if the obtained interest point (o) has a greater value (val) than the current.
	 **/
	bool operator< (const DkInterestPoint &o) const {
		return (this->val < o.val);
	};

	/**
	 * Checks if an interest point ip lies within a square around the current interest point.
	 * @param ip the interest point compared.
	 * @param a the length of each side of the square
	 * @return true if the current interest point lies with the square
	 **/
	bool withinSquare(DkInterestPoint *ip, float a) {
		float ha = a*0.5f;

		return (ip->vec.x > this->vec.x-ha &&
				ip->vec.x < this->vec.x+ha &&
				ip->vec.y > this->vec.y-ha &&
				ip->vec.y < this->vec.y+ha);
	};

	/** 
	 * Converts the instance to a string.
	 * @return a string containing all values of the given instance.
	 **/
	std::string toString();

};

/**
 * A local descriptor.
 * This class DK_CORE_API stores interest points. An interest point is a pixel which
 * is a local maximum with respect to the method computed. In this case
 * interest points are extracted by means of the DoG. In addition to the
 * interest point, the local descriptor (feature vector) is stored which
 * is associated with the interest point.
 **/
class DK_CORE_API DkDescriptor {

public:
	
	DkVector pos;	/**< the interest point's coordinates*/
	float scale;	/**< the interest point's scale*/
	float val;		/**< the pixel value at the interest point's location*/
	float angle;	/**< the descriptor's angle*/
	int label;		/**< value of the label image*/
	Mat feature;	/**< the local descriptor*/
	int fontType;

	/** 
	 * Default constructor.
	 **/
	DkDescriptor() : pos(), scale(1), val(-1), fontType(-1) {};

	/**
	 * Creates an local descriptor.
	 * The coordinate origin is at the images top left corner.
	 * @param x the x coordinate
	 * @param y the y coordinate (it increases from top to bottom)
	 * @param val the interest point's value
	 **/
	DkDescriptor(float x, float y, float val) : pos(), scale(1), val(-1), fontType(-1) {
		this->pos.x = x;
		this->pos.y = y;
		this->val = val;
	};

	/**
	 * Creates an interest point.
	 * The coordinate origin is at the images top left corner.
	 * @param x the x coordinate
	 * @param y the y coordinate (it increases from top to bottom)
	 * @param scale the interest point's scale
	 * @param val the interest point's value
	 **/
	DkDescriptor(float x, float y, float scale, float val) : pos(), scale(1), val(-1), fontType(-1) {
		this->pos.x = x;
		this->pos.y = y;
		this->scale = scale;
		this->val = val;
	};

	/**
	 * Creates an interest point.
	 * The coordinate origin is at the images top left corner.
	 * @param pos the interest point's coordinates
	 * @param scale the interest point's scale
	 * @param val the interest point's value
	 **/
	DkDescriptor(DkVector pos, float scale, float val) : pos(), scale(1), val(-1), fontType(-1) {
		this->pos = pos;
		this->scale = scale;
		this->val = val;
	};


	/**
	 * Default destructor.
	 **/
	~DkDescriptor() {};

	/**
	 * Checks if two interest points have the same values.
	 * @return true if both interest points have the same values (except for their index)
	 **/
	bool operator== (const DkDescriptor &o) {
		return (this->pos == o.pos && this->val == o.val) ? true : false;
	};

	/**
	 * Compares the value of two interest points.
	 * This method is needed in order to sort a std::vector< of interest points.
	 * @param o an other interest point to compare the current with.
	 * @return true if the obtained interest point (o) has a greater value (val) than the current.
	 **/
	bool operator< (const DkDescriptor &o) {
		
		return (this->val < o.val);
	};

	DkBox getBBox() const {

		return DkBox(pos-getRadius(), DkVector(2.0f*getRadius(), 2.0f*getRadius()));
	}

	float getRadius() const {

		return 6*scale;
	};

	void setRadius(float radius) {

		scale = radius/6;
	};

	/** 
	 * Converts the instance to a string.
	 * @return a string containing all values of the given instance.
	 **/
	std::string toString();

};

/**
 * A simple point class DK_CORE_API.
 * This class DK_CORE_API is needed for a fast computation
 * of the polygon overlap.
 **/
class DK_CORE_API DkIPoint {
	
public:
	int x;
	int y;

	DkIPoint() : x(0), y(0) {};

	DkIPoint(int x, int y) {
		this->x = x;
		this->y = y;
	};
};


/**
 * A simple vertex class DK_CORE_API.
 * This class DK_CORE_API is needed for a fast computation
 * of the polygon overlap.
 **/
class DK_CORE_API DkVertex {

public:
	DkIPoint ip;
	DkIPoint rx;
	DkIPoint ry;
	int in;

	DkVertex() {};

	DkVertex (DkIPoint ip, DkIPoint rx, DkIPoint ry) {
		this->ip = ip;
		this->rx = rx;
		this->ry = ry;
		in = 0;
	};
};

class DK_CORE_API DkIntersectPoly {

// this class DK_CORE_API is based on a method proposed by norman hardy
// see: http://www.cap-lore.com/MathPhys/IP/aip.c

public:

	DkIntersectPoly() {};

	DkIntersectPoly(std::vector<DkVector> vecA, std::vector<DkVector> vecB) {

		this->vecA = vecA;
		this->vecB = vecB;
		interArea = 0;
	};

	double compute() {

		// defines
		gamut = 500000000;
		minRange = DkVector(FLT_MAX, FLT_MAX);
		maxRange = DkVector(-FLT_MAX, -FLT_MAX);
		computeBoundingBox(vecA, &minRange, &maxRange);
		computeBoundingBox(vecB, &minRange, &maxRange);

		scale = maxRange - minRange;

		if (scale.minCoord() == 0) return 0; //rechteck mit höhe oder breite = 0

		scale.x = gamut/scale.x;
		scale.y = gamut/scale.y;

		float ascale = scale.x * scale.y;

		// check input
		if (vecA.size() < 3 || vecB.size() < 3) {
			wout << "The polygons must consist of at least 3 points but they are: (vecA: " << vecA.size() << ", vecB: " << vecB.size() << dkendl;
			return 0;
		}

		// compute edges
		std::vector<DkVertex> ipA;
		std::vector<DkVertex> ipB;
		
		getVertices(vecA, &ipA, 0);
		getVertices(vecB, &ipB, 2);

		for (unsigned int idxA = 0; idxA < ipA.size()-1; idxA++) {
			for (unsigned int idxB = 0; idxB < ipB.size()-1; idxB++) {

				if (ovl(ipA[idxA].rx, ipB[idxB].rx) && ovl(ipA[idxA].ry, ipB[idxB].ry)) {

					int64 a1 = -area(ipA[idxA].ip, ipB[idxB].ip, ipB[idxB+1].ip);
					int64 a2 = area(ipA[idxA+1].ip, ipB[idxB].ip, ipB[idxB+1].ip);

					if (a1 < 0 == a2 < 0) {
						int64 a3 = area(ipB[idxB].ip, ipA[idxA].ip, ipA[idxA+1].ip);
						int64 a4 = -area(ipB[idxB+1].ip, ipA[idxA].ip, ipA[idxA+1].ip);

						if (a3 < 0 == a4 < 0) {

							if (a1 < 0) {
								cross(ipA[idxA], ipA[idxA+1], ipB[idxB], ipB[idxB+1], (double)a1, (double)a2, (double)a3, (double)a4);
								ipA[idxA].in++;
								ipB[idxB].in--;
							}
							else {
								cross(ipB[idxB], ipB[idxB+1], ipA[idxA], ipA[idxA+1], (double)a3, (double)a4, (double)a1, (double)a2);
								ipA[idxA].in--;
								ipB[idxB].in++;
							}
						}
					}

				}
			}
		}

		inness(ipA, ipB);
		inness(ipB, ipA);

		double areaD = (double)interArea / (ascale+FLT_MIN);

		return areaD;

	};

private: 

	std::vector<DkVector> vecA;
	std::vector<DkVector> vecB;
	int64 interArea;
	DkVector maxRange;
	DkVector minRange;
	DkVector scale;
	float gamut;


	void inness(std::vector<DkVertex> ipA, std::vector<DkVertex> ipB) {

		int s = 0;
		
		DkIPoint p = ipA[0].ip;

		for (int idx = (int)ipB.size()-2; idx >= 0; idx--) {

			if (ipB[idx].rx.x < p.x && p.x < ipB[idx].rx.y) {
				bool sgn = 0 < area(p, ipB[idx].ip, ipB[idx+1].ip);
				s += (sgn != ipB[idx].ip.x < ipB[idx+1].ip.x) ? 0 : (sgn ? -1 : 1);
			}
		}

		for (unsigned int idx = 0; idx < ipA.size()-1; idx++) {
			if (s != 0)
				cntrib(ipA[idx].ip.x, ipA[idx].ip.y, ipA[idx+1].ip.x, ipA[idx+1].ip.y, s);
			s += ipA[idx].in;
		}

	};

	void cross(DkVertex a, DkVertex b, DkVertex c, DkVertex d, double a1, double a2, double a3, double a4) {

		double r1 = a1 / ((double)a1+a2 + DBL_EPSILON);
		double r2 = a3 / ((double)a3+a4 + DBL_EPSILON);

		cntrib(	(int)a.ip.x + cvRound((double)(r1 * (double)(b.ip.x - a.ip.x))),
				(int)a.ip.y + cvRound((double)(r1 * (double)(b.ip.y - a.ip.y))),
				b.ip.x, b.ip.y, 1);
		cntrib(	d.ip.x, d.ip.y,
				(int)c.ip.x + cvRound((double)(r2 * (double)(d.ip.x - c.ip.x))),
				(int)c.ip.y + cvRound((double)(r2 * (double)(d.ip.y - c.ip.y))),
				1);
	};

	void cntrib(int fx, int fy, int tx, int ty, int w) {

		interArea += (int64)w * (tx - fx) * (ty + fy)/2;
	};


	int64 area(DkIPoint a, DkIPoint p, DkIPoint q) {
 
		return (int64)p.x * q.y - (int64)p.y * q.x +
			(int64)a.x * (p.y - q.y) + (int64)a.y * (q.x - p.x);
	};

	bool ovl(DkIPoint p, DkIPoint q) {

		return (p.x < q.y && q.x < p.y);
	};

	void getVertices(const std::vector<DkVector> vec, std::vector<DkVertex> *ip, int noise) {

		std::vector<DkIPoint> vecTmp;

		// transform the coordinates and modify the least significant bits (that's fun)
		for (int idx = 0; idx < (int)vec.size(); idx++) {

			DkIPoint cp;
			cp.x = ((int)((vec[idx].x - minRange.x) * scale.x - gamut/2) & ~7) | noise | (idx & 1);
			cp.y = ((int)((vec[idx].y - minRange.y) * scale.y - gamut/2) & ~7) | noise | (idx & 1);

			vecTmp.push_back(cp);
		}

		vecTmp.push_back( *(vecTmp.begin()));	// append the first element

		for (int idx = 0; idx < (int)vecTmp.size(); idx++) {

			int nIdx = idx % (int)(vecTmp.size()-1) + 1;	// the last element should refer to the second (first & last are the very same)

			DkIPoint cEdgeX = (vecTmp[idx].x < vecTmp[nIdx].x) ? 
				DkIPoint(vecTmp[idx].x, vecTmp[nIdx].x) : 
				DkIPoint(vecTmp[nIdx].x, vecTmp[idx].x);
			DkIPoint cEdgeY = (vecTmp[idx].y < vecTmp[nIdx].y) ? 
				DkIPoint(vecTmp[idx].y, vecTmp[nIdx].y) : 
				DkIPoint(vecTmp[nIdx].y, vecTmp[idx].y);

				ip->push_back(DkVertex(vecTmp[idx], cEdgeX, cEdgeY));
		}
	};

	void computeBoundingBox(std::vector<DkVector> vec, DkVector *minRange, DkVector *maxRange) {


		for (unsigned int idx = 0; idx < vec.size(); idx++) {

			*minRange = minRange->getMinVec(vec[idx]);
			*maxRange = maxRange->getMaxVec(vec[idx]);	// in our case it's the max vector
		}
	};

};

