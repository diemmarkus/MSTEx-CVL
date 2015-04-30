/*******************************************************************************************************
 DkUtils.h
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

#ifdef DK_DEBUG
#define dkendl std::endl
#else
// removing std::endl allows for removing
// the debug lines
#define dkendl "";
#endif

//#include "DkCoreIncludes.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <time.h>

#include <sstream>
#include <stdarg.h>

// for svm params
#include <iostream>
#include <fstream>

#include "DkError.h"

#ifdef _WIN32
	#include <wtypes.h>

#endif

#ifdef linux
	typedef unsigned long DWORD;
#endif

enum morphTypes {DK_ERODE=0, DK_DILATE};
enum DebugLevel {DK_DEFAULT_OUTPUT=-1, DK_NONE, DK_WARNING, DK_MODULE, DK_INFO, DK_DEBUG_INFO, DK_DEBUG_C, DK_DEBUG_ALL};
enum SpeedLebel {DK_NO_SPEED_UP=0, DK_SPEED_UP, DK_APPROXIMATE};

using namespace cv;

/**
* This class DK_CORE_API contains general functions which are useful.
**/
class  DkUtils {

private:
	DK_CORE_API static int debugLevel;

public:

	DK_CORE_API static cv::Scalar blue;
	DK_CORE_API static cv::Scalar blueDark;
	DK_CORE_API static cv::Scalar yellow;

	/**
	* Prints a matrix to the standard output.
	* This is especially useful for copy and pasting e.g.
	* histograms to matlab and visualizing them there.
	* @param src an image CV_8UC1 or CV_32FC1 or CV_64FC1.
	* @param varName the variable name for Matlab.
	**/
	static void printMat(const Mat src, std::string varName) {
		if (src.depth() == CV_32FC1)
			printMat<float>(src, varName);
		else if (src.depth() == CV_64FC1)
			printMat<double>(src, varName);
		else if (src.depth() == CV_8UC1) {
			Mat tmp;
			src.convertTo(tmp,CV_32FC1);
			printMat<float>(tmp, varName);
		}
		else
			DkUtils::printDebug(DK_WARNING, "I could not visualize the mat: %s\n", varName.c_str());

		
	}

	/**
	* Prints a matrix to the standard output.
	* This is especially useful for copy and pasting e.g.
	* histograms to matlab and visualizing them there.
	* @param src an image CV_32FC1.
	* @param varName the variable name for Matlab.
	**/
	template <typename numFmt>
	static void printMat(const Mat src, std::string varName) {

		printf("%s = %s\n", varName.c_str(), printMat<numFmt>(src).c_str());
	}

	/**
	* Prints a matrix to the standard output.
	* This is especially useful for copy and pasting e.g.
	* histograms to matlab and visualizing them there.
	* @param src an image CV_32FC1.
	* @param varName the variable name for Matlab.
	**/
	static std::string printMat(const Mat src) {
				
		return printMat<float>(src);
	}

	static std::string keyPointToString(const KeyPoint& kp) {

		std::string str;
		str += "<" + DkUtils::stringify(kp.pt.x, 1) + ", " + DkUtils::stringify(kp.pt.y, 1) + "> "; 
		str += "size|octave: " + DkUtils::stringify(kp.size) + " | " + DkUtils::stringify(kp.octave); 
		str += " angle: " + DkUtils::stringify(kp.angle, 1);

		return str;
	}

	/**
	* Prints a matrix to the standard output.
	* This is especially useful for copy and pasting e.g.
	* histograms to matlab and visualizing them there.
	* @param src an image CV_32FC1.
	* @param varName the variable name for Matlab.
	**/
	template <typename numFmt>
	static std::string printMat(const Mat src) {

		std::string msg = " [";	// matlab...

		int cnt = 0;

		for (int rIdx = 0; rIdx < src.rows; rIdx++) {

			const numFmt* srcPtr = src.ptr<numFmt>(rIdx);

			for (int cIdx = 0; cIdx < src.cols; cIdx++, cnt++) {


				msg += DkUtils::stringify(srcPtr[cIdx]);

				msg += (cIdx < src.cols-1) ? " " : "; "; // next row matlab?

				if (cnt % 7 == 0)
					msg += "...\n";
			}

		}
		msg += "];\n";

		return msg;
	}


	/**
	* Prints the Mat's attributes to the standard output.
	* The Mat's attributes are: size, depth, number of channels and
	* dynamic range.
	* @param img an image (if it has more than one channel, the dynamic range
	* is not displayed)
	* @param varname the name of the matrix
	**/
	static void getMatInfo(Mat img, std::string varname) {

		printf("%s: %s\n", varname.c_str(), getMatInfo(img).c_str());
	}

	/**
	* Converts the Mat's attributes to a string.
	* The Mat's attributes are: size, depth, number of channels and
	* dynamic range.
	* @param img an image (if it has more than one channel, the dynamic range
	* is not converted).
	* @return a string with the Mat's attributes.
	**/
	static std::string getMatInfo(Mat img) {

		std::string info = "\n\nimage info:\n";

		if (img.empty()) {
			info += "   <empty image>\n";
			return info;
		}

		info += "   " + DkUtils::stringify(img.rows) + " x " + DkUtils::stringify(img.cols) + " (rows x cols)\n";
		info += "   channels: " + DkUtils::stringify(img.channels()) + "\n";

		int depth = img.depth();
		info += "   depth: ";
		switch (depth) {
		case CV_8U:
			info += "CV_8U";
			break;
		case CV_32F:
			info += "CV_32F";
			break;
		case CV_32S:
			info += "CV_32S";
			break;
		case CV_64F:
			info += "CV_64F";
			break;
		default:
			info += "unknown";
			break;
		}

		if (img.channels() == 1) {
			info += "\n   dynamic range: ";

			double min, max;
			minMaxLoc(img, &min, &max);
			info += "[" + DkUtils::stringify(min) + " " + DkUtils::stringify(max) + "]\n";
		}
		else
			info += "\n";

		return info;

	}

	template <typename num>
	static void writeMat(std::string filePath, const Mat&img) {
		
		
		std::ofstream file;
		
#ifdef WIN32
		file.open(filePath, std::ios::out | std::ios::trunc);		// overwrite the file
#endif

#ifdef linux
		file.open(filePath.c_str(), std::ios::out | std::ios::trunc);		// overwrite the file
#endif

		file << img.rows << "\n";
		file << img.cols << "\n";

		for (int rIdx = 0; rIdx < img.rows; rIdx++) {
			const num* ptr = img.ptr<num>(rIdx);

			for (int cIdx = 0; cIdx < img.cols; cIdx++) {
				file << ptr[cIdx] << ", ";
			}

			file << "\n";
		}

		file.close();
	}

	static Mat loadMat(std::string filePath) {

		//std::replace(filePath.begin(), filePath.end(), '\\', '/');

		std::filebuf file;
		
#ifdef _WIN32
		file.open(filePath, std::ios::in);
#endif

#ifdef linux
		file.open(filePath.c_str(), std::ios::in);
#endif
		
		std::istream stream(&file);

		// this code is very dirty - if the file is not exactly what we expect, strange things might happen
		std::string line;
		std::getline(stream, line);

		int rows = atoi(line.c_str());
		std::getline(stream, line);
		int cols = atoi(line.c_str());
		
		Mat img = Mat(rows, cols, CV_32FC1);

		for (int rIdx = 0; rIdx < rows; rIdx++) {

			float* srcPtr = img.ptr<float>(rIdx);

			if (!getline(stream, line))
				throw DkIllegalArgumentException("wrong file format sorry: " + filePath, __LINE__, __FILE__);

			std::stringstream linestream(line);
			std::string value;

			for (int cIdx = 0; cIdx < cols; cIdx++) {

				if (!getline(linestream, value, ','))
					throw DkIllegalArgumentException("wrong column format sorry: " + filePath, __LINE__, __FILE__);
				
				srcPtr[cIdx] = (float)atof(value.c_str());
			}
		}

		file.close();

		return img;
	}

	static void histToText(std::string filePath, const Mat& img) {
#ifdef WIN32
		if (img.type() != CV_32FC1) {
			std::string msg = "The image should be CV_32FC1, but it is: " + getMatInfo(img);
			throw DkMatException(msg, __LINE__, __FILE__);
		}
		if (img.rows != 1) {

			throw DkMatException("Image must be 1xN\n", __LINE__, __FILE__);
		}

		try {
			// get the time
			struct tm newtime;
			__time32_t aclock;

			char buffer[32];
			_time32( &aclock );
			_localtime32_s( &newtime, &aclock );
			asctime_s(buffer, 32, &newtime);

			// write the rectangles to a file for matlab...
			std::ofstream rectFile;
			rectFile.open(DkUtils::stringToWchar(filePath), std::ios::out | std::ios::app);

			rectFile << "%% histogram - " << buffer << "\n";
			rectFile << "hist = [";

			const float* imgPtr = img.ptr<float>();

			for (int idx = 0; idx < img.cols; idx++) {

				rectFile << imgPtr[idx] << ", ";
			}
			rectFile << "];\n\n";

			// drawRectangles is in utilities...
			rectFile << "%plot the histogram\n";
			rectFile << "figure; bar(hist, 'FaceColor', [0.8 0.8 0.8]);\n";
			rectFile << "ylim([min(hist) max(hist)+0.1*max(hist)]);\n";

			//rectFile << "hold on;\n";

			//// print rectangles
			//rectFile << "p.lineColor = [255 198 0]/255;\n";
			//rectFile << "plotRectangle(rects(rects(:,6) == 1, 1), rects(rects(:,6) == 1, 2), rects(rects(:,6) == 1, 4), "
			//	<< "rects(rects(:,6) == 1, 3), rects(rects(:,6) == 1, 5), p);\n";

			//// manuscript rectangles
			//rectFile << "p.lineColor = [0 150 200]/255;\n";
			//rectFile << "plotRectangle(rects(rects(:,6) == 2, 1), rects(rects(:,6) == 2, 2), rects(rects(:,6) == 2, 4), "
			//	<< "rects(rects(:,6) == 2, 3), rects(rects(:,6) == 2, 5), p);\n";

			//// no text
			//rectFile << "p.lineColor = [200 200 200]/255;\n";
			//rectFile << "plotRectangle(rects(rects(:,6) == 0, 1), rects(rects(:,6) == 0, 2), rects(rects(:,6) == 0, 4), "
			//	<< "rects(rects(:,6) == 0, 3), rects(rects(:,6) == 0, 5), p);\n\n";

			rectFile.close();

		}
		catch(...) {
			printf("I could not write to %s\n sorry!\n", filePath.c_str());
		}
#endif	
	}

	/**
	* Appends an attribute name to the filename given.
	* generates: image0001.tif -> img0001_mask.tif
	* @param fName the filename with extension.
	* @param ext the new file extension if it is "" the old extension is used.
	* @param attribute the attribute which extends the filename.
	* @return the generated filename.
	**/
	static std::string createFileName(std::string fName, std::string attribute, std::string ext = "") {

		if (ext == "") ext = fName.substr(fName.length()-4, fName.length()); // use the old extension

		// generate: img0001.tif -> img0001_mask.tif
		return fName.substr(0, fName.length()-4) + attribute + ext;
	}

	static std::string removeExtension(std::string fName) {

		return fName.substr(0, fName.find_last_of("."));
	}

	static std::string wstringToString(const std::wstring& wstr) {

		return std::string(wstr.begin(), wstr.end());
	}

	static std::wstring stringToWstring(const std::string& str) {

		return std::wstring(str.begin(), str.end());
	}

	/**
	* Converts a number to a string.
	* @throws an exception if number is not a number.
	* @param number any number.
	* @return a string representing the number.
	**/
	template <typename numFmt>
	static std::string stringify(numFmt number) {

		std::stringstream stream;
		if (! (stream << number)) {
			std::string msg = "Sorry, I could not cast it to a string";
			throw DkCastException(msg, __LINE__, __FILE__);
		}

		return stream.str();
	}

	/**
	* Converts a number to a string.
	* @throws an exception if number is not a number.
	* @param number any number.
	* @param n the number of decimal places.
	* @return a string representing the number.
	**/
	template <typename numFmt>
	static std::string stringify(numFmt number, double n) {

		int rounded = cvRound(number * pow(10,n));

		return stringify(rounded/pow(10,n));
	};

#ifdef WIN32
	static LPCWSTR stringToWchar(std::string str) {
		wchar_t *wChar = new wchar_t[(int)str.length()+1];
		size_t convertedChars = 0;
		mbstowcs_s(&convertedChars, wChar, str.length()+1, str.c_str(), _TRUNCATE);
		//mbstowcs(wChar, str.c_str(), str.length()+1);

		return (LPCWSTR)wChar;
	};
#endif

#ifdef linux
	static const char* stringToWchar(std::string str) {
		return str.c_str();
	};
#endif

	static std::string stringTrim(const std::string str) {


		std::string strT = str;

		if (strT.length() <= 1) return strT;	// .empty() may result in errors

		// remove whitespace
		size_t b = strT.find_first_not_of(" ");
		size_t e = strT.find_last_not_of(" ");
		strT = strT.substr(b, e+1);

		if (strT.length() <= 1) return strT;	// nothing to trim left

		// remove tabs
		b = strT.find_first_not_of("\t");
		e = strT.find_last_not_of("\t");

		// fixes a bug if the string contains only "\t\t"
		if (b == e)
			return std::string();

		strT = strT.substr(b, e+1);

		return strT;
	};

	static std::string stringRemove(const std::string str, const std::string repStr) {

		std::string strR = str;

		if (strR.length() <= 1) return strR;

		size_t pos = 0;

		while ((pos = strR.find_first_of(repStr)) < strR.npos) {

			strR.erase(pos, repStr.length());
		}

		return strR;
	};

	/**
	* Sets the actual debug level.
	* @param l the debug level of the application.
	**/
	static void setDebug(int l) {
		debugLevel = l;
	};

	/**
	* Returns the current debug level.
	* @return the debug level of the application.
	**/
	static int getDebug() {
		return debugLevel;
	};

#ifdef DK_DEBUG

	/**
	* Prints a debug message according the message level and the current debug level defined in DkUtils.
	* The debug command prints only a message if DK_DEBUG is defined.
	* Debug levels are: DK_NONE=0,DK_WARNING, DK_MODULE, DK_DEBUG_A, DK_DEBUG_B, DK_DEBUG_C, DK_DEBUG_ALL.
	* @param level the debug level of the message.
	* @param fmt the format string of the message.
	**/
	static void printDebug(int level,const char *fmt,...) {
		va_list ap;


		if (debugLevel >= DK_WARNING && level == DK_WARNING)
			printf("WARNING: ");
		else if (debugLevel >= DK_MODULE && level == DK_MODULE)
			printf(">> ");

		va_start(ap,fmt);
		if ((fmt) && (level <= debugLevel)) {
			vprintf(fmt, ap);
			fflush(stdout);
		}
		va_end(ap);
	}
#else
	/**
	* If DK_DEBUG is undefined do nothing.
	**/
	inline static void printDebug(...) {};
#endif

};


#ifdef DK_DEBUG

/**
 * This class DK_CORE_API provides debug outputs for the cmd and files simultaneously.
 * It is derived from std::filebuf. It can output info to a file and/or
 * to the command line if specified. In addition it allows for different
 * debug output levels (see enum DebugLevel).
 **/
class DK_CORE_API DkDebugStreamBuffer : public std::filebuf {
	
	/**
 	 * This code is based on code from:
	 * http://www.horstmann.com/cpp/iostreams.html
	 **/ 

public:
	enum{DK_NONE, DK_STD_ONLY, DK_FILE_ONLY, DK_ALL};

	/**
	 * Constructs a debug stream buffer.
	 * @param debugLevel the stream's debug level (e.g. DK_WARNING)
	 **/ 
	DkDebugStreamBuffer(int debugLevel = DK_WARNING) { 
		std::filebuf::open("NUL", std::ios::out); 
		debugOutput = DK_ALL;			// TODO: control this from DkUtils??
		myDebugLevel = debugLevel;
		userDebugLevel = -1;
	};
	
	/**
	 * Changes the maximum debug level.
	 * The debug level provided is the maximum debug
	 * level that is printed for this stream.
	 * If it is not specified, the global debug level
	 * DkUtils::debugLevel() is used.
	 * @param userDebugLevel
	 **/ 
	void setUserDebug(int userDebugLevel) { 
		this->userDebugLevel = userDebugLevel; 
	};

	/**
	 * Returns the user debug level.
	 * The debug level provided is the maximum debug
	 * level that is printed for this stream.
	 * If it is not specified, the global debug level
	 * DkUtils::debugLevel() is used.
	 * @return int -1 if no user debug level was specified.
	 **/ 
	int getUserDebug() { 
		return userDebugLevel; 
	};

	/**
	 * Changes the stream's debug level.
	 * The level can either be: DK_WARNING, DK_MODULE, DK_INFO, DK_DEBUG_INFO.
	 * @param userDebugLevel see enum DebugLevel
	 **/ 
	void setDebugLevel(int debugLevel) {
		this->myDebugLevel = debugLevel;
	};

	/**
	 * Returns the stream's debug level.
	 * The level can either be: DK_WARNING, DK_MODULE, DK_INFO, DK_DEBUG_INFO.
	 * @return int the debug level
	 **/ 
	int getDebugLevel() {
		return myDebugLevel;
	};

	/**
	 * Opens the file specified.
	 * @param fname filename and path.
	 **/ 
	virtual void open(const char fname[]) {  

		close();
		
		if (fname != NULL && fname[0] != '\0')
			std::filebuf::open(fname, std::ios::app /*| std::ios::trunc*/);
		else
			std::filebuf::open("NUL", std::ios::out); 
	};

	/**
	 * Closes the file.
	 **/ 
	virtual void close() { 
		std::filebuf::close(); 
	};

	/**
	 * Overwrites the std::filebuf::sync() function.
	 * This function gets called by std::endl.
	 * @return int 0 if succeeded.
	 **/ 
	virtual int sync() {  

		//printf("my debug level: %i global debug: %i userDebug: %i -- ", myDebugLevel, DkUtils::getDebug(), userDebugLevel);

		if ((userDebugLevel == -1 && myDebugLevel > DkUtils::getDebug()) ||	// use global debug level if user level was not specified...
			(userDebugLevel != -1 && myDebugLevel > userDebugLevel)) {
				//fflush(0);		// clear buffer
				//printf("\n");
				setp(pbase(), pbase());		// clear buffer
				return 0;
		}

		if (debugOutput == DK_STD_ONLY || debugOutput == DK_ALL)
			std::cout.write(pbase(), pptr()-pbase());
		if (debugOutput == DK_STD_ONLY)
			setp(pbase(), pbase());		// clear buffer

		if (debugOutput == DK_FILE_ONLY || debugOutput == DK_ALL)
			return std::filebuf::sync(); 
		else
			return 0;
	};
	
protected:
	int debugOutput;
	int userDebugLevel;
	int myDebugLevel;
};

/**
 * This class DK_CORE_API implements the new debugging outputs.
 * It is based on std::ostream but uses the DkDebugStreamBuffer.
 * Hence, debugging levels are implemented and the output can be
 * written to a log file. If DK_DEBUG is not specified as pre-processor
 * argument, the debug code will be removed by the compiler.
 * NOTE: if std::endl is used instead of dkendl, the compiler cannot
 * remove the debugging code.
 **/ 
class DK_CORE_API DkDebugStream : public std::ostream {

public:

	static std::string defaultLogFile;	/**< the global log file.>*/

	/**
	 * This class DK_CORE_API provides extended debugging output.
	 * If the filename is not specified, the output is written
	 * to the global log file. If this is empty too, the output
	 * is solely written to the command line.
	 * @param filename the filepath of the logfile
	 **/ 
	DkDebugStream(std::string filename = "", int debugLevel = DK_DEBUG_INFO) : std::ostream(buffer = new DkDebugStreamBuffer(debugLevel)), std::ios(0) {
		fname = filename;

		if (fname.empty() && !defaultLogFile.empty())
			fname = defaultLogFile;
	};

	/**
	 * The copy constructor.
	 * In contrast to std::ostream we do want the object to be copy and assignable.
	 * In order to do so, we change the stream buffer (see stroustrup).
	 * NOTE: this could lead to troubles if one file is written by multiple DkDebugStream
	 * instances from different Threads.
	 **/ 
	DkDebugStream(const DkDebugStream &d) : std::ostream(buffer = new DkDebugStreamBuffer(DK_DEBUG_INFO)), std::ios(0) {

		fname = d.fname;

		if (d.buffer) {
			buffer->setDebugLevel(d.buffer->getDebugLevel());
			buffer->setUserDebug(d.buffer->getUserDebug());

			// is that nice? -- i mean, it's a constructor...
			if (d.buffer->is_open())
				open(fname);
		}			
	};

	
	/**
	 * Default destructor.
	 **/ 
	~DkDebugStream() { 
		release();
	};


	/**
	 * Releases the buffer.
	 **/ 
	virtual void release() {
		close();
		delete rdbuf();
	};
	
	/**
	 * Public assignment operator.
	 **/
	DkDebugStream& operator= (const DkDebugStream& d) {
		
		if (this == &d)
			return *this;

		this->fname = d.fname;
		
		if (d.buffer) {
			buffer->setDebugLevel(d.buffer->getDebugLevel());
			buffer->setUserDebug(d.buffer->getUserDebug());

			if (d.buffer->is_open())
				open(fname);
		}
		return *this;
	};

	/**
	 * Opens a new file.
	 * The file will not be created.
	 * @param fname filepath to the logfile.
	 **/ 
	void open(std::string fname = "") { 
		
		//if (fname.empty())
		//	return;

		buffer->open(fname.c_str()); 
		this->fname = fname;
	};

	/**
	 * Closes the file buffer.
	 **/ 
	void close() { 
		
		buffer->close(); 
	};

	/**
	 * Changes the user debug level.
	 * The maximum debug level that should be
	 * output, can be specified here. Otherwise,
	 * the global debug level will be used.
	 * @param userDebugLevel 
	 **/ 
	void setUserDebug(int userDebugLevel) { 
		
		if (buffer)
			buffer->setUserDebug(userDebugLevel); 
	};

	/**
	 * Returns the current user debug level.
	 * @return int -1 if no user debug level is specified.
	 **/ 
	int getUserDebug() { 
		return (buffer) ? buffer->getUserDebug() : 0; 
	};

protected:
	DkDebugStreamBuffer* buffer;
	std::string fname;
};

#else

/**
 * Dummy DebugStream class DK_CORE_API.
 * This class DK_CORE_API does nothing. Except for removing
 * all debug outputs. Note: the compiler cannot
 * optimize codes if << std::endl; is used.
 * (then, the << operator has sideffects to DkDebugStreamBuffer)
 * Solution: use dkendl which removes the std::endl if no
 * debug output is specified (DK_DEBUG not specified in the pre-processor)
 **/ 
class DK_CORE_API DkDebugStream : public std::ostream {

public:
	static std::string defaultLogFile;

	// dummy constructors
	DkDebugStream(std::string) : std::ostream(0), std::ios(0) {};
	DkDebugStream(std::string, int) : std::ostream(0), std::ios(0) {};
	DkDebugStream() : std::ostream(0), std::ios(0) {};

	DkDebugStream(const DkDebugStream&) : std::ostream(0), std::ios(0) {};

	virtual ~DkDebugStream() {}

	/**
	 * Default assignment operator.
	 **/
	DkDebugStream& operator= (const DkDebugStream&) {
		return *this;
	};

	// dummies
	void open(std::string fname = 0) {};

	void close() {};

	void setUserDebug(int) {};

	int getUserDebug() { return 0; };

	// tell the compiler that we don't do anything
	template<typename T>
	friend DkDebugStream& operator<< (DkDebugStream& s, T&) {
		return s;
	};

	//template<typename T>
	//DkDebugStream & operator<<(T&) {
	//	return *this;
	//}
};

#endif

// singleton pattern here
class DK_CORE_API DebugResources {

public:


		static DkDebugStream& wout() {
			return wout_p;
		}

		static DkDebugStream& mout() {
			return mout_p;
		}

		static DkDebugStream& iout() {
			return iout_p;
		}

		static DkDebugStream& dout() {
			return dout_p;
		}

protected:
	static DkDebugStream wout_p;
	static DkDebugStream mout_p;
	static DkDebugStream iout_p;
	static DkDebugStream dout_p;

};

static DkDebugStream& wout = DebugResources::wout();
static DkDebugStream& mout = DebugResources::mout();
static DkDebugStream& iout = DebugResources::iout();
static DkDebugStream& dout = DebugResources::dout();

static DkDebugStream& woutc = DebugResources::wout();
static DkDebugStream& moutc = DebugResources::mout();
static DkDebugStream& ioutc = DebugResources::iout();
static DkDebugStream& doutc = DebugResources::dout();


class DK_CORE_API DkHandleLog {

public:
	static std::string getHandleReport() {

#ifdef WIN32
		DWORD type_char = 0, 
			type_disk = 0, 
			type_pipe = 0, 
			type_remote = 0, 
			type_unknown = 0,
			handles_count = 0;

		GetProcessHandleCount(GetCurrentProcess(), &handles_count);
		handles_count *= 4;
		for (DWORD handle = 0x4; handle < handles_count; handle += 4) {
			switch (GetFileType((HANDLE)handle)){
			case FILE_TYPE_CHAR:
				type_char++;
				break;
			case FILE_TYPE_DISK:
				type_disk++;
				break;
			case FILE_TYPE_PIPE: 
				type_pipe++;
				break;
			case FILE_TYPE_REMOTE: 
				type_remote++;
				break;
			case FILE_TYPE_UNKNOWN:
				if (GetLastError() == NO_ERROR) type_unknown++;
				break;

			}
		}

		std::string str;
		str += "Handle report: ";
		str += DkUtils::stringify(type_char) + " char handles ";
		str += DkUtils::stringify(type_disk) + " disk handles ";
		str += DkUtils::stringify(type_pipe) + " pipe handles ";
		str += DkUtils::stringify(type_remote) + " remote handles ";
		str += DkUtils::stringify(type_unknown) + " unknown handles ";

		return str;
#endif
		return "";
	}

};


//#define wout DebugResources::wout()
//#define mout DebugResources::mout()
//#define iout DebugResources::iout()
//#define dout DebugResources::dout()
