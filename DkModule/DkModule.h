/***************************************************
 *   DkModule.h
 *   
 *   Created on: 05.07.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/

#pragma once

#include "DkModuleInclude.h"
#include "DkImageSource.h"
#include "DkUtils.h"

enum releaseImgs{DK_KEEP_IMGS = 0, DK_RELEASE_IMGS, DK_SAVE_IMGS};

using namespace cv;


/**
 * This is the base class for all modules.
 * It provides all functions which are implemented
 * by the modules.
 **/
class DK_MODULE_API DkModule {

public:
	
	/**
	 * Default constructor.
	 **/
	DkModule();

	/**
	 * Provides the source images to the class.
	 * @param imgs the pre-processed source images.
	 **/
	DkModule(DkImageSource *imgs);

	/**
	 * Default destructor.
	 **/
	virtual ~DkModule() {};

	friend std::ostream& operator<<(std::ostream& s, DkModule& m) {

		// this makes the operator<< virtual (stroustrup)
		return s << m.toString();
	};

	/**
	 * Runs Image Processing routines
	 * @return void
	 **/ 
	virtual void compute() = 0;

	/**
	 * Converts the module's parameters and results to a string.
	 * @return a string containing all parameters and results of the module.
	 **/
	virtual std::string toString() const = 0;
	
	/**
	 * Changes the release behavior.
	 * @param releaseDebug: if releaseDebug == 1, all debug images are releases
	 * as soon as possible. Functions such as getDebugSomething() must not be
	 * called. If releaseDebug == 0, all images computed are kept in the memory
	 * until the module's destructor or release() is called.
	 **/
	virtual void setReleaseDebug(int releaseDebug) {

		if (releaseDebug < 0 || releaseDebug > 2)
			wout << "[" << className << "] releaseDebug should be within [0 2] but it is: " << releaseDebug << dkendl;

		this->releaseDebug = releaseDebug;
	};

	virtual void setDebugLevel(int debugLevel) {
		userDebugLevel = debugLevel;

		wout.setUserDebug(debugLevel);
		mout.setUserDebug(debugLevel);
		iout.setUserDebug(debugLevel);
		dout.setUserDebug(debugLevel);
	};

	/**
	 * Returns the module's name.
	 * @return the module's name.
	 **/
	virtual std::string getName() {
		return className;
	};

	/**
	 * Writes the debug images to the hard disk.
	 * The images are stored in "path" with the assigned filename
	 * and _CLASSNAME_ATTRIBUTE.
	 * @param path the desired path name.
	 * @param filename the desired filename & format.
	 * @param img an image.
	 **/
	virtual void saveDebugImg(std::string path, std::string filename, Mat img) {
		wout << "[" << className << "] no images to save" << dkendl;
	};

protected:
	int releaseDebug;					/**< if 1, all debug images are released as soon as possible.**/
	int userDebugLevel;					/**< user debug level **/

	DkImageSource *imgs;				/**< the pre-processed source images.**/
	std::string className;				/**< the module's name.**/

	// debug outputs
	DkDebugStream wout;					/**< warning output**/
	DkDebugStream mout;					/**< warning output**/
	DkDebugStream iout;					/**< warning output**/
	DkDebugStream dout;					/**< warning output**/

	virtual void checkInput() const = 0;		/**< checks if all input images are in the specified format.**/


};


/**
 * 
 **/ 
class DK_MODULE_API DkInterface : public DkModule {

public:
	// for now: pass it on
	DkInterface() : DkModule() {};
	DkInterface(DkImageSource* imgs) : DkModule(imgs) {};
	virtual ~DkInterface() {};

	/**
	 * The computation stuff is done here.
	 * This method must be called implicitly by all getters.
	 * However, it is public so that the user may call it
	 * explicitly at a specific time.
	 **/ 
	virtual void compute() = 0;

	virtual Mat draw(Mat& img) = 0;


};
