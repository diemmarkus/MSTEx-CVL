/*******************************************************************************************************
 DkBaseMS.cpp
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

#include "DkBaseMS.h"

#include "DkSegmentation.h"
#include "DkMSModule.h"
#include "DkRgbModule.h"

#include <iostream>
#include <fstream>

#include "DkCentralWidget.h"


static int filecount = 1;

DkBaseMS::DkBaseMS() {
	configPath = "";
}

void DkBaseMS::init() {
	//DkBase::compMode = DK_FULL_COMPUTATION;

	// load default values
	DkBase::init();

	if (configPath != "" && QFileInfo(configPath.c_str()).exists()) {
		mout << "using " << configPath << " as config file" << dkendl;
		readConfigFile(configPath);
	} else {
		
		if (QFileInfo("config.conf").exists())
			readConfigFile("config.conf");	// to distinguish between external config and debug config
		else
			readConfigFile("..\\DkNomacs\\config.conf");
	}

	return;
}

DkBaseComputeFunction DkBaseMS::resolveFunction(int mode) {

	DkBaseComputeFunction fnc = 0;

	switch (mode) {
		case DK_EVALUATION:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseMS::showImages);			break;
		case DK_RGB_MODE:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseMS::showImages);			break;
		//case DK_SAMPLE:		callFunction(currentFile, img, maskFile, mask);	break;

		// if I don't know it - send it to the parent
		default:			fnc = DkBase::resolveFunction(mode);		break;
	}

	return fnc;
}

void DkBaseMS::showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	//printf("OpenCV: %s", cv::getBuildInformation().c_str());

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	if (mask.empty()) {
		mask = Mat(img.size(), CV_8UC1);
		mask = 255;
		iout << "empty mask" << dkendl;
	}

	std::string fname = "binar_r_" + imgFile->file().absoluteDir().dirName().toStdString() + ".png";

	DkSegmentationModule* module = 0;

	if (mode == DK_RGB_MODE) {
		module = new DkRgbModule(imgFile->file().absolutePath().toStdWString(), imgFile->file().fileName().toStdWString());
		fname = "binar_r_" + imgFile->file().baseName().toStdString() + ".png";
	}
	else
		module = new DkMSModule(imgFile->file().absoluteDir().absolutePath().toStdWString());

	if (!module) {
		wout << "We have some serious issues here - the module is NULL!!" << dkendl;
		return;
	}

	module->load();
	module->compute();

	Mat segImg = module->getSegImg();
	Mat mixedImg;

	// mix with GT --------------------------------------------------------------------
	if (!module->getGT().empty() && !segImg.empty()) {
		mixedImg = segImg;
		mixedImg.convertTo(mixedImg, CV_32F, 0.2f/255.0f);

		Mat gtImgF = module->getGT();
		gtImgF.convertTo(gtImgF, CV_32F, 1.0f/255.0f);
		DkIP::invertImg(gtImgF);
		gtImgF *= 0.4f;

		mixedImg += gtImgF;
		mixedImg.convertTo(mixedImg, CV_8UC1, 255.0f);
	}
	// mix with GT --------------------------------------------------------------------

	DkMSData m = module->getMSImages();
	cv::Mat cImg = m.removeBackground(m.getVisChannel(), m.getBgChannel());

	if (DkIP::imwrite(DkBase::debugPath + fname, segImg))
		mout << fname << " written..." << dkendl;
	if (!mixedImg.empty() && DkIP::imwrite(DkBase::debugPath + DkUtils::createFileName(fname, "-res"), mixedImg))
		mout << DkUtils::createFileName(fname, "-res") << " written..."  << dkendl;

	if (DkIP::imwrite(DkBase::debugPath + DkUtils::createFileName(fname, "-gray"), module->getPredictedImage(), true))
		mout << DkUtils::createFileName(fname, "-gray") << " written..."  << dkendl;

	if (show) {

		cv::Mat vImg = module->getMSImages().getVisChannel();
		cv::Mat vImgL = module->getMSImages().getBgChannel();

		plot(module->getPredictedImage(), segImg, true);
		//plot(img, module.getPredictedImage());
	}

}

void DkBaseMS::saveDebug(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

}

void DkBaseMS::mouseClicked(QMouseEvent* event, QPoint imgPos) {

	if (event->modifiers() == Qt::ControlModifier)
		pts.push_back(imgPos);

}

void DkBaseMS::keyPressEvent(QKeyEvent* event) {

	DkBase::keyPressEvent(event);
}

void DkBaseMS::folderFinished() {

	mout << "Finished folder -----------------" << dkendl;

}

void DkBaseMS::postLoad() {

	if (!DkBase::show && trayIcon) {

		QString msg = "I have finished in:" + QString::fromStdString(totalTime.getTotal());
		trayIcon->showMessage(msg, "Snippet processing u", QSystemTrayIcon::Critical, 10000);
	}


	DkBase::postLoad();
}

void DkBaseMS::preLoad() {

	mout << DkHandleLog::getHandleReport() << dkendl;

	DkBase::preLoad();
}
