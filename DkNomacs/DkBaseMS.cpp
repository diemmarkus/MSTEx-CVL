/***************************************************
 *   DkBaseStefan.cpp
 *   
 *   Created on: 08.11.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *				 Stefan Fiel
 *      Company: Vienna University of Technology
 ***************************************************/

#include "DkBaseMS.h"
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
		//case DK_SAMPLE:		callFunction(currentFile, img, maskFile, mask);	break;

		// if I don't know it - send it to the parent
		default:			fnc = DkBase::resolveFunction(mode);		break;
	}

	return fnc;
}

void DkBaseMS::showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
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
