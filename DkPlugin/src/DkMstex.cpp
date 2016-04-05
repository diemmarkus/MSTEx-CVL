/*******************************************************************************************************
 DkMstexPlugin.cpp

 nomacs is a fast and small image viewer with the capability of synchronizing multiple instances

 Copyright (C) 2015 Markus Diem

 This file is part of nomacs.

 nomacs is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 nomacs is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *******************************************************************************************************/

#include "DkMstex.h"

// nomacs includes
#include "DkImageStorage.h"

// our things
#include "DkRgbModule.h"
#include "DkMSModule.h"

#pragma warning(push, 0)	// no warnings from includes - begin
#include <QAction>
#include <QDebug>
#include <QMessageBox>
#include <QPluginLoader>

#include "opencv2\core\core.hpp"
#pragma warning(pop)		// no warnings from includes - end

namespace nmp {

/**
*	Constructor
**/
DkMstexPlugin::DkMstexPlugin(QObject* parent) : QObject(parent) {

	// create run IDs
	QVector<QString> runIds;
	runIds.resize(id_end);

	runIds[id_binarize_msi] = "649a6bdf0c394d3fb8174274c9e203eb";
	runIds[id_binarize_rgb] = "38f2539055634457a6ec6500f7dc0454";
	runIds[id_visualize_msi] = "26b3018a8c2742e0abca533d6a823e21";
	mRunIDs = runIds.toList();

	// create menu actions
	QVector<QString> menuNames;
	menuNames.resize(id_end);
		
	menuNames[id_binarize_msi] = tr("Binarize MSI Image");
	menuNames[id_binarize_rgb] = tr("Binarize RGB Image");
	menuNames[id_visualize_msi] = tr("Visualize MSI");
	mMenuNames = menuNames.toList();

	// create menu status tips
	QVector<QString> statusTips;
	statusTips.resize(id_end);

	statusTips[id_binarize_msi] = tr("Binarizes MSI data, the folder must contain 8 images named F1.png - F8.png.");
	statusTips[id_binarize_rgb] = tr("Binarizes a RGB image.");
	statusTips[id_visualize_msi] = tr("Visualizes the ACE.");
	mMenuStatusTips = statusTips.toList();
}

/**
*	Destructor
**/
DkMstexPlugin::~DkMstexPlugin() {
}


/**
* Returns unique ID for the generated dll
**/
QString DkMstexPlugin::id() const {

	return PLUGIN_ID;
};


/**
* Returns descriptive iamge for every ID
* @param plugin ID
**/
QImage DkMstexPlugin::image() const {

	return QImage(":/Mstex/img/mstex.png");
};

/**
* Returns plugin version for every ID
* @param plugin ID
**/
QString DkMstexPlugin::version() const {

	return PLUGIN_VERSION;
};

QList<QAction*> DkMstexPlugin::createActions(QWidget* parent) {

	if (mActions.empty()) {
		QAction* ca = new QAction(mMenuNames[id_binarize_msi], this);
		ca->setObjectName(mMenuNames[id_binarize_msi]);
		ca->setStatusTip(mMenuStatusTips[id_binarize_msi]);
		ca->setData(mRunIDs[id_binarize_msi]);	// runID needed for calling function runPlugin()
		mActions.append(ca);

		ca = new QAction(mMenuNames[id_binarize_rgb], this);
		ca->setObjectName(mMenuNames[id_binarize_rgb]);
		ca->setStatusTip(mMenuStatusTips[id_binarize_rgb]);
		ca->setData(mRunIDs[id_binarize_rgb]);	// runID needed for calling function runPlugin()
		mActions.append(ca);

		ca = new QAction(mMenuNames[id_visualize_msi], this);
		ca->setObjectName(mMenuNames[id_visualize_msi]);
		ca->setStatusTip(mMenuStatusTips[id_visualize_msi]);
		ca->setData(mRunIDs[id_visualize_msi]);	// runID needed for calling function runPlugin()
		mActions.append(ca);
	}

	return mActions;
}

QList<QAction*> DkMstexPlugin::pluginActions() const {

	return mActions;
}

/**
* Main function: runs plugin based on its ID
* @param plugin ID
* @param image to be processed
**/
QSharedPointer<nmc::DkImageContainer> DkMstexPlugin::runPlugin(const QString &runID, QSharedPointer<nmc::DkImageContainer> imgC) const {

	if (!mRunIDs.contains(runID) || !imgC)
		return imgC;

	cv::Mat img = nmc::DkImage::qImage2Mat(imgC->image());

	// binarize msi
	if(runID == mRunIDs[id_binarize_msi]) {

		QImage rImg = applyMSImaging(imgC, true);

		if (!rImg.isNull())
			imgC->setImage(rImg, tr("MSI Binarization"));
	}
	if(runID == mRunIDs[id_binarize_rgb]) {

		std::wstring fPathW = std::wstring((const wchar_t *)imgC->filePath().utf16());
		
		DkRgbModule rgbModule(fPathW);
		rgbModule.load(img);
		rgbModule.compute();

		cv::Mat bwImg = rgbModule.getSegImg();
		cv::cvtColor(bwImg, bwImg, CV_GRAY2RGB);

		QImage bwImgQt = nmc::DkImage::mat2QImage(bwImg);
		imgC->setImage(bwImgQt, tr("MSI Binarization"));
	}
	if(runID == mRunIDs[id_visualize_msi]) {
		QImage rImg = applyMSImaging(imgC, false);

		if (!rImg.isNull())
			imgC->setImage(rImg, tr("MSI Visualization"));

	}

	// wrong runID? - do nothing
	return imgC;
}

// code from: http://stackoverflow.com/questions/5625884/conversion-of-stdwstring-to-qstring-throws-linker-error
std::wstring DkMstexPlugin::qStringToStdWString(const QString &str) const {
#ifdef _MSC_VER
	return std::wstring((const wchar_t *)str.utf16());
#else
	return str.toStdWString();
#endif
}

// code from: http://stackoverflow.com/questions/5625884/conversion-of-stdwstring-to-qstring-throws-linker-error
QString DkMstexPlugin::stdWStringToQString(const std::wstring &str) const {
#ifdef _MSC_VER
	return QString::fromUtf16((const ushort *)str.c_str());
#else
	return QString::fromStdWString(str);
#endif
}

QImage DkMstexPlugin::applyMSImaging(const QSharedPointer<nmc::DkImageContainer> imgC, bool binarize) const {
	
	QString dirPath = imgC->fileInfo().absolutePath();

	std::wstring dirPathW = qStringToStdWString(dirPath);

	DkMSModule msModule(dirPathW);
	std::vector<std::wstring> files = msModule.indexFolder(dirPathW);
	std::vector<cv::Mat> images;

	if (files.size() > 15) {
		QString msg = tr("Sorry, I cannot binarize %1 since it does not seem to be a Multi-Spectral image. ").arg(dirPath);
		msg += "Note, that 8 images are expected in the folder, where each corresponds to an MS channel.";
		QMessageBox::critical(QApplication::activeWindow(), tr("MSTEx Error"), msg);
		return QImage();
	}


	for (const std::wstring& fileNameW : files) {

		QString fileName = stdWStringToQString(fileNameW);

		QImage img(QFileInfo(dirPath, fileName).absoluteFilePath());
		images.push_back(nmc::DkImage::qImage2Mat(img));
	}

	msModule.setImages(files, images);
	msModule.setPseudoColor(!binarize);
	msModule.compute();

	QImage imgQt;

	if (binarize) {
		cv::Mat bwImg = msModule.getSegImg();
		cv::cvtColor(bwImg, bwImg, CV_GRAY2RGB);

		imgQt = nmc::DkImage::mat2QImage(bwImg);
	}
	else {

		cv::Mat colImg = msModule.getColImg();
		imgQt = nmc::DkImage::mat2QImage(colImg);
	}

	return imgQt;

}


};

