/*******************************************************************************************************
 DkBase.h
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

//#include <opencv/highgui.h>	// TODO: delete this in the end!! -> qt
#include "DkSnippetNoMacsInclude.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
//#include <typeinfo>

#include <stdio.h>

#ifdef WIN32
	#include <WinSock2.h>
	#include <windows.h>
	#include <conio.h>
	#include <tchar.h>
	#include <commctrl.h>

#endif
#ifdef linux
	#include <sys/types.h>
	#include <dirent.h>
	#include <sys/stat.h>
#endif

// viewer
#if _MSC_VER == 1700
#include <DkImageContainer.h>
#include <DkNoMacs.h>	// init app
#include <DkViewPort.h>
//#include <DkImage.h>
#else
#include <DkNoMacs.h>		// init app
#include <DkViewPort.h>
#include <DkImage.h>
#endif

#pragma warning(push, 0)	// no warnings from includes - begin
// Qt includes
#include <QFileInfo>
#include <QReadLocker>
#include <QStringList>
#include <QSystemTrayIcon>
#include <QFutureWatcher>
#pragma warning(pop)		// no warnings from includes - end

// memory
//#include "mem/memmgr.h"

#include "DkTimer.h"
#include "DkMath.h"

using namespace cv;

// Well that's a bit disgusting...
// the problem we have here, is that we cannot
// connect signal/slots between different namespaces
// using namespace nmc; - cannot be used because we share the same code base
// so we just define the class we have issues with:
#define DkImageContainerT nmc::DkImageContainerT

enum training {
	DK_NO_TRAIN = 0, 
	DK_SAVE,
	DK_LCL,			// layout clustering
	DK_LINK,		//makes links of files in a directory to files in a search directory
	DK_FAST_PREVIEW,
	DK_EVALUATION,
	DK_RGB_MODE,
};

enum page_face{ONLY_FRONT = 0, ONLY_BACK, BOTH};
enum trainingSvm{DK_NO_SVM = 0, DK_TRAIN_TWO_CLASSES, DK_TRAIN_THREE_CLASSES, DK_TRAIN_NEW_TWO_CLASSES, DK_TRAIN_NEW_THREE_CLASSES, DK_TRAIN_LIBSVM_TEXT};
enum rotationMode{DK_COMBINED = 0, DK_FNNC, DK_GRADIENT};

class DkBase;

typedef  void (DkBase::*DkBaseMemFn)(QFileInfo fileInfo);
typedef  void (DkBase::*DkBaseComputeFunction)(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);

class DkSleeper : public QThread {
public:
	static void msleep(unsigned long msecs) {QThread::msleep(msecs);};
};

class DkBase : public QObject {
	Q_OBJECT

public:

	// this enum is used with compMode
	enum compStates {

		DK_FAST_COMPUTATION,		// uses xml data, if available (default)
		DK_FULL_COMPUTATION,		// always performs a full computation
	};


	// parameters
	static std::string gtTerm;
	static std::string maskTerm;
	static std::string backTerm;
	static std::string userFilename;

	// paths
	static std::string strDirPath;
	static std::string debugPath;
	static std::string savePath;
	static std::string gtPath;
	static std::string gtFilePath;
	static std::string texPath;
	static std::string texSvmPath;
	static std::string evalPath;
	static std::string formPath;
	static std::string formEvalPath;
	static std::string imgRootPath;
	static std::string mlPath;
	static std::string featureDirectory;
	static std::string bowVocabularyFile;
	static std::string lookupFile;
	static std::string classifierFilename;
	static std::string cascadeClassifierName;

	static int rectoVerso;
	static int mode;
	static int svmMode;
	static int rotMode;
	static int readMask;
	//static int classifierMode;

	static int compMode;
	static bool show;
	static bool recursive;
	static bool keepHierarchy;	/**< re-create the folders hierarchy**/

	// static function
	static Mat qImage2Mat(QImage img);
	static QImage mat2QImage(Mat img);
	static void enableFloatingPointExceptions();

	DkBase();
	virtual ~DkBase();
	
	void init();

	void scriptNomacs();
	virtual void script(QFileInfo fileInfo = QFileInfo());
	void loadRecursive(QDir file, DkBaseMemFn recFunction);

	void readConfigFile(std::string filename);
	virtual void preLoad(QFileInfo fileInfo = QFileInfo());
	virtual void postLoad();
	
	//void run();	// thread
	//void stop();

	virtual void saveDebug(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) = 0;
	virtual void showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) = 0;
	virtual DkBaseComputeFunction resolveFunction(int mode);
	void makeLink(QFileInfo filePath);

	void plot(Mat img, Mat mask, bool normImgs = false);
	
signals:
	void messageSignal(QString msg, int time, int location = 0);	// 0 == center info
	void updateImageSignal(QImage img);
	void updateSlaveImageSignal(QImage img);
	void setTitleSignal(QFileInfo file, QSize attr = QSize());
	void setTitleSlaveSignal(QFileInfo file, QSize attr = QSize());

public slots:

	// key events...
	virtual void mouseClicked(QMouseEvent* event, QPoint imgPos);
	virtual void keyPressEvent(QKeyEvent* event);
	void setImage(QSharedPointer<DkImageContainerT> imgFile = QSharedPointer<DkImageContainerT>());
	void clientInitialized();
	void close();
	void computeFinished();

protected:

	std::string cFilename;
	DkTimer totalTime;
	int oldMode;

	QReadWriteLock lock;
	bool isActive;
	bool somethingTodo;

	QVector<QSharedPointer<DkImageContainerT> > dirFileList;	// used for re-link
	QSharedPointer<DkImageContainerT> currentFile;
	QSharedPointer<DkImageContainerT> maskFile;

	int numSkipFiles;
	int numClientsInitialized;
	int generalCnt;
	QStringList generalStr;

	nmc::DkNoMacsIpl* nomacs;
	nmc::DkNoMacsIpl* nomacsSlave;
	QSystemTrayIcon* trayIcon;
	QFutureWatcher<void> computeWatcher;

	//vector<string> getFilenames();
	
	std::string createKeyFileName(std::string filename, std::string term = maskTerm);
	void connectViewer(nmc::DkNoMacsIpl* nomacs);
	void setLoadFilters(nmc::DkImageLoader* loader);
	virtual void compute();
	
	virtual void indexDir(QFileInfo fileInfo = QFileInfo());

	virtual void deleteAllFiles(QDir &dir);

	bool getCvImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile, cv::Mat& img, cv::Mat& mask);
	virtual void folderFinished() {};	//diem: things that need to be done if a folder is finished

};
