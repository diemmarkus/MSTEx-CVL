/*******************************************************************************************************
 DkBase.cpp
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

#include "DkBase.h"
#include "DkError.h"
#include "DkCentralWidget.h"

std::string DkBase::gtTerm = "_gt";
std::string DkBase::maskTerm = "_mask";
std::string DkBase::backTerm = "_back";
std::string DkBase::userFilename = "";

//paths
std::string DkBase::strDirPath = "";
std::string DkBase::debugPath = "";
std::string DkBase::savePath = "";
std::string DkBase::gtPath = "..\\..\\..\\DkModules\\xmldata\\gt_text.xml";
std::string DkBase::gtFilePath = "";
std::string DkBase::texPath = "C:\\VSProjects\\data\\train_tex.xml";
std::string DkBase::texSvmPath = "..\\..\\..\\DkModules\\xmldata\\svm_rbf.xml";
std::string DkBase::evalPath = "";
std::string DkBase::formPath = "C:\\VSProjects\\data\\";
std::string DkBase::formEvalPath = "C:\\VSProjects\\data\\formEval.csv";
std::string DkBase::imgRootPath = "H:\\img\\ipk\\record-impaul\\";
std::string DkBase::mlPath = "D:\\McDonalds\\configs\\";
std::string DkBase::featureDirectory = "Features";
std::string DkBase::bowVocabularyFile = "bowVocabulary.yml";
std::string DkBase::lookupFile = "classLookup.yml";
std::string DkBase::classifierFilename = "classifier.yml";
std::string DkBase::cascadeClassifierName = "haarcascade_frontalface_default.xml";

int DkBase::compMode = DkBase::DK_FAST_COMPUTATION;

bool DkBase::show = true;
bool DkBase::recursive = false;
bool DkBase::keepHierarchy = false;

int DkBase::rectoVerso = BOTH; //BOTH ONLY_BACK

int DkBase::mode = DK_NO_TRAIN;
int DkBase::svmMode = DK_NO_SVM;
int DkBase::rotMode = DK_COMBINED;
//int DkBase::classifierMode = DkClassifier::DK_UNKNOWN_CLASSIFIER;

int DkBase::readMask = 1;

Mat DkBase::qImage2Mat(QImage img) {

	Mat mat2;
	QImage imgRGB; // must be initialized here!

	if (img.format() == QImage::Format_ARGB32 || img.format() == QImage::Format_RGB32) {
		Mat mat = Mat(img.height(), img.width(), CV_8UC4, (uchar*)img.bits(), img.bytesPerLine());
		mat2 = Mat(mat.rows, mat.cols, CV_8UC3);
		int from_to[] = { 0,0,  1,1,  2,2 };
		cv::mixChannels(&mat, 1, &mat2, 1, from_to, 3);
		DkUtils::printDebug(DK_INFO, "qImage2Mat: argb32\n");
	}
	else if (img.format() == QImage::Format_RGB888) {
		
		mat2 = Mat(img.height(), img.width(), CV_8UC3, (uchar*)img.bits(), img.bytesPerLine());
		DkUtils::printDebug(DK_INFO, "qImage2Mat: rgb888\n");
	}
	else if (img.format() == QImage::Format_Indexed8) {

		mat2 = Mat(img.height(), img.width(), CV_8UC1, (uchar*)img.bits(), img.bytesPerLine());
		DkUtils::printDebug(DK_INFO, "qImage2Mat: 8UC1\n");
	}
	else {
		DkUtils::printDebug(DK_INFO, "sorry i cannot convert the format: %i, but i try to\n", (int)img.format());
		imgRGB = img.convertToFormat(QImage::Format_RGB888);
		mat2 = Mat(imgRGB.height(), imgRGB.width(), CV_8UC3, (uchar*)imgRGB.bits(), imgRGB.bytesPerLine());
	}

	mat2 = mat2.clone();	// we need to own the pointer

	return mat2; 
}

QImage DkBase::mat2QImage(Mat img) {
	
	QImage qImg;
	Mat cImg = img;


	// convert image
	if (cImg.depth() != CV_8U)
		img.convertTo(cImg, CV_8U, 255);

	if (cImg.type() == CV_8UC1) {
		Mat tmp;
		cvtColor(cImg, tmp, CV_GRAY2RGB);	// Qt does not support writing to index8 images
		cImg = tmp;
	}
	if (cImg.type() == CV_8UC3) {
		qImg = QImage(cImg.data, cImg.cols, cImg.rows, (int)cImg.step, QImage::Format_RGB888);
		qImg = qImg.copy();	// clone so we definitely own the pointer...
	}

	return qImg;
}

DkBase::DkBase(void) {

	trayIcon = 0;
	DkBase::mode = DK_NO_TRAIN;

}

DkBase::~DkBase(void) {
	
	computeWatcher.blockSignals(true);	// do not deliver anything after closing 

	if (trayIcon) {
		trayIcon->hide();	// windows will not hide it instantly
		delete trayIcon;
	}

}

// load classifiers etc...
void DkBase::preLoad(QFileInfo fileInfo) {

	nmc::DkSettings::load();

	totalTime = DkTimer();

#ifdef WITH_XERCES
	// get the XML data
	if (!(mode == DK_NO_TRAIN || mode == DK_SAVE) ) {
		
		try{
			DkXMLParser::initParser();
			DkXMLParser parser(gtPath);
			gtVec = parser.getGtData();
			printf("loading: %s...\n", gtPath.c_str());
		} catch(DkException de) {
			printf("%s\n", de.Msg());
			printf("could not load gt xml - trying to train without the xml...\n");
		}

	}

	if (mode == DK_TXT_EVAL) {
		DkTimer dt = DkTimer();
		DkXMLParser::initParser();
		DkXMLParser parser(txtPath);
		txtVec = parser.getTxtData();
		printf("training data read in: %s\n", dt.getTotal().c_str());
	}

	if (mode == DK_TEX_EVAL) {
		DkTimer dt = DkTimer();
		DkXMLParser::initParser();
		DkXMLParser parser(texPath);
		//DkXMLParser parser = DkXMLParser(IDR_XMLDATA3);
		texVec = parser.getTexData();
		DkTexture::setTrainData(&texVec);
		printf("training data read in: %s\n", dt.getTotal().c_str());
	}
#endif

	// init random
	srand((unsigned int)DkTimer::getTime());

	oldMode = mode;

	// start with fast preview
	if (userFilename == "" && show)
		mode = DK_FAST_PREVIEW;

	if (mode == DK_LINK) {
		DkTimer dt;
		indexDir();
		qDebug() << dirFileList;
		mout << "I indexed: " << dirFileList.size() << " files in: " << dt << dkendl;
	}

	DkUtils::printDebug(DK_MODULE, "pre-load has finished...\n\n");
}

void DkBase::scriptNomacs() {


	DkTimer dt;
	QString filePath = QString::fromStdString(strDirPath + userFilename);	// if user filename is empty -> do nothing...

	nmc::DkSettings::load();

	// init slave first -> so nomacs has the focus
	if (!nomacsSlave)
		nomacsSlave = new nmc::DkNoMacsIpl();
	if (!nomacs)
		nomacs = new nmc::DkNoMacsIpl();

	nomacsSlave->getTabWidget()->addTab();
	nomacs->getTabWidget()->addTab();

	// set our icon (so that we do not confuse viewers/snippet)
	QIcon myIcon = QIcon(QString(":/DkNomacs/dir.ico"));
	nomacs->setWindowIcon(myIcon);
	nomacsSlave->setWindowIcon(myIcon);

	// setup slave next to its master
	QRect g = nomacs->frameGeometry();
	nomacsSlave->move(g.right()+10, g.top());

	connectViewer(nomacs);
	connectViewer(nomacsSlave);

	//DkCentralWidget* widget = static_cast<DkCentralWidget*>(nomacs->centralWidget());
	// nomacs specific
	connect(nomacs->getTabWidget(), SIGNAL(imageLoadedSignal(QSharedPointer<DkImageContainerT>)), this, SLOT(setImage(QSharedPointer<DkImageContainerT>)));
	connect(this, SIGNAL(updateImageSignal(QImage)), nomacs->viewport(), SLOT(setEditedImage(QImage)));
	connect(this, SIGNAL(messageSignal(QString, int, int)), nomacs->viewport()->getController(), SLOT(setInfo(QString, int, int)));

	// slave specific
	connect(this, SIGNAL(updateSlaveImageSignal(QImage)), nomacsSlave->viewport(), SLOT(setEditedImage(QImage)));

	//// set up filters
	//setLoadFilters(nomacs->getTabWidget()->getImageLoader());
	
	nomacsSlave->saveSettings = false;	// the slave should not save any settings
	
	// now load the file (or user file)
	if (!filePath.isEmpty())
		nomacs->viewport()->loadFile(QFileInfo(filePath));
	else {
		nomacs->getTabWidget()->getCurrentImageLoader()->loadLastDir();
		nomacs->viewport()->loadFirst();
	}
	
	DkUtils::printDebug(DK_MODULE, "nomacs initialized in: %s\n", dt.getTotal().c_str());

}

void DkBase::script(QFileInfo fileInfo) {
	
	DkTimer dt;


	//// resolve symlinks
	//if (fileInfo.isSymLink()) {
	//	mout << "resolving: " << fileInfo.absoluteFilePath().toStdString() << dkendl;
	//	fileInfo = fileInfo.symLinkTarget();
	//}

	QString filePath = (fileInfo.exists()) ? fileInfo.absoluteFilePath() : QString::fromStdString(strDirPath + userFilename);	// if user filename is empty -> do nothing...

	nmc::DkSettings::load();
	nmc::DkImageLoader* loader = new nmc::DkImageLoader();
	nmc::DkImageLoader* maskLoader = new nmc::DkImageLoader();

	setLoadFilters(loader);

	QVector<QSharedPointer<DkImageContainerT> > files;

	// load config file & check if it exists
	if (userFilename.empty()) {

		QDir dir = QDir(filePath);
		loader->setDir(dir);

		// scan the folder recursively
		if (recursive) loadRecursive(dir, &DkBase::script);

		files = loader->getImages();
				
		//for (int idx = 0; idx < files.size(); idx++)
		//	mout << files[idx].toStdString() << dkendl;
		
		if (files.empty() && !recursive)
			throw DkFileException("sorry, the folder: " + filePath.toStdString() + " has no images...", __LINE__, __FILE__);
		else if (files.empty() && recursive) {
			mout << "no files found in: " << filePath.toStdString() << dkendl;
			return;
		}
		// NOTE if nomacs is set to recursive, we might process folders twice... this if prevents that
		else if (dir.absolutePath() != loader->getDir().absolutePath()) {
			mout << "folder already processed: " << dir.absolutePath().toStdString() << dkendl;
			return;
		}

		//if (!files.first()->loadImage())
		//	throw DkFileException("sorry, I could not load the first file in: " + filePath.toStdString(), __LINE__, __FILE__);

		mout << "\nprocessing folder: " << filePath.toStdString() << " [" << files.size() << "] files" << dkendl;
	}
	else {

		mout << userFilename << " entering user filename...." << dkendl;
		QSharedPointer<DkImageContainerT> img(new DkImageContainerT(QFileInfo(filePath)));

		if (!img->exists())
			throw DkFileException("sorry, the file: " + filePath.toStdString() + " does not exist...", __LINE__, __FILE__);

		//loader->loadFile(loader->getFile());
		//imgCv = qImage2Mat(loader->getImage());
		//compute();
		//DkUtils::printDebug(DK_MODULE, "user file...\n");
		DkUtils::printDebug(DK_MODULE, "userFile: %s\n", userFilename.c_str());

	}

	// create hierarchy
	if (keepHierarchy && QFileInfo(QString::fromStdString(imgRootPath)).exists()) {

		QString cPath = loader->getDir().absolutePath();
		QString basePath = QFileInfo(QString::fromStdString(strDirPath)).absoluteFilePath();
		cPath.replace(basePath, QString::fromStdString(imgRootPath));

		QDir newPath(QString::fromStdString(imgRootPath));
		if (newPath.mkpath(cPath))
			mout << "I created: " << cPath.toStdString() << dkendl;

		debugPath = cPath.toStdString();
	}

		int idx = loader->findFileIdx(QFileInfo(filePath, QString::fromStdString(userFilename)), files);
		if (idx == -1) idx = 0;

		int s = (mode == DK_RGB_MODE) ? files.size() : 1;

		// processing loop
		// uncomment files.size() if you want to process all files
		for (; idx < s; idx+=numSkipFiles) {

			// current file
			currentFile = files[idx];

			if (!currentFile) {
				wout << "File Container is a NULL pointer!" << dkendl;
				continue;	// should never happen
			}

			if (!currentFile->exists()) {
				mout << currentFile->file().absoluteFilePath().toStdString() << " does NOT exist....\n" << dkendl;
				continue;
			}

			// read mask
			if (readMask) {
			
				QFileInfo mFile(currentFile->file().absoluteDir(), QString::fromStdString(createKeyFileName(currentFile->file().fileName().toStdString())));
				maskFile = QSharedPointer<DkImageContainerT> (new DkImageContainerT(mFile));

				mout << "mask name: " << maskFile->file().absoluteFilePath().toStdString() << dkendl;

				//// get the slave's loader
				//if (maskFile->loadImage()) {
				//	mout << "loading mask file: " << maskFile->image().width() << " x " << maskFile->image().height() << dkendl;
				//}
				//else
				//	mout << "sorry, I could not find the mask\n skipping: " << currentFile->file().fileName().toStdString() << dkendl;
			}

			//currentFile->loadImage();
		
			try {
				DkMem mem;
				compute();
				mout << mem << dkendl;
			}
			catch(DkException iae) {
				printf("%s\n", iae.Msg().c_str());
				wout << iae.Msg() << dkendl;
				emit messageSignal(QString::fromStdString(iae.Msg()), -1);
			}
			catch(cv::Exception& e) {
				wout << "OpenCV Error: " << e.what() << dkendl;
			}
			catch(std::exception& e) {
				wout << "Error: " << e.what() << dkendl;
			}
			catch(...) {
				wout << "ERROR - skipping: " << dkendl;
			}

			float pc = (float)(idx+1)/(float)files.size();
		
			// progress bar for flo
			std::string pb = "[";
			for (int pIdx = 0; pIdx < 20; pIdx++) {

				if (pc*20 > pIdx)
					pb += "=";
				else
					pb += " ";
			}
			pb += "]\n";
			mout << "\n\n" << pb << DkUtils::stringify(pc*100, 2) << "% computed...\n" << dkendl;

			// TODO: Do we still need a loader update?
			//// we need to do this a bit complicated, because we do not want to load in a thread
			//loader->loadFile(loader->getChangedFileInfo(1, true));
		}
	
	delete loader;
	delete maskLoader;
	
	//if (!fileInfo.isDir() && !fileInfo.exists())
	//	postLoad();

	folderFinished();

	if (dt.getTotalSec() > 10*60)
		trayIcon->showMessage("DkSnippet", "processing: " + filePath + "\n took me: " + QString::fromStdString(dt.getTotal()));

	//mout << "I finished processing..." << dkendl;

}

void DkBase::indexDir(QFileInfo fileInfo) {

	DkTimer dt;

	QString filePath = (fileInfo.exists()) ? fileInfo.absoluteFilePath() : QString::fromStdString(debugPath);	// if user filename is empty -> do nothing...

	nmc::DkImageLoader* loader = new nmc::DkImageLoader();

	setLoadFilters(loader);
	QStringList files;

	//mout << " searching folder..." << dkendl;
	QDir dir = QDir(filePath);
	loader->setDir(dir);
	QDir cDir = loader->getDir();

	// scan the folder recursively
	if (recursive) loadRecursive(dir, &DkBase::indexDir);

	QVector<QSharedPointer<DkImageContainerT> > tmpFiles = loader->getImages();
	qCopy(tmpFiles.begin(), tmpFiles.end(), dirFileList.end());

	delete loader;
}


//void DkBase::run() {
//
//	while (true) { 
//
//		lock.lockForWrite();
//		usleep(100);	// save cpu time
//
//		if (!isActive) {
//			DkUtils::printDebug(DK_MODULE, "main thread got a kill signal\n");
//			lock.unlock();
//			break;
//		}
//
//		// check mode and do something
//		if (somethingTodo) {
//
//			//if (nomacs->viewport()->getImageLoader()->getCurrentImage()->hasImage() && mode != DK_LCL) {
//			//	DkUtils::printDebug(DK_MODULE, "the image should not be empty, but it is...\n");
//			//	continue;
//			//}
//
//			try {
//				compute();
//			}
//			catch(DkException iae) {
//				printf("%s\n", iae.Msg().c_str());
//				wout << iae.Msg() << dkendl;
//				emit messageSignal(QString::fromStdString(iae.Msg()), -1);
//			}
//			somethingTodo = false;
//
//		}
//		lock.unlock();
//	}
//
//	//postLoad();
//}

void DkBase::compute() {

	if (computeWatcher.isRunning()) {
		mout << "I am already computing something - so I'll skip your request..." << dkendl;
		return;
	}


	QString msg;

	switch (mode) {
		case DK_NO_TRAIN:	msg = "computing..."; break;
		case DK_EVALUATION:	msg = "[Evaluation]"; break;
		case DK_SAVE:		msg = "[save debug]"; break;
		case DK_LINK:		msg = "[link imgs]"; break;
		case DK_RGB_MODE:	msg = "[rgb mode]"; break;
		case DK_LCL:		break;
		default:			
			emit messageSignal("unknown mode: " + mode, 3000);
			DkUtils::printDebug(DK_MODULE, "unknown mode: %i\n", mode);
			return;
	}
	
	emit messageSignal(msg, -1, 3);	// 3 == top left
	
	if (currentFile)
		mout << currentFile->file().fileName().toStdString() <<  " ------------------------------------" << dkendl;

	// ok we do something...
	// for safety: copy the images here as they could be changed by the main thread
	// update: false, you do not need to copy'em
	DkBaseComputeFunction fun = resolveFunction(mode);


//#ifndef _DEBUG
//	if (DkBase::show) {
//		// run the computation in a thread
//		computeWatcher.setFuture(QtConcurrent::run(this, fun, currentFile, maskFile));
//	}
//	else
//#endif
		(this->*fun)(currentFile, maskFile);
}

DkBaseComputeFunction DkBase::resolveFunction(int mode) {

	DkBaseComputeFunction fnc = &DkBase::showImages;

	switch (mode) {
	case DK_NO_TRAIN:	fnc = &DkBase::showImages;	break;
	case DK_SAVE:		fnc = &DkBase::saveDebug;	break;

	default:
		mout << "sorry, I do not know the mode: " << mode << dkendl;
	}

	return fnc;
}

void DkBase::computeFinished() {
	emit messageSignal("", 1, 3);	// stop msg
	mout << "finished computing..." << dkendl;

}

// store classifiers/results etc.
void DkBase::postLoad() {

	if (mode == DK_LINK) {
		mout << "end of link job - could not find " << generalCnt << "files:" << dkendl;
		for (int i=0; i< generalStr.size(); i++) {
			mout << "file: " << generalStr[i].toStdString() << dkendl;
		}
	}
	
	std::cout << "END of job - it took me " << totalTime << dkendl;

}

void DkBase::setLoadFilters(nmc::DkImageLoader* loader) {

	if (!loader)
		return;

	loader->ignoreKeywords.append(QString::fromStdString(maskTerm));
	loader->ignoreKeywords.append(QString::fromStdString(gtTerm));

	if (rectoVerso == ONLY_FRONT)
		loader->ignoreKeywords.append(QString::fromStdString(backTerm));
	else if (rectoVerso == ONLY_BACK)
		loader->keywords.append(QString::fromStdString(backTerm));

}

void DkBase::loadRecursive(QDir dir, DkBaseMemFn recFunction) {

	if (!recFunction) {
		wout << "you should specify a function when calling loadRecursive" << dkendl;
		return;
	}

	if (!dir.exists())
		return;
	
	// lots of fun if you do not filter . and .. dirs
	QFileInfoList dirList = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

	for (int idx = 0; idx < dirList.size(); idx++) {

		try {
			// this seems a bit nasty, but by these means we can load recursive on every member function
			(this->*recFunction)(dirList[idx]);
		}
		catch(DkFileException fe) {
			wout << fe.what() << dkendl;
		}
		catch(cv::Exception& e) {
			wout << "OpenCV Error: " << e.what() << dkendl;
		}
		catch(std::exception& e) {
			wout << "Error: " << e.what() << dkendl;
		}
		catch(...) {
			wout << "ERROR - skipping: " << dirList[idx].absoluteFilePath().toStdString() << dkendl;
		}
	}

}


void DkBase::connectViewer(nmc::DkNoMacsIpl* nomacs) {

	// connections from nomacs
	connect(nomacs, SIGNAL(closeSignal()), this, SLOT(close()));
	connect(nomacs->viewport(), SIGNAL(keyReleaseSignal(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));
	connect(nomacs->viewport(), SIGNAL(mouseClickSignal(QMouseEvent*, QPoint)), this, SLOT(mouseClicked(QMouseEvent*, QPoint)));
	connect(nomacs, SIGNAL(clientInitializedSignal()), this, SLOT(clientInitialized()));

}

void DkBase::mouseClicked(QMouseEvent*, QPoint) {

	// dummy here
}

void DkBase::keyPressEvent(QKeyEvent* event) {

	if (event->key() == Qt::Key_F) {
		
		if (mode != DK_FAST_PREVIEW) {
			oldMode = mode;
			mode = DK_FAST_PREVIEW; 
			emit messageSignal(QString("fast preview"), 3000);
		}
		else {
			mode = oldMode;
			emit messageSignal(QString("mode: computing what ever you like"), 3000);
		}
	}
	if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {

		if (mode == DK_FAST_PREVIEW) {
			mode = oldMode;
			emit messageSignal("switched to computing\n just for you...", 1000);
		}
		
		setImage(nomacs->getTabWidget()->getCurrentImage());

	}
	if (event->key() == Qt::Key_F3 && nomacs && nomacsSlave) {
		
		QRect g = nomacs->frameGeometry();
		nomacsSlave->move(g.right() + 10, g.top());
		nomacsSlave->resize(g.size() - (nomacsSlave->frameGeometry().size() - nomacsSlave->geometry().size())); // frame offset
	}
	if (event->key() == Qt::Key_A && nomacs) {
		emit messageSignal("this ain't the any key...", 2000);
	}

	//DkUtils::printDebug(DK_MODULE, "got a key event\n");
}

void DkBase::setImage(QSharedPointer<DkImageContainerT> imgFile) {
	
	DkTimer dt;

	// do not set new images until the computing thread has finished...
	if (somethingTodo)
		return;
	
	if (!imgFile && nomacs)
		imgFile = nomacs->getTabWidget()->getCurrentImage();

	// if image is still empty
	if (!imgFile || !imgFile->loadImage())
		return;

	currentFile = imgFile;
	//mout << "c file: " << currentFile->file().absoluteFilePath().toStdString() << dkendl;
	std::string maskFilename = createKeyFileName(currentFile->file().fileName().toStdString());
	
	QSharedPointer<nmc::DkImageLoader> loader = nomacsSlave->getTabWidget()->getCurrentImageLoader();
	maskFile = loader->findOrCreateFile(QFileInfo(currentFile->file().absoluteDir(), QString::fromStdString(maskFilename)));

	qDebug() << "loading: " << maskFile->file().absoluteFilePath(); 

	if (maskFile->loadImage()) {

		nomacsSlave->getTabWidget()->getCurrentImageLoader()->loadDir(maskFile->file());
		nomacsSlave->getTabWidget()->getCurrentImageLoader()->load(maskFile);
	}
	else {
		
		//mout << "sorry, I could not find the mask..." << dkendl;
		maskFile->clear();	// reset
		
		// do we need that state?
		readMask = false;
	}

	if (currentFile->image().size() != maskFile->image().size() && readMask) {
		emit messageSignal("the mask size does not correspond to the image size\nI won't compute that...", 3000);
		return;
	}

	if (mode != DK_FAST_PREVIEW) {
		compute();
	}

	mout << "image set in " << dt << dkendl;

}

void DkBase::clientInitialized() {

	numClientsInitialized++;
	if (numClientsInitialized == 2) {

		// ok, both clients have finished initialization
		nomacsSlave->syncWith(nomacs->getServerPort());
	}
}

void DkBase::close() {

	if (nomacs) {
		nomacs->close();
		nomacs = 0;
	}
	if (nomacsSlave) {
		nomacsSlave->close();
		nomacsSlave = 0;
	}

}

std::string DkBase::createKeyFileName(std::string filename, std::string term) {

	QString fileString = QString::fromStdString(filename);
	QFileInfo file = QFileInfo(fileString);

	QString ext = file.completeSuffix();
	fileString.remove(ext);
	fileString.remove(fileString.size()-1, fileString.size());	// remove dot
	fileString.append(QString::fromStdString(term));
	fileString.push_back(".");
	fileString.append(ext);


	//DkUtils::printDebug(DK_MODULE, "maskname: %s\n", fileString.toStdString().c_str());

	return fileString.toStdString();
}

bool DkBase::getCvImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile, cv::Mat& img, cv::Mat& mask) {

	if (!imgFile)
		return false;

	if (!imgFile->loadImage())
		return false;

	img = nmc::DkImage::qImage2Mat(imgFile->image());

	// remove alpha channel if it is there
	if (img.channels() == 4)
		cvtColor(img, img, CV_RGBA2BGR);

	if (img.empty())
		return false;

	// load the mask
	if (!maskFile) // was a mask assigned?	
		return true;

	maskFile->loadImage();
	mask = nmc::DkImage::qImage2Mat(maskFile->image());
	
	// remove alpha channel if it is there
	if (mask.channels() == 4)
		cvtColor(mask, mask, CV_RGBA2BGR);

	mout << "\n" << currentFile->file().fileName().toStdString() << " - (" << img.rows << " x " << img.cols << " " << img.channels() << " channels) -------------" << dkendl;

	return true;
}


void DkBase::deleteAllFiles(QDir &dir) {

	QFileInfoList files = dir.entryInfoList();

	for (int idx = 0; idx < files.size(); idx++) {

		if (files[idx].isFile()) {
			QString filePath = dir.absolutePath() + "//" + files[idx].fileName();
			dout << "removing: " << filePath.toStdString() << dkendl;
			QFile file(filePath);
			file.remove();
		}
	}


}

void DkBase::makeLink(QFileInfo filePath) {

	//QStringList files = dirFileList.filter(filePath.fileName());


	//if (files.size() == 1) {
	//	
	//	QFile imgFile(files[0]);
	//	QString linkFile = filePath.absoluteFilePath()+QString(".lnk");
	//	bool copied = imgFile.link(linkFile);

	//	if (!copied) {
	//		wout << "I could not create the link: " << linkFile.toStdString() << dkendl;
	//		generalCnt++;
	//		generalStr.append(files[0]);
	//	}
	//	else
	//		wout << linkFile.toStdString() << " created..." << dkendl;

	//}
	//else if (files.size() > 1) {
	//	wout << "I have found more than one matches for: " << filePath.fileName().toStdString() << dkendl;
	//	generalCnt++;
	//	generalStr.append(filePath.fileName());
	//	qDebug() << files;
	//}
	//else {
	//	wout << "I could not locate the file: " << filePath.fileName().toStdString() << dkendl;
	//	generalCnt++;
	//	generalStr.append(filePath.fileName());
	//}


	mout << "You're calling a STUB!" << dkendl;
	//QString searchFile;


}

void DkBase::readConfigFile(std::string filename) {


	printf("\n------------- reading config file: %s -------------\n\n", filename.c_str());
	std::ifstream evalFile;
	evalFile.open(DkUtils::stringToWchar(filename), std::ios::in);

	//evalFile << toString() << "\n\n";

	std::string line;
	while(std::getline(evalFile, line)) {
		std::stringstream  lineStream(line);
		std::string        cmd, param;	
		
		std::getline(lineStream, cmd, '=');
		std::getline(lineStream, param, '=');

		cmd = DkUtils::stringTrim(cmd);
		param = DkUtils::stringTrim(param);

		// remove quotes and semicolons
		param = DkUtils::stringRemove(param, ";");
		param = DkUtils::stringRemove(param, "\"");
		param = DkUtils::stringRemove(param, "\r");

		//printf("cmd: '%s'\n   param: '%s'\n", cmd.c_str(), param.c_str());

		// modes
		if (cmd == "DkBase::mode") {
			printf(">> mode = %s\n", param.c_str());
			//Enum.parse(typeof(trianing), "DK_NO_TRAIN");

			// very very ugly... (but fast to implement)
			if (param == "DK_NO_TRAIN") DkBase::mode = DK_NO_TRAIN;
			else if (param == "DK_LCL") DkBase::mode = DK_LCL;
			else if (param == "DK_EVALUATION") DkBase::mode = DK_EVALUATION;
			else if (param == "DK_RGB_MODE") DkBase::mode = DK_RGB_MODE;
			else DkUtils::printDebug(DK_WARNING, "[readConfigFile] I don't know: '%s'\n", param.c_str());
		}
		else if (cmd == "DkBase::svmMode") {

			printf(">> svmMode = %s\n", param.c_str());

			if (param == "DK_NO_SVM") DkBase::svmMode = DK_NO_SVM;
			else if (param == "DK_TRAIN_TWO_CLASSES") DkBase::svmMode = DK_TRAIN_TWO_CLASSES;
			else if (param == "DK_TRAIN_THREE_CLASSES") DkBase::svmMode = DK_TRAIN_THREE_CLASSES;
			else if (param == "DK_TRAIN_NEW_TWO_CLASSES") DkBase::svmMode = DK_TRAIN_NEW_TWO_CLASSES;
			else if (param == "DK_TRAIN_NEW_THREE_CLASSES") DkBase::svmMode = DK_TRAIN_NEW_THREE_CLASSES;
			else if (param == "DK_TRAIN_LIBSVM_TEXT") DkBase::svmMode = DK_TRAIN_LIBSVM_TEXT;
			else DkUtils::printDebug(DK_WARNING, "[readConfigFile] I don't know: '%s'\n", param.c_str());
		}
		else if (cmd == "DkBase::rotMode") {

			printf(">> rotMode = %s\n", param.c_str());

			if (param == "DK_COMBINED") DkBase::rotMode = DK_COMBINED;
			else if (param == "DK_FNNC") DkBase::rotMode = DK_FNNC;
			else if (param == "DK_GRADIENT") DkBase::rotMode = DK_GRADIENT;
			else DkUtils::printDebug(DK_WARNING, "[readConfigFile] I don't know: '%s'\n", param.c_str());
		}
		else if (cmd == "DkBase::compMode") {

			printf(">> compMode = %s\n", param.c_str());

			if (param == "DkBase::DK_FAST_COMPUTATION") DkBase::compMode = DK_FAST_COMPUTATION;
			else if (param == "DkBase::DK_FULL_COMPUTATION") DkBase::compMode = DK_FULL_COMPUTATION;
			else DkUtils::printDebug(DK_WARNING, "[readConfigFile] I don't know: '%s'\n", param.c_str());
		}
		else if (cmd == "DkUtils::setDebug") {

			printf(">> setDebug = %s\n", param.c_str());

			if (param == "DK_NONE") DkUtils::setDebug(DK_NONE);
			else if (param == "DK_WARNING") DkUtils::setDebug(DK_WARNING);
			else if (param == "DK_MODULE") DkUtils::setDebug(DK_MODULE);
			else if (param == "DK_DEBUG_A") DkUtils::setDebug(DK_INFO);
			else if (param == "DK_DEBUG_B") DkUtils::setDebug(DK_DEBUG_INFO);
			else if (param == "DK_DEBUG_C") DkUtils::setDebug(DK_DEBUG_C);
			else if (param == "DK_DEBUG_ALL") DkUtils::setDebug(DK_DEBUG_ALL);
			else DkUtils::printDebug(DK_WARNING, "[readConfigFile] I don't know: '%s'\n", param.c_str());
		}
		else if (cmd == "DkDebugStream::defaultLogFile") {

			printf(">> setLogFile = %s\n", param.c_str());
			
			DkDebugStream::defaultLogFile = param;
			wout.open(DkDebugStream::defaultLogFile);
			mout.open(DkDebugStream::defaultLogFile);
			iout.open(DkDebugStream::defaultLogFile);
			dout.open(DkDebugStream::defaultLogFile);

			// write the time to the log file
			time_t rawtime;
			struct tm * timeinfo;

			time(&rawtime);
			timeinfo = localtime(&rawtime);

			wout << "\n\n\nWelcome to DkSnippet " << asctime (timeinfo) << dkendl;
		}
		// paths
		else if (cmd == "DkBase::strDirPath") {
			printf(">> strDirPath = %s\n", param.c_str());
			DkBase::strDirPath = param;
		}
		else if (cmd == "DkBase::debugPath") {
			printf(">> debugPath = %s\n", param.c_str());
			DkBase::debugPath = param;
		}
		else if (cmd == "DkBase::savePath") {
			printf(">> savePath = %s\n", param.c_str());
			DkBase::savePath = param;
		}
		else if (cmd == "DkBase::gtPath") {
			printf(">> gtPath = %s\n", param.c_str());
			DkBase::gtPath = param;
		}
		else if (cmd == "DkBase::gtFilePath") {
			printf(">> gtFilePath = %s\n", param.c_str());
			DkBase::gtFilePath = param;
		}
		else if (cmd == "DkBase::mlPath") {
			printf(">> mlPath = %s\n", param.c_str());
			DkBase::mlPath = param;
		}
		else if (cmd == "DkBase::featureDirectory") {
			printf(">> featureDirectory = %s\n", param.c_str());
			DkBase::featureDirectory = param;
		}
		else if (cmd == "DkBase::bowVocabularyFile") {
			printf(">> bowVocabularyFile = %s\n", param.c_str());
			DkBase::bowVocabularyFile = param;
		}
		else if (cmd == "DkBase::lookupFile") {
			printf(">> lookupFile = %s\n", param.c_str());
			DkBase::lookupFile = param;
		}
		else if (cmd == "DkBase::classifierFilename") {
			printf(">> classifierFilename = %s\n", param.c_str());
			DkBase::classifierFilename = param;
		}
		else if (cmd == "DkBase::cascadeClassifierName") {
			printf(">> cascadeClassifierName = %s\n", param.c_str());
			DkBase::cascadeClassifierName = param;
		}
		else if (cmd == "DkBase::userFilename") {
			printf(">> userFilename = %s\n", param.c_str());
			DkBase::userFilename = param;
		}
		else if (cmd == "DkBase::readMask") {
			printf(">> readMask = %s\n", param.c_str());
			DkBase::readMask = (param == "true") ? 1 : 0;
		}
		else if (cmd == "DkBase::evalPath") {
			printf(">> evalPath = %s\n", param.c_str());
			evalPath = param;
		}
		else if (cmd == "DkBase::formPath") {
			printf(">> formPath = %s\n", param.c_str());
			DkBase::formPath = param;
		} else if (cmd == "DkBase::imgRootPath") {
			printf(">> imgRootPath = %s\n", param.c_str());
			DkBase::imgRootPath = param;
		}
		// for rotateImage
		else if (cmd == "DkBase::clip") {
			printf(">> clip IS NO LONGER SUPPORTED\n");
		}
		else if (cmd == "DkBase::show") {
			printf(">> show = %s\n", param.c_str());
			DkBase::show = (param == "true") ? 1 : 0;		
		}
		else if (cmd == "DkBase::numSkipFiles") {
			printf(">> DkBase::numSkipFiles = %s\n", param.c_str());
			bool numberOk = false;
			DkBase::numSkipFiles = QString::fromStdString(param).toInt(&numberOk);
			if (!numberOk) {
				DkBase::numSkipFiles = 1;
				printf(">> WARNING: %s is not a number\n", param.c_str());
			}
		}
		else if (cmd == "DkBase::recursive") {
			printf(">> recursive = %s\n", param.c_str());
			DkBase::recursive = (param == "true") ? 1 : 0;		
		}
		else if (cmd == "DkBase::keepHierarchy") {
			printf(">> keepHierarchy = %s\n", param.c_str());
			DkBase::keepHierarchy = (param == "true") ? 1 : 0;		
		}
		// window parameters
		else if (cmd == "DkBase::posX") {
			printf(">> posX = IS NO LONGER SUPPORTED\n");
		}
		else if (cmd == "DkBase::posY") {
			printf(">> posY = IS NO LONGER SUPPORTED\n");
		}
		else if (cmd == "DkBase::winSize") {
			printf(">> winSize = IS NO LONGER SUPPORTED\n");
		}
	}
	evalFile.close();

	printf("\n\n");
}

#ifdef WIN32
void DkBase::enableFloatingPointExceptions() {
	
	////Quelle:
	////http://msdn.microsoft.com/en-us/library/e9b52ceh.aspx
	////https://www.securecoding.cert.org/confluence/display/cplusplus/FLP03-CPP.+Detect+and+handle+floating+point+errors

	////Set the x86 floating-point control word according to what
	////exceptions you want to trap.
	//_clearfp(); //Always call _clearfp before setting the control
	////word
	////Because the second parameter in the following call is 0, it
	////only returns the floating-point control word
	//unsigned int cw = _controlfp(0, 0); //Get the default control
	////word
	////Set the exception masks off for exceptions that you want to
	////trap.  When a mask bit is set, the corresponding floating-point
	////exception is //blocked from being generating.
	//cw &=~(_EM_ZERODIVIDE|_EM_DENORMAL|_EM_INVALID);

	////For any bit in the second parameter (mask) that is 1, the
	////corresponding bit in the first parameter is used to update
	////the control word.
	//_controlfp(cw, _MCW_EM); //Set it.
}

#endif
#ifdef linux
void DkBase::enableFloatingPointExceptions() {
	return;
}
#endif
void DkBase::plot(Mat img, Mat mask, bool normImgs) {

	if (normImgs) {
	
		if (img.depth() != CV_32F)
			img.convertTo(img, CV_32F, 1.0f/255.0f);
	
		if (mask.depth() != CV_32F)
			mask.convertTo(mask, CV_32F, 1.0f/255.0f);
	
		if (img.depth() == CV_32F)
			normalize(img, img, 1.0f, 0, NORM_MINMAX);
		if (mask.depth() == CV_32F)
			normalize(mask, mask, 1.0f, 0, NORM_MINMAX);

		img.convertTo(img, CV_8U, 255);
		mask.convertTo(mask, CV_8U, 255);
	}

	if (nomacs) {
		QImage qImg = mat2QImage(img);
		emit updateImageSignal(qImg);
		//QCoreApplication::processEvents();
	}
	if (nomacsSlave) {
		nomacsSlave->getTabWidget()->loadFile(nomacs->getTabWidget()->getCurrentImage()->file());
		QImage qMask = mat2QImage(mask);
		emit updateSlaveImageSignal(qMask);
		//QCoreApplication::processEvents();
	}
}

void DkBase::init() {

	numSkipFiles = 1;
	DkBase::enableFloatingPointExceptions();
	connect(&computeWatcher, SIGNAL(finished()), this, SLOT(computeFinished()));

	cFilename = "";
	nomacs = 0;
	nomacsSlave = 0;
	numClientsInitialized = 0;
	generalCnt = 0;
	nmc::DkSettings::initFileFilters();

	somethingTodo = false;
	isActive = true;

	// init tray icon
	QIcon myIcon = QIcon(QString(":/DkNomacs/dir.ico"));
	trayIcon = new QSystemTrayIcon(myIcon);
	trayIcon->setToolTip("Seriously, you should know that icon...\n");
	trayIcon->show();

	//printf("default log file: \n", DkDebugStream::defaultLogFile.c_str());

	// since the open(NUL) is not in the constructor any more (due to performance issues) we need to init the log stream here
	wout.open(DkDebugStream::defaultLogFile);
	mout.open(DkDebugStream::defaultLogFile);
	iout.open(DkDebugStream::defaultLogFile);
	dout.open(DkDebugStream::defaultLogFile);
}
