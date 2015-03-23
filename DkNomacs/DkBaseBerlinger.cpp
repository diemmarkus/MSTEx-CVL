/***************************************************
 *   DkBaseStefan.cpp
 *   
 *   Created on: 08.11.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *				 Stefan Fiel
 *      Company: Vienna University of Technology
 ***************************************************/

#include "DkBaseBerlinger.h"
#include <iostream>
#include <fstream>
#include "DkSegmentation.h"
#include "DkCentralWidget.h"

static int filecount = 1;

DkBaseBerlinger::DkBaseBerlinger() : numBowClusters(100), bowTrainer(numBowClusters) {
	configPath = "";
}

void DkBaseBerlinger::init() {
	//DkBase::compMode = DK_FULL_COMPUTATION;

	// load default values
	DkBase::init();
	tpImg = 0;
	fpImg = 0;

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

DkBaseComputeFunction DkBaseBerlinger::resolveFunction(int mode) {

	DkBaseComputeFunction fnc = 0;

	switch (mode) {
		case DK_CREATE_VOC:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::createVocabulary);	break;
		case DK_CREATE_VOC_SEG:		fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::createVocabulary);	break;
		case DK_CREATE_VOC_OBJ:		fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::trainImageObjDetect);break;
		case DK_TRAIN_ML:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::trainImage);			break;
		case DK_TRAIN_ML_OBJ:		fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::trainImageObjDetect);break;
		case DK_TRAIN_ML_SEG:		fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::trainImageObjDetect);break;
		case DK_EVALUATION:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::showImages);			break;
		case DK_SHOW_VOC:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::showVocabulary);		break;
		case DK_OBJ_DETECT:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::detectObject);		break;
		case DK_TEST_API:			fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::testAPI);			break;
		case DK_TRACK:				fnc = static_cast<DkBaseComputeFunction>(&DkBaseBerlinger::track);				break;
		//case DK_SAMPLE:		callFunction(currentFile, img, maskFile, mask);	break;

		// if I don't know it - send it to the parent
		default:			fnc = DkBase::resolveFunction(mode);		break;
	}

	return fnc;
}

void DkBaseBerlinger::createVocabulary(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	Mat img, mask;

	// create feature object
	DkFeatureExtraction fe;

	std::string featurePath(DkBase::mlPath + DkBase::featureDirectory + "\\");
	QFileInfo featureFile(QString::fromStdString(featurePath),  imgFile->file().baseName() + QString::fromStdString(fe.getFileAttribute()));
	
	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	//cv::resize(img, img,  cv::Size(), 0.5, 0.5, CV_INTER_AREA);

	DkSlidingWindow burgerBox;
	burgerBox.setWindow(DkBox(DkVector(0,0), img.size()));
	bool isBackground = imgFile->file().filePath().contains("background") != 0;
	Mat segImg;

	// remove features which are not within the object detected
	if (mode == DK_CREATE_VOC_OBJ) {

		Mat dummy;
		burgerBox = getBestBox(img, dummy, isBackground);

		if (burgerBox.isEmpty()) {
			mout << "No object detected, rejecting..." << dkendl;
			return;
		}

		mout << "Burger box: " << burgerBox.getWindow() << dkendl;
	}
	else if (mode == DK_CREATE_VOC_SEG && !isBackground) {

		DkKMeansSegmentation segm(img);
		segm.compute();
		burgerBox.setWindow(segm.getBBox());
		segImg = segm.getSegImg();
	}

	DkTimer dt;
		
	// computation -------------------------------------------
	mout << "performing a full computation..." << dkendl;
	//fe.setMinSize(12);
	fe.setMaxNumDescriptors(500);
	fe.compute(img, burgerBox.getWindow().getCvRect());
	fe.filterKeypoints(segImg);
	//fe.write(featurePath, imgFile->file().baseName().toStdString());
	// computation -------------------------------------------

	mout << DkUtils::stringify(fe.getKeypoints().size()) << " keypoints extracted in " << dt << dkendl;

	// reduce keypoints
	Mat descriptors = fe.getDescriptors();
	int numDesc = cvFloor(descriptors.rows*0.5);
	numDesc = std::min(descriptors.rows, 300);

	if (!numDesc) {
		mout << "no descriptors left to add... skipping!" << dkendl;
		return;
	}

	resize(descriptors, descriptors, Size(descriptors.cols, numDesc), 0, 0, CV_INTER_NN);

	try {
		bowTrainer.add(descriptors);
		mout << descriptors.rows << " added for BoW training..." << dkendl;
	}
	catch(...) {
		mout << "exception caught when adding vocabulary, skipping!" << dkendl;
	}

	if (show) {
		Mat kpImg = fe.draw(img);

		if (mode == DK_CREATE_VOC_OBJ || mode == DK_CREATE_VOC_SEG) {
			DkIP::drawRectangle(kpImg, DkRectCorners(burgerBox.getWindow()), cv::Scalar(100, 100, 200), 0.2f);
		}
		plot(fe.getDescriptors(), kpImg, true);
	}

	imgFile->clear();
	if (maskFile) maskFile->clear();
}

void DkBaseBerlinger::watershedSegmentation(QSharedPointer<DkImageContainerT> imgFile, const std::vector<QPoint>& pos) {

	Mat img, mask;

	if (!getCvImages(imgFile, imgFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	DkTimer dt;

	//cv::cvtColor(img, img, CV_RGB2GRAY);
	Mat markerImg(img.size(), CV_32SC1, cv::Scalar(0));
	std::vector<DkVector> dots;
	for (int idx = 0; idx < pos.size(); idx++) {
		
		QPoint p = pos[idx];

		if (p.x() < 0 || p.x() > img.cols || p.y() < 0 || p.y() > img.rows)
			continue;
		
		int* px = markerImg.ptr<int>(p.y());
		px[p.x()] = idx+1;
		dots.push_back(DkVector((float)p.x(), (float)p.y()));	// visualization only
	}
	
	cv::watershed(img, markerImg);

	mout << "[WatershedSegmentation] image segmented in " << dt << dkendl;

	if (show) {
		DkIP::drawDots(img, dots);
		plot(img, markerImg, true);
	}
}

void DkBaseBerlinger::detectObject(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	DkTimer dtc;

	std::vector<cv::KeyPoint> kpts;

	int maxRows = 960;
	cv::resize(img, img, cv::Size(cvRound((float)maxRows/img.rows*img.cols), maxRows), 0, 0, CV_INTER_AREA);

	cv::SimpleBlobDetector::Params p;
	//p.filterByArea = false;
	p.maxArea = 5000;
	p.minArea = 500;
	p.filterByCircularity = false;
	//p.filterByConvexity = false;
	p.filterByColor = true;
	p.blobColor = 255;
	p.filterByCircularity = true;

	cv::SimpleBlobDetector detector(p);
	detector.detect(img, kpts);

	mout << kpts.size() << " simple blobs detected..." << dkendl;

	if (show) {

		Mat rImg = img.clone();
		rImg = DkFeatureExtraction::draw(rImg, kpts, DkUtils::yellow);
		plot(rImg, img);
	}


	//DkBurgerSegmentation segB(img);
	//DkInputParameter p;
	//p.detectorMaxArea = 0;
	//p.detectorMaxSide = 0;
	//segB.compute();

	//std::vector<DkPolyRect> unfilteredRects = segB.getRects();
	//std::vector<DkCircle> unfilteredCircles = segB.getCircles();
	//segB.filterDuplicates();

	//DkBurgerSegmentation segM(img);
	////segM.scale = 1.0f;
	//segM.looseDetection = false;
	//segM.compute();

	//std::vector<DkPolyRect> unfilteredRectsM = segM.getRects();
	//segM.filterDuplicates();

	//// compare object detector
	//Mat imgG;
	//cv::cvtColor(img, imgG, CV_RGB2GRAY);
	//float sf = 480.0f/imgG.cols;
	//cv::resize(imgG, imgG, cv::Size(), sf, sf, CV_INTER_AREA);
	//DkVector rval(1/sf, 1/sf);
	////DkVector rval(1,1);
	//std::vector<int> rLevels;
	//std::vector<double> levelWeights;

	//DkTimer dts;
	//std::vector<cv::Rect> boxes;
	//boxCascade.detectMultiScale(imgG, boxes, rLevels, levelWeights, 1.1, 5, 0, cv::Size(15,15), cv::Size(800, 800));
	//mout << boxes.size() << " object(s) detected in: " << dts << dkendl;

	////boxCascade.detectMultiScale( img, boxes,
	////	1.1, 2, 0
	////	//|CV_HAAR_FIND_BIGGEST_OBJECT
	////	//|CV_HAAR_DO_ROUGH_SEARCH
	////	|CV_HAAR_SCALE_IMAGE
	////	,
	////	Size(30, 30) );

	//Mat img1 = img.clone();

	//// rescale
	//for (size_t idx = 0; idx < boxes.size(); idx++) {
	//	cv::Rect& r = boxes.at(idx);
	//	DkBox dr = r;
	//	dr.uc = dr.uc.mul(rval);	
	//	dr.lc = dr.lc.mul(rval);
	//	r = dr.getCvRect();
	//}

	//if (show) {
	//	Mat img1 = img.clone();
	//	Mat img2 = img.clone();

	//	segB.draw(img1, unfilteredRects, cv::Scalar(150, 0, 0));
	//	segB.draw(img1);

	//	segM.draw(img2, unfilteredRectsM, cv::Scalar(150, 0, 0));
	//	//segM.draw(img2);

	//	for (size_t idx = 0; idx < boxes.size(); idx++) {
	//		cv::rectangle(img2, boxes.at(idx), DkUtils::blueDark, 2);
	//	}

	//	for (DkCircle c : unfilteredCircles) {
	//		c.draw(img1, cv::Scalar(150, 0, 0));
	//	}

	//	std::vector<DkCircle> circles = segB.getCircles();
	//	for (DkCircle c : circles) {
	//		c.draw(img1, DkUtils::yellow);
	//	}

	//	plot(img1, img2);
	//}
	//  --------------------------------------------------------------------

	//DkTimer dt;

	//Mat img, mask;

	//if (!getCvImages(imgFile, maskFile, img, mask)) {
	//	wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
	//	return;
	//}

	//Mat imgG;
	//cv::cvtColor(img, imgG, CV_RGB2GRAY);

	//float sf = 480.0f/imgG.cols;
	//cv::resize(imgG, imgG, cv::Size(), sf, sf, CV_INTER_AREA);
	//DkVector rval(1/sf, 1/sf);

	//std::vector<int> rLevels;
	//std::vector<double> levelWeights;

	//DkTimer dts;
	//std::vector<cv::Rect> boxes;
	//std::vector<cv::Rect> rejected;
	//boxCascade.detectMultiScale(imgG, boxes, rLevels, levelWeights, 1.1, 5, 0, cv::Size(15,15), cv::Size(800, 800));
	//mout << boxes.size() << " object(s) detected in: " << dts << dkendl;
	////boxCascade.detectMultiScale(imgG, rejected, rLevels, levelWeights, 1.1, 5, 0, cv::Size(15,15), cv::Size(800, 800), true);

	//Mat img1 = img.clone();
	//
	//// rescale
	//for (size_t idx = 0; idx < boxes.size(); idx++) {
	//	cv::Rect& r = boxes.at(idx);
	//	DkBox dr = r;
	//	dr.uc = dr.uc.mul(rval);	
	//	dr.lc = dr.lc.mul(rval);
	//	r = dr.getCvRect();
	//}

	//// rejected
	//for (size_t idx = 0; idx < rejected.size(); idx++) {
	//	cv::Rect& r = rejected.at(idx);
	//	DkBox dr = r;
	//	dr.uc = dr.uc.mul(rval);	
	//	dr.lc = dr.lc.mul(rval);
	//	r = dr.getCvRect();
	//}

	//if (show) {

	//	for (size_t idx = 0; idx < rejected.size(); idx++) {
	//		cv::rectangle(img1, rejected.at(idx), cv::Scalar(100, 40, 40), 2);
	//	}

	//	for (size_t idx = 0; idx < boxes.size(); idx++) {
	//		cv::rectangle(img1, boxes.at(idx), cv::Scalar(95, 116, 153), 4);
	//		cv::rectangle(img1, boxes.at(idx), cv::Scalar(130, 161, 206), 1);
	//	}

	//	// show the old classifier in the right window
	//	std::vector<int> rLevels;
	//	std::vector<double> levelWeights;

	//	cv::CascadeClassifier boxCascadeOld;
	//	std::string oldClassifierPath = "D:\\McDonalds\\classifier\\cascade3000.xml";
	//	if (!boxCascadeOld.load(oldClassifierPath))
	//		mout << "could not load OLD classifier: " << oldClassifierPath << dkendl;

	//	//DkTimer dts;
	//	std::vector<cv::Rect> boxesOld;
	//	boxCascadeOld.detectMultiScale(imgG, boxesOld, rLevels, levelWeights, 1.2, 15, 0, cv::Size(15,15), cv::Size(800, 800));
	//	mout << boxesOld.size() << " object(s) detected (OLD) in: " << dts << dkendl;

	//	for (size_t idx = 0; idx < boxesOld.size(); idx++) {
	//		cv::Rect& r = boxesOld.at(idx);
	//		DkBox dr = r;
	//		dr.uc = dr.uc.mul(rval);	
	//		dr.lc = dr.lc.mul(rval);
	//		r = dr.getCvRect();
	//	}

	//	Mat img2 = img.clone();
	//	for (size_t idx = 0; idx < boxesOld.size(); idx++) {
	//		cv::rectangle(img2, boxesOld.at(idx), cv::Scalar(95, 116, 153), 4);
	//		cv::rectangle(img2, boxesOld.at(idx), cv::Scalar(130, 161, 206), 1);
	//	}

	//	plot(img1, img2);
	//}

	//if (boxes.empty()) {
	//	emit messageSignal("No products found", -1);
	//	mout << "no products found..." << dkendl;
	//}

}

void DkBaseBerlinger::showVocabulary(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	DkTimer dt;

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	// compute features
	DkFeatureExtraction fe;
	fe.compute(img);
	mout << fe << dkendl;

	//DkBagger bagger(fe.getKeypoints(), fe.getDescriptors());
	//bagger.compute();

	if (fe.getKeypoints().empty()) {
		wout << "WARNING: no interest points detected, skipping..." << dkendl;
		return;
	}

	DkBoW bow;
	std::string savePath = DkBase::mlPath + "Dictionary\\";
	QString fName = (imgFile->file().isSymLink()) ? QFileInfo(imgFile->file().symLinkTarget()).fileName() : imgFile->file().fileName();
	bow.saveVocabulary(img, fe.getKeypoints(), fe.getDescriptors(), savePath, fName.toStdString());

	// visualization
	if (show) {
		Mat kpImg = fe.draw(img);

		plot(kpImg, kpImg, true);
	}
	else {
		imgFile->clear();

		if (maskFile)
			maskFile->clear();
	}

	mout << "[DkBaseBerlinger] computed in: " << dt  << dkendl;


}

void DkBaseBerlinger::trainImage(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	DkTimer dt;

	Mat img, mask;
	DkFeatureExtraction fe;

	std::string featurePath(DkBase::mlPath + DkBase::featureDirectory + "\\");
	QFileInfo featureFile(QString::fromStdString(featurePath),  imgFile->file().baseName() + QString::fromStdString(fe.getFileAttribute()));
	QFileInfo bowFile(QString::fromStdString(featurePath), imgFile->file().baseName() + QString::fromStdString(DkBagger::fileAttribute));

	mout << "processing: " << imgFile->file().absoluteFilePath().toStdString() << dkendl;

	if (compMode != DK_FAST_COMPUTATION || !featureFile.exists()) {
		if (!featureFile.absoluteDir().exists()) {
			QDir featureDir = featureFile.absoluteDir();
			featureDir.mkpath(".");
		}

		if (!getCvImages(imgFile, maskFile, img, mask)) {
			wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
			return;
		}

		// computation -------------------------------------------
		fe.compute(img);
		//fe.write(featurePath, imgFile->file().baseName().toStdString());
		// computation -------------------------------------------
	}
	else {
		fe.read(featurePath, imgFile->file().baseName().toStdString());
	}

	if (fe.getKeypoints().empty()) {
		wout << "WARNING: no interest points detected, skipping..." << dkendl;
		return;
	}

	DkBagger bagger(fe.getKeypoints(), fe.getDescriptors());

	if (compMode != DK_FAST_COMPUTATION || !bowFile.exists()) {
		bagger.compute();
		//bagger.write(featurePath, imgFile->file().baseName().toStdString());
	}
	else 
		bagger.read(featurePath, imgFile->file().baseName().toStdString());

	// add GT
	std::vector<DkSlidingWindow> cWindows = bagger.getWindows();
	DkLabelManager labelManager(imgFile->file().baseName().toStdString(), true);
	labelManager.addLabel();
	labelManager.labelWindows(cWindows);	// the label manager collects these labels

	classifierTrainer.addSamples(cWindows);

	// visualization
	if (show) {
		if (!getCvImages(imgFile, maskFile, img, mask)) {
			wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
			return;
		}

		Mat kpImg = fe.draw(img);
		Mat winImg = bagger.draw(img);

		plot(kpImg, winImg, true);
	}

	mout << "[DkBaseBerlinger] computed in: " << dt  << dkendl;

	imgFile->clear();

	if (maskFile)
		maskFile->clear();
}

void DkBaseBerlinger::trainImageObjDetect(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	DkTimer dt;

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	// maybe global params:
	float rescale = 0.5;
	int numScales = 4;
	Mat sImg = img;

	// first original size
	trainSingleImageObjDetect(imgFile, sImg, 0);

	for (int idx = 1; idx < numScales; idx++) {

		resize(sImg, sImg, cv::Size(), rescale, rescale, CV_INTER_AREA);
		trainSingleImageObjDetect(imgFile, sImg, idx);
	}

	mout << "[DkBaseBerlinger] computed in: " << dt  << dkendl;

	imgFile->clear();

	if (maskFile)
		maskFile->clear();
}

void DkBaseBerlinger::trainSingleImageObjDetect(QSharedPointer<DkImageContainerT> imgFile, cv::Mat img, int scale) {

	DkTimer dt;

	cv::Mat mask;
	DkFeatureExtraction fe;

	std::string featurePath(DkBase::mlPath + DkBase::featureDirectory + "\\");
	std::string baseFeatureFileName = imgFile->file().baseName().toStdString() + "_" + DkUtils::stringify(scale);
	QFileInfo featureFile(QString::fromStdString(featurePath),  QString::fromStdString(baseFeatureFileName) + QString::fromStdString(fe.getFileAttribute()));
	QFileInfo bowFile(QString::fromStdString(featurePath), QString::fromStdString(baseFeatureFileName) + QString::fromStdString(DkBagger::fileAttribute));
	
	bool isBackground = imgFile->file().filePath().contains("background") != 0;

	if (isBackground)
		mout << "training background..." << dkendl;

	try {
		//mout << "processing: " << imgFile->file().absoluteFilePath().toStdString() << dkendl;

		Mat segImg;
		DkSlidingWindow burgerBox = getBestBox(img, segImg, isBackground);

		// reject image if it is empty
		if (burgerBox.getWindow().isEmpty()) {
			mout << "no object detected - rejecting..." << dkendl;
			return;
		}

		if (compMode != DK_FAST_COMPUTATION || !featureFile.exists()) {
			if (!featureFile.absoluteDir().exists()) {
				QDir featureDir = featureFile.absoluteDir();
				featureDir.mkpath(".");
			}

			//std::vector<cv::Rect> boxes;
			//boxes.push_back(burgerBox.getCvRect());

			// computation -------------------------------------------
			fe.compute(img, burgerBox.getWindow().getCvRect());

			if (!segImg.empty())
				fe.filterKeypoints(segImg);
			//fe.write(featurePath, baseFeatureFileName);
			// computation -------------------------------------------
		}
		else {
			fe.read(featurePath, baseFeatureFileName);
			mout << "features loaded from: " << featurePath + imgFile->file().baseName().toStdString() << dkendl;
		}

		if (fe.getKeypoints().empty() || fe.getKeypoints().size() < 5) {
			wout << "WARNING: no (or too few) interest points detected, skipping..." << dkendl;
			return;
		}

		std::vector<DkSlidingWindow> cWindows;

		if (mode == DK_CREATE_VOC_OBJ) {
			Mat descriptors = fe.getDescriptors();

			int numDesc = 100;
			if (descriptors.rows > numDesc)
				resize(descriptors, descriptors, Size(descriptors.cols, numDesc), 0, 0, CV_INTER_NN);

			try {
				bowTrainer.add(descriptors);
				mout << descriptors.rows << " added for BoW training..." << dkendl;
			}
			catch(...) {
				mout << "exception caught when adding vocabulary, skipping!" << dkendl;
			}
		}
		else {
	
			std::vector<DkProductInfo> boxes;
			boxes.push_back(burgerBox);

			DkBagger bagger(fe.getKeypoints(), fe.getDescriptors());

			if (compMode != DK_FAST_COMPUTATION || !bowFile.exists()) {
				bagger.compute(boxes);
				//bagger.write(featurePath, baseFeatureFileName);
			}
			else 
				bagger.read(featurePath, baseFeatureFileName);

			// add GT
			cWindows = bagger.getWindows();
			bool newLabel = false;
			DkLabelManager labelManager(imgFile->file().baseName().toStdString(), true);
			labelManager.addLabel(&newLabel);
			labelManager.labelWindows(cWindows);	// the label manager collects these labels

			if (newLabel)
				featureWindows.insert(featureWindows.end(), cWindows.begin(), cWindows.end());

			classifierTrainer.addSamples(cWindows);
		}

		// visualization
		if (show) {

			Mat kpImg = img.clone();
			Mat objImg = img.clone();

			for (DkSlidingWindow cw : cWindows) {
				kpImg = DkFeatureExtraction::draw(kpImg, cw.getKeyPoints());
				cw.draw(objImg);
			}

			plot(kpImg, objImg);

			mout << "image shown..." << dkendl;
		}
	}
	catch (...) {
		mout << "[WARNING] Exception caught, skipping..." << dkendl;
	}

	mout << "[Single Image] computed in: " << dt  << dkendl;

	imgFile->clear();

	if (maskFile)
		maskFile->clear();
}

DkSlidingWindow DkBaseBerlinger::getBestBox(const Mat& img, Mat& segImg, bool isBackground) {

	Mat imgG;
	cv::cvtColor(img, imgG, CV_RGB2GRAY);
	std::vector<cv::Rect> boxes;
	DkSlidingWindow burgerBox;


	if (!isBackground) {
		
		if (mode == DK_CREATE_VOC_OBJ || mode == DK_TRAIN_ML_OBJ) {
		
			//// object detection -------------------------------------------
			//std::vector<int> rLevels;
			//std::vector<double> levelWeights;

			//DkTimer dts;
			//boxCascade.detectMultiScale(imgG, boxes, rLevels, levelWeights, 1.2, 15, 0, cv::Size(15,15), cv::Size(800, 800));

			//if (boxes.empty()) {
			//	mout << "no objects detected - skipping..." << dkendl;
			//	return DkBox();
			//}
			//else
			//	mout << boxes.size() << " object(s) detected in: " << dts << dkendl;


			//for (int idx = 0; idx < boxes.size(); idx++) {
			//	if (burgerBox.size().width < boxes.at(idx).size().width)
			//		burgerBox = boxes.at(idx);
			//}

			//// reject wrong ratios - to somehow detect object detection errors
			////float areaRatio = ((float)burgerBox.size().width*burgerBox.size().height)/(img.rows*img.cols);
			////if (areaRatio < 0.1f) {
			////	mout << "WRONG area ratio: " << areaRatio << " image rejected..." << dkendl;
			////	return DkBox();
			////}
			//// object detection -------------------------------------------

			DkBurgerSegmentation segM(img);
			segM.compute();
			segM.filterDuplicates();

			std::vector<DkPolyRect> prs = segM.getRects();
			double maxA = 0;

			for (DkPolyRect& p : prs) {

				if (p.getArea() > maxA) {
					burgerBox.setWindow(p.getBBox());
					burgerBox.setPolyRect(p);
					maxA = p.getArea();
				}
			}

		}
		else if (mode == DK_CREATE_VOC_SEG || mode == DK_TRAIN_ML_SEG) {
			DkKMeansSegmentation segM = img;
			segM.k = 1;
			segM.compute();
			burgerBox.setWindow(segM.getBBox());
			segImg = segM.getSegImg();
		}
	}
	else {
		int minSize = min(img.rows, img.cols);
		DkBox b(DkVector(), DkVector((float)minSize, (float)minSize));

		float randomScale = (float)(qrand()/RAND_MAX*0.5 + 0.5);
		b.setSize(randomScale*b.size());

		DkVector dxy(DkVector((float)img.cols, (float)img.rows)-b.size());
		dxy *= 0.5f;
		b.moveBy(dxy);

		burgerBox.setWindow(b);
	}

	return burgerBox;
}

void DkBaseBerlinger::showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	try {

		Mat img, mask;

		if (!getCvImages(imgFile, maskFile, img, mask)) {
			wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
			return;
		}

		//if (img.cols >= 1920)
		//	cv::resize(img, img, cv::Size(), 0.5, 0.5);

		// test burgers which cannot be detected yet --------------------------------------------------------------------
		DkTimer dtm;
		Mat segImg;
		int oldMode = mode;
		mode = DK_TRAIN_ML_SEG;
		DkSlidingWindow burgerBox = getBestBox(img, segImg, false);
		mode = oldMode;
		std::vector<DkProductInfo> win;
		win.push_back(burgerBox);

		win = recognizer.classify(img.data, (int)img.cols, (int)img.rows, (int)img.step, win);

		DkLabelManager labelManager(imgFile->file().absoluteFilePath().toStdString(), DkLabelManager::gt_path);	// hard coded GT
		labelManager.labelWindows(win);	// the label manager collects these labels
		evaluateResults(win, labelManager, confusionMatrix, detectionStats);

		// visualization
		if (show) {

			//Mat rImg = DkBagger::draw(img, win);
			cv::Mat lImg = img.clone();
			cv::Mat rImg = img.clone();

			for (size_t idx = 0; idx < win.size(); idx++) {

				const DkSlidingWindow& cWin = win[idx].getSlidingWindowConst();
				//mout << "win GT: " << cWin.getGtLabel() << dkendl;
				//mout << "label manager GT: " << labelManager.getGtLabelIdx() << dkendl;

				if (labelManager.getLabel(cWin.getLabel()) != "background")
					cWin.draw(lImg);
				else
					cWin.draw(rImg);

				rImg = DkFeatureExtraction::draw(rImg, cWin.getKeyPoints());
			}

			plot(lImg, segImg, true);
		}
		else {
			imgFile->clear();

			if (maskFile)
				maskFile->clear();
		}

		// test burgers which cannot be detected yet --------------------------------------------------------------------

	//	// reject image if it is empty
	//	if (burgerBox.isEmpty()) {
	//		//mout << "no object detected - rejecting..." << dkendl;
	//		return;
	//	}



	//	std::vector<DkProductInfo> win;

	//	std::vector<std::string> activeClasses;
	//	//activeClasses.push_back("McChicken");
	//	//activeClasses.push_back("EcteGoldstuicke");
	//	//activeClasses.push_back("Donut");

	//	activeClasses.push_back("5ChickenWings");
	//	activeClasses.push_back("BigMac");
	//	activeClasses.push_back("DerMorgenGut");
	//	activeClasses.push_back("Donut");
	//	activeClasses.push_back("FiletOFish");
	//	activeClasses.push_back("FrischZubereitet");
	//	activeClasses.push_back("HamburgerRoyalKaese");
	//	activeClasses.push_back("McChicken");

	//	cv::resize(img, img, cv::Size(), 2, 2, CV_INTER_LANCZOS4);

	//	// test new api
	//	win = recognizer.detect(img.data, (int)img.cols, (int)img.rows, (int)img.step, DkObjectRecognition::DK_DETECT_RECT);
	//	win = recognizer.classify(img.data, (int)img.cols, (int)img.rows, (int)img.step, win, activeClasses);

	//	mout << "whole recognition takes: " << dtm << dkendl;

	//	//DkLabelManager labelManager(imgFile->file().baseName().toStdString(), true);
	//	DkLabelManager labelManager(imgFile->file().absoluteFilePath().toStdString(), DkLabelManager::gt_path);	// hard coded GT
	//	labelManager.labelWindows(win);	// the label manager collects these labels
	//	evaluateResults(win, labelManager, confusionMatrix, detectionStats);

	//	// visualization
	//	if (show) {

	//		//Mat rImg = DkBagger::draw(img, win);
	//		cv::Mat lImg = img.clone();
	//		cv::Mat rImg = img.clone();

	//		for (size_t idx = 0; idx < win.size(); idx++) {

	//			const DkSlidingWindow& cWin = win[idx].getSlidingWindowConst();
	//			//mout << "win GT: " << cWin.getGtLabel() << dkendl;
	//			//mout << "label manager GT: " << labelManager.getGtLabelIdx() << dkendl;

	//			if (labelManager.getLabel(cWin.getLabel()) != "background")
	//				cWin.draw(lImg);
	//			else
	//				cWin.draw(rImg);

	//			rImg = DkFeatureExtraction::draw(rImg, cWin.getKeyPoints());
	//		}

	//		plot(lImg, rImg, true);
	//	}
	//	else {
	//		imgFile->clear();

	//		if (maskFile)
	//			maskFile->clear();
	//	}
	}
	catch (DkException e) {
		mout << "[DkException] caught: " << e.Msg() << dkendl;
	}
	catch (cv::Exception cve) {
		mout << "[CV Exception] caught: " << cve.msg << dkendl;
	}

}

void DkBaseBerlinger::testAPI(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {

	DkTimer dt;

	try {

		Mat img, mask;

		if (!getCvImages(imgFile, maskFile, img, mask)) {
			wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
			return;
		}

		//if (img.cols >= 1920)
		//	cv::resize(img, img, cv::Size(), 0.5, 0.5);

		DkTimer dtm;
	
		std::vector<DkProductInfo> win;
		std::vector<DkProductInfo> winRaw;
	
		if (false) {
			win = recognizer.recognize(img.data, (int)img.cols, (int)img.rows, (int)img.step);
		}
		else {

			std::vector<std::string> activeClasses;
			//activeClasses.push_back("Apfeltasche");
			//activeClasses.push_back("ChickenCordonBleu");
			//activeClasses.push_back("ChickenMcNuggets6er");
			//activeClasses.push_back("ChickenMcNuggets9er");
			//activeClasses.push_back("ChickenMcNuggets20er");
			//activeClasses.push_back("DerMorgenGut");
			//activeClasses.push_back("Donut");
			//activeClasses.push_back("EchteGoldstuecke");
			//activeClasses.push_back("FiletOFish");
			//activeClasses.push_back("FrischZubereitet");
			//activeClasses.push_back("HamburgerRoyalKaese");
			//activeClasses.push_back("HamburgerRoyalTS");
			//activeClasses.push_back("HamEggs");
			//activeClasses.push_back("McChicken");
			//activeClasses.push_back("McCroissant");
			//activeClasses.push_back("WienerFruehstueck");
			
			DkInputParameter params;
			params.detectorMaxSide = cvRound(img.cols*0.5f);
			params.detectorMinArea = 12000;
			params.detectorMaxArea = 80000;
			//params.detectorMaxArea = 8000000;	// general purpo
			params.rescaleArea = 1.0;
			params.classStrings = activeClasses;

			// test new api
			win = recognizer.detect(img.data, (int)img.cols, (int)img.rows, (int)img.step, DkObjectRecognition::DK_DETECT_RECT, params);

			//if (show) {
			//	params.filterDuplicates = false;
			//	winRaw = recognizer.detect(img.data, (int)img.cols, (int)img.rows, (int)img.step, DkObjectRecognition::DK_DETECT_RECT, params);
			//}

			//win.insert(win.begin(), DkProductInfo());	// debug

			//std::vector<char*> buffers;

			//for (size_t idx = 0; idx < win.size(); idx++) {
			//
			//	char* buffer;
			//	size_t bufferSize;	// we don't need the size here (fun fact here: the delete operator is the only one who implicitly knows the length)
			//
			//	win.at(idx).getStorageBuffer(&buffer, bufferSize);
			//	buffers.push_back(buffer);
			//}

			//// here would be some freaking good java code which distributes stuff etc...

			//std::vector<DkProductInfo> serializedWindows;
			//for (size_t idx = 0; idx < buffers.size(); idx++) {

			//	DkProductInfo tmpInfo;
			//	tmpInfo.setStorageBuffer(buffers.at(idx));

			//	serializedWindows.push_back(tmpInfo);
			//	delete buffers.at(idx);
			//}

			//win = recognizer.classify(img.data, (int)img.cols, (int)img.rows, (int)img.step, serializedWindows, activeClasses);
			win = recognizer.classify(img.data, (int)img.cols, (int)img.rows, (int)img.step, win, params);

			//buffers.clear();

			//for (size_t idx = 0; idx < win.size(); idx++) {

			//	char* buffer;
			//	size_t bufferSize;	// we don't need the size here (fun fact here: the delete operator is the only one who implicitly knows the length)

			//	win.at(idx).getStorageBuffer(&buffer, bufferSize);
			//	buffers.push_back(buffer);
			//}

			//// here would be some freaking good java code which distributes stuff etc...

			//serializedWindows.clear();

			//for (size_t idx = 0; idx < buffers.size(); idx++) {

			//	DkProductInfo tmpInfo;
			//	tmpInfo.setStorageBuffer(buffers.at(idx));

			//	serializedWindows.push_back(tmpInfo);
			//	delete buffers.at(idx);
			//}

			//win = serializedWindows;
		} 

		std::vector<DkSlidingWindow> sws;
		for (DkProductInfo info : win)
			sws.push_back(info.getSlidingWindow());

		DkTemplateMatcher matcher(sws);
		matcher.setReleaseDebug(DK_SAVE_IMGS);
		matcher.compute();
		sws = matcher.getSlidingWindows();

		std::vector<DkProductInfo> oldWin = win;
		win.clear();
		for (DkSlidingWindow sw : sws) {
			win.push_back(sw);
		}

		mout << "whole recognition takes: " << dtm << dkendl;

		//DkLabelManager labelManager(imgFile->file().baseName().toStdString(), true);
		DkLabelManager labelManager(imgFile->file().absoluteFilePath().toStdString(), DkLabelManager::gt_path);	// hard coded GT
		labelManager.labelWindows(win);	// the label manager collects these labels
		labelManager.labelWindows(oldWin);	// the label manager collects these labels
		evaluateResults(win, labelManager, confusionMatrix, detectionStats);

		// visualization
		if (show) {

			//Mat rImg = DkBagger::draw(img, win);
			cv::Mat lImg = img.clone();
			cv::Mat rImg = img.clone();

			//for (size_t idx = 0; idx < winRaw.size(); idx++) {
			//	const DkSlidingWindow& cWin = winRaw[idx].getSlidingWindowConst();
			//	DkPolyRect p = cWin.getPolyRect();
			//	p.draw(rImg, cv::Scalar(150,0,0));
			//}

			for (size_t idx = 0; idx < win.size(); idx++) {
			
				const DkSlidingWindow& cWin = win[idx].getSlidingWindowConst();
				//mout << "win GT: " << cWin.getGtLabel() << dkendl;
				//mout << "label manager GT: " << labelManager.getGtLabelIdx() << dkendl;

				if (labelManager.getLabel(cWin.getLabel()) != "background" && cWin.getLabel() != -1)
					cWin.draw(lImg);
				else
					cWin.draw(rImg);

				if (cWin.getLabel() == -1)
					cWin.draw(lImg);
								
				rImg = DkFeatureExtraction::draw(rImg, cWin.getKeyPoints());
				DkPolyRect p = cWin.getPolyRect();
				p.draw(rImg, DkUtils::yellow);
			}

			for (DkProductInfo info : oldWin)
				info.getSlidingWindowConst().draw(rImg);

			plot(lImg, rImg, true);
		}
		else {
			imgFile->clear();

			if (maskFile)
				maskFile->clear();
		}
	}
	catch (DkException e) {
		mout << "[DkException] caught: " << e.Msg() << dkendl;
	}
	catch (cv::Exception cve) {
		mout << "[CV Exception] caught: " << cve.msg << dkendl;
	}


	mout << "[DkBaseBerlinger] computed in: " << dt  << dkendl;

}

void DkBaseBerlinger::track(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {


	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	bool detectObjects = trackedObjects.empty();

	if (detectObjects)
		trackedObjects = recognizer.detect(img.data, (int)img.cols, (int)img.rows, (int)img.step, DkObjectRecognition::DK_DETECT_RECT);
	else {
		trackedObjects = recognizer.track(img.data, (int)img.cols, (int)img.rows, (int)img.step, trackedObjects);
		mout << "tracked objects: " << trackedObjects.size() << dkendl;
	}

	if (show) {

		//Mat rImg = DkBagger::draw(img, win);
		cv::Mat lImg = img.clone();
		cv::Mat rImg = img.clone();
		cv::Scalar col = (detectObjects) ? Scalar(100,100,100) : Scalar(130, 161, 206);

		for (size_t idx = 0; idx < trackedObjects.size(); idx++) {

			const DkSlidingWindow& cWin = trackedObjects[idx].getSlidingWindowConst();
			cWin.draw(lImg, col);
			mout << "painting: " << cWin.getWindow() << dkendl;

			rImg = DkFeatureExtraction::draw(rImg, cWin.getKeyPoints());
		}

		plot(lImg, rImg, true);
	}
}

void DkBaseBerlinger::evaluateResults(const std::vector<DkProductInfo>& windows, const DkLabelManager& labelManager, cv::Mat& confusionMatrix, cv::Mat& detectionStats) {

	if (detectionStats.empty()) {
		detectionStats = Mat(1, STATS_END, CV_32SC1, cv::Scalar(0));
	}
	if (confusionMatrix.empty()) {
		confusionMatrix = Mat(labelManager.getNumClasses(), labelManager.getNumClasses(), CV_32SC1, cv::Scalar(0));
	}

	// first evaluate the detection performance
	unsigned int* detectionPtr = detectionStats.ptr<unsigned int>();

	switch (windows.size()) {
	case 0: detectionPtr[FN]++; break;
	case 1: detectionPtr[TP]++; break;
	default: 
		detectionPtr[FP] += (unsigned int)windows.size()-1; 
		detectionPtr[TP]++;
	}

	int maxArea = 0;
	int maxIdx = -1;

	// classification performance
	for (int idx = 0; idx < windows.size(); idx++) {

		if (windows[idx].getRectangle().area() > maxArea) {
			maxArea = cvRound(windows[idx].getRectangle().area());
			maxIdx = idx;
		}
	}

	if (maxIdx == -1) {
		mout << "could not evaluate image..." << dkendl;
		return;
	}

	if (labelManager.getGtLabelIdx() >= 0) {

		unsigned int* confPtr = confusionMatrix.ptr<unsigned int>(labelManager.getGtLabelIdx());
		confPtr[windows[maxIdx].getLabel()]++;
		mout << "conf matrix index: (" << labelManager.getGtLabelIdx() << "," << windows[maxIdx].getLabel() << ")" << dkendl;
	}
}

void DkBaseBerlinger::saveDebug(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile) {
	
	DkTimer dt;
	// memory logger
	DkMem dm;

	Mat img, mask;

	if (!getCvImages(imgFile, maskFile, img, mask)) {
		wout << "Sorry, I cannot process empty images, skipping..." << dkendl;
		return;
	}

	plot(mask, img);

	mout << "[DkBaseBasic] computed in: " << dt  << dkendl;
	return;
}

void DkBaseBerlinger::trainBoW() {

	if (bowTrainer.descripotorsCount() < numBowClusters) {
		wout << "WARNING: cannot train BoW since you assigned " << bowTrainer.descripotorsCount() << " features, but we need at least " << numBowClusters << dkendl;
	}
	else {

		DkTimer dt;

		int featDim = 0;
		if (!bowTrainer.getDescriptors().empty())
			featDim = bowTrainer.getDescriptors()[0].cols;

		mout << "Clustering vocabulary using  " << bowTrainer.descripotorsCount() << " (dimension: " << featDim << ") features, this might take a minute..." << dkendl;
		Mat bowCenters = bowTrainer.cluster();	// number of cluster centers

		std::string bowFName = mlPath + DkBase::bowVocabularyFile;
		std::cout << "writing cluster centers to: " << bowFName << std::endl;
		FileStorage fs(bowFName, FileStorage::WRITE);

		if (!fs.isOpened()) {
			std::cout << "Sorry, I cannot write to: " << bowFName << std::endl;
			return;
		}

		//mDetector->write(fsDet);	// TODO: for now opencv has lots of bugs in I/O of detectors
		cv::write(fs, "vocabulary", bowCenters);

		fs.release();

		mout << "BoW vocabulary created in " << dt << dkendl;
	}
}

void DkBaseBerlinger::trainML() {

	mout << "\ntraining -------------------------------" << dkendl;

	switch (DkBase::classifierMode) {
	case DkClassifier::DK_UNKNOWN_CLASSIFIER:
		mout << "no classifier assigned - not training" << dkendl;
		break;

	case DkClassifier::DK_RANDOM_TREES: {

		mout << "converting trainer to random trees classifier..." << dkendl;
		//// well the first line below is more elegant than copying the sliding windows
		//// however, there is a design error - we open too many char handles in this case!
		//DkRandomTreesTrainer randomTrees(classifierTrainer);
		DkRandomTreesTrainer randomTrees;
		randomTrees.setSlidingWindows(classifierTrainer.getSlidingWindows());
		classifierTrainer.releaseData();	// clean up a bit
		randomTrees.compute();
		randomTrees.write(DkBase::mlPath, DkBase::classifierFilename);
		mout << randomTrees << dkendl;
		}
		break;

	case DkClassifier::DK_RANDOM_TREES_ONE_VS_ALL: {

		mout << "converting trainer to one vs all random trees classifier..." << dkendl;
		//// well the first line below is more elegant than copying the sliding windows
		//// however, there is a design error - we open too many char handles in this case!
		//DkRandomTreesTrainer randomTrees(classifierTrainer);
		
		DkMultiClassRTTrainer randomTrees;
		randomTrees.setSlidingWindows(classifierTrainer.getSlidingWindows());
		classifierTrainer.releaseData();	// clean up a bit
		randomTrees.compute();
		randomTrees.write(DkBase::mlPath, DkBase::classifierFilename);
		mout << randomTrees << dkendl;
		}
		break;

		//default:
		//	wout << "WARNING: unknown classifier mode: " << DkBase::classifierMode << dkendl;
	case DkClassifier::DK_RANDOM_TREES_ONE_VS_ONE: {

		mout << "converting trainer to one vs one random trees classifier..." << dkendl;

		DkOneVsOneMultiClassRTTrainer oldTrees;
		if (!oldTrees.load(DkBase::mlPath, DkBase::classifierFilename))
			mout << "could not load old model: " << classifierFilename << " training new..." << dkendl;

		DkOneVsOneMultiClassRTTrainer randomTrees;
		randomTrees.setSlidingWindows(classifierTrainer.getSlidingWindows());
		randomTrees.setOldClassifier(&oldTrees);
		classifierTrainer.releaseData();	// clean up a bit
		randomTrees.compute();
		randomTrees.write(DkBase::mlPath, DkBase::classifierFilename);
		mout << randomTrees << dkendl;
		}
		break;
	default:
		wout << "WARNING: unknown classifier mode: " << DkBase::classifierMode << dkendl;
	}
	
	if (DkLabelManager::write(DkBase::mlPath, DkBase::lookupFile))
		mout << "...written" << dkendl;
}

void DkBaseBerlinger::evalImages() {

	int sTP = 0;
	int sFP = 0;
	int invalid = 0;

	std::ofstream evalFile;
	evalFile.open(evalPath.c_str(), std::ios::out | std::ios::trunc);		// overwrite the file

	std::vector<DkSlidingWindow>::iterator evalIter = slidingWindows.begin();
	DkLabelManager m;
	

	while (evalIter != slidingWindows.end()) {
		DkSlidingWindow sw = *evalIter;

		if (sw.getLabelResult() == DkSlidingWindow::DK_TRUE_POSITIVE)
			sTP++;
		else if (sw.getLabelResult() == DkSlidingWindow::DK_FALSE_POSITIVE)
			sFP++;
		else {
			invalid++;
			evalIter++;
			continue;
		}

		evalFile << DkUtils::stringify(sw.getLabel()) << ", "
			<< DkUtils::stringify(sw.getGtLabel()) << ", "
			<< DkUtils::stringify(sw.getLabelResult() == DkSlidingWindow::DK_TRUE_POSITIVE) << ", "
			<< DkUtils::stringify(sw.getLabelResult() == DkSlidingWindow::DK_FALSE_POSITIVE) << ",\n";
		evalIter++;
	}
	evalFile.close();

	if (sTP + sFP > 0)
		mout << "overall precision: " << sTP/(float)(sFP+sTP)*100 << "% having " << sTP << " true positives and " << sFP << " false positives" << dkendl;
	mout << invalid << " invalid sliding windows..." << dkendl;

	mout << "precision per image: " << (float)tpImg/(fpImg+tpImg) << dkendl;
	mout << (fpImg+tpImg) << " images evaluated" << dkendl;

}

void DkBaseBerlinger::saveEvaluationResults(const std::string& fName) {

	if (!confusionMatrix.empty()) {

		std::ofstream filestream;
		string res = DkBase::evalPath + fName;
		filestream.open(res.c_str(), std::fstream::out | std::fstream::app);

		if (!filestream.is_open())
			mout << "could not write to: " << DkBase::evalPath + fName << dkendl;
		else {
			filestream << "detectionError = [";

			unsigned int* statPtr = detectionStats.ptr<unsigned int>();
			for (int idx = 0; idx < detectionStats.cols; idx++) {
				filestream << statPtr[idx] << ",";
			}
			filestream << "];\n";
			filestream << "confusionMatrix = [";
			for (int rIdx = 0; rIdx < confusionMatrix.rows; rIdx++) {
				unsigned int* statPtr = confusionMatrix.ptr<unsigned int>(rIdx);
				for (int idx = 0; idx < confusionMatrix.cols; idx++) {
					filestream << statPtr[idx] << ",";
				}
			}
			filestream << "];\n";
			filestream.close();
			mout << "results written to: " << DkBase::evalPath + fName << dkendl;
		}
	}

	mout << DkHandleLog::getHandleReport() << dkendl;


}

void DkBaseBerlinger::mouseClicked(QMouseEvent* event, QPoint imgPos) {

	if (event->modifiers() == Qt::ControlModifier)
		pts.push_back(imgPos);

	if (pts.size() > 2)
		watershedSegmentation(nomacs->getTabWidget()->getCurrentImage(), pts);
}

void DkBaseBerlinger::keyPressEvent(QKeyEvent* event) {

	if (event->key() == Qt::Key_F5) {
		trackedObjects.clear();
		pts.clear();
	}

	DkBase::keyPressEvent(event);
}

void DkBaseBerlinger::folderFinished() {

	mout << "Finished folder -----------------" << dkendl;

	if (!confusionMatrix.empty()) {

		QString folderName = currentFile->file().absolutePath().remove(QDir(QString::fromStdString(strDirPath)).absolutePath());
		mout << "removing: " << strDirPath << " from: " << currentFile->file().absolutePath().toStdString() << dkendl;
		folderName.replace("/", "-");
		folderName.replace("\\", "-");
		saveEvaluationResults(folderName.toStdString() + "-results.txt");

		// reset stats
		confusionMatrix.release();
		detectionStats.release();
	}

}

void DkBaseBerlinger::postLoad() {

	if (mode == DK_CREATE_VOC || mode == DK_CREATE_VOC_OBJ || mode == DK_CREATE_VOC_SEG || mode == DK_TRAIN_ALL) {
		trainBoW();
	}

	if (mode == DK_TRAIN_ML || mode == DK_TRAIN_ML_OBJ || mode == DK_TRAIN_ML_SEG || mode == DK_TRAIN_ALL) {
		trainML();
	}

	if (mode == DK_EVALUATION) {
		evalImages();
	}

	if (!featureWindows.empty()) {

		std::string fn = DkUtils::createFileName(DkBase::classifierFilename, "-templates");
		if (DkTemplateMatcher::write(DkBase::mlPath, fn, featureWindows))
			mout << featureWindows.size() << " template features written to " << DkBase::mlPath << fn << dkendl;

		// integrity check
		std::vector<DkSlidingWindow> w;
		DkTemplateMatcher::load(DkBase::mlPath, fn, w);

		if (w.size() != featureWindows.size()) {
			wout << "[WARNING] The number of templates do not correspond!" << dkendl;
		}
		else {
			for (int idx = 0; idx < w.size(); idx++) {


				if (w[idx].getKeyPoints().size() != featureWindows[idx].getKeyPoints().size()) {
					wout << "[WARNING] keypoint size does not correspond!"  << dkendl;
				}
				if (w[idx].getDescriptors().size() != featureWindows[idx].getDescriptors().size()) {
					wout << "[WARNING] descriptor size does not correspond!"  << dkendl;
					DkUtils::getMatInfo(w[idx].getDescriptors(), "descriptorsLoaded");
					DkUtils::getMatInfo(featureWindows[idx].getDescriptors(), "descriptors");
				}
			}
		}

		if (w.empty())
			wout << "[WARNING] no features could be loaded." << dkendl;
	}

	if (!DkBase::show && trayIcon) {

		QString msg = "I have finished in:" + QString::fromStdString(totalTime.getTotal());
		trayIcon->showMessage(msg, "Snippet processing u", QSystemTrayIcon::Critical, 10000);
	}


	DkBase::postLoad();
}

void DkBaseBerlinger::preLoad() {

	if (mode != DK_TEST_API && mode != DK_TRACK) {
		// init bow trainer 
		// currently we use 1000 cluster centers and try 3 attempts
		bowTrainer = BOWKMeansTrainer(numBowClusters, TermCriteria(), 3, KMEANS_PP_CENTERS);
		DkBoW::loadVocabulary(DkBase::mlPath, bowVocabularyFile);

		DkLabelManager::loadLookup(DkBase::mlPath, lookupFile);

		if (mode == DK_NO_TRAIN || mode == DK_EVALUATION)
			DkClassifier::loadClassifier(DkBase::mlPath, classifierFilename);

		if (boxCascade.load(DkBase::mlPath + DkBase::cascadeClassifierName))
			mout << DkBase::cascadeClassifierName << " loaded" << dkendl;
		else
			wout << "sorry, " << DkBase::mlPath + DkBase::cascadeClassifierName << " could not be loaded..." << dkendl;

		// load templates
		std::vector<DkSlidingWindow> wins;
		std::string tmpName = DkUtils::createFileName(classifierFilename, "-templates");
		if (DkTemplateMatcher::load(DkBase::mlPath, tmpName, wins))
			mout << tmpName << " loaded" << dkendl;
		else
			wout << "sorry, " << DkBase::mlPath + tmpName << " could not be loaded..." << dkendl;

		DkTemplateMatcher::templateWindows = wins;

	}
	if (mode == DK_TEST_API || mode == DK_TRACK)
		recognizer = DkObjectRecognition(DkBase::mlPath, bowVocabularyFile, classifierFilename, DkBase::cascadeClassifierName, lookupFile);

	if (!confusionMatrix.empty())
		saveEvaluationResults("evaluationResults.txt");


	//// set our ignore list
	//DkLabelManager::addActiveItem("allg");
	//DkLabelManager::addActiveItem("royal-kaese");

	//DkRandomTreesTrainer randomTrees(classifierTrainer);
	////randomTrees.compute();
	//randomTrees.write(DkBase::mlPath, DkBase::classifierFilename);
	//mout << randomTrees << dkendl;

	mout << DkHandleLog::getHandleReport() << dkendl;

	DkBase::preLoad();
}
