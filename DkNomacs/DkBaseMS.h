/***************************************************
 *   DkBaseStefan.cpp
 *   
 *   Created on: 08.11.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *				 Stefan Fiel
 *      Company: Vienna University of Technology
 ***************************************************/

#pragma once

#include "DkBase.h"

#include "DkMachineLearning.h"

using namespace cv;

class DkBaseMS;


class DkBaseMS : public DkBase {

public:
	DkBaseMS();
	virtual ~DkBaseMS() {};

	enum {
		TP = 0,
		FP,
		FN,

		STATS_END,
	};

	// methods which differs from each user, should be overwritten
	void init(void);
	virtual void showImages(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void saveDebug(QSharedPointer<DkImageContainerT> imgFile, QSharedPointer<DkImageContainerT> maskFile);
	virtual void preLoad();
	virtual void postLoad();
	virtual DkBaseComputeFunction resolveFunction(int mode);

	std::string configPath;

protected:
	cv::Mat dbgImg;
	std::vector<QPoint> pts;

	virtual void folderFinished();
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void mouseClicked(QMouseEvent* event, QPoint imgPos);
	
};
