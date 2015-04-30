/*******************************************************************************************************
 DkBaseMS.h
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

#include "DkBase.h"

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
