/*******************************************************************************************************
 DkMSModule.cpp
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

#include "DkSegmentationModule.h"

#include "DkTimer.h"
#include "DkSegmentation.h"
#include "DkRandomTrees.h"
#include "DkGrabCut.h"
#include "DkAce.h"

//#include <winsock2.h>	// needed since libraw 0.16

std::vector<std::wstring> DkSegmentationModule::indexFolder(const std::wstring& folderName) const {

	std::wstring folderIdxName = folderName + L"\\*.*";

	const wchar_t* fname = folderIdxName.c_str();


	WIN32_FIND_DATAW findFileData;
	HANDLE MyHandle = FindFirstFileW(fname, &findFileData);

	std::vector<std::wstring> fileNameList;
	std::wstring fileName;

	if( MyHandle != INVALID_HANDLE_VALUE) {

		do {

			fileName = findFileData.cFileName;
			fileNameList.push_back(fileName);
		} 
		while(FindNextFileW(MyHandle, &findFileData) != 0);
	}

	FindClose(MyHandle);

	//std::wcout << fileNameList.size() << " files indexed in " << folderName << std::endl;

	return fileNameList;
}
