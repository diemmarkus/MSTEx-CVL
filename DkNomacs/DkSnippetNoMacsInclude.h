/*******************************************************************************************************
 DkSnippetNoMacsInclude.h
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
#pragma warning(disable: 4996)

//#include <opencv/cv.h>

// msvc2012 x64
#if _MSC_VER == 1700 && defined(DK_INCLUDES) && defined(_WIN64) && defined(_DEBUG) 

#pragma comment (lib, "../../../DkSnippetNoMacs/lib2012x64/nomacsd.lib")
#pragma comment (lib, "../../../lib2012x64/opencv_highgui242d.lib")


#elif _MSC_VER == 1700 && defined(DK_INCLUDES) && defined(_WIN64)

//#pragma comment (lib, "../../../DkSnippetNomacs/lib2012x64/libraw.lib")
//#pragma comment (lib, "../../../DkSnippetNomacs/lib2012x64/libexpat.lib")
//#pragma comment (lib, "../../../DkSnippetNomacs/lib2012x64/zlib1.lib")
//#pragma comment (lib, "../../../DkSnippetNomacs/lib2012x64/exiv2.lib")
#pragma comment (lib, "../../../DkSnippetNoMacs/lib2012x64/nomacs.lib")
#pragma comment (lib, "../../../lib2012x64/opencv_highgui242.lib")

// msvc2012 x86
#elif _MSC_VER == 1700 && defined(DK_INCLUDES) && defined(_DEBUG)

#pragma comment (lib, "../../../DkSnippetNoMacs/lib2012/nomacsd.lib")
#pragma comment (lib, "../../../lib2012/opencv_highgui242d.lib")
#pragma comment (lib, "../../../lib2012/opencv_flann242d.lib")
#pragma comment (lib, "../../../lib2012/opencv_features2d242d.lib")
#pragma comment (lib, "../../../lib2012/opencv_nonfree242d.lib")
#pragma comment (lib, "../../../lib2012/opencv_objdetect242d.lib")

#elif _MSC_VER == 1700 && defined(DK_INCLUDES)

#pragma comment (lib, "../../../DkSnippetNoMacs/lib2012/nomacs.lib")
#pragma comment (lib, "../../../lib2012/opencv_highgui242.lib")
#pragma comment (lib, "../../../lib2012/opencv_flann242.lib")
#pragma comment (lib, "../../../lib2012/opencv_features2d242.lib")
#pragma comment (lib, "../../../lib2012/opencv_nonfree242.lib")
#pragma comment (lib, "../../../lib2012/opencv_objdetect242.lib")

// msvc2010 x86
#elif defined(_DEBUG) && defined(DK_INCLUDES)

#pragma comment (lib, "../../../lib/opencv_highgui242d.lib")
#pragma comment (lib, "../../../lib/opencv_flann242d.lib")
#pragma comment (lib, "../../../lib/opencv_features2d242d.lib")
#pragma comment (lib, "../../../lib/opencv_nonfree242d.lib")
#pragma comment (lib, "../../../lib/opencv_objdetect242d.lib")

#elif defined DK_INCLUDES

#pragma comment (lib, "../../../lib/opencv_highgui242.lib")
#pragma comment (lib, "../../../lib/opencv_flann242.lib")
#pragma comment (lib, "../../../lib/opencv_features2d242.lib")
#pragma comment (lib, "../../../lib/opencv_nonfree242.lib")
#pragma comment (lib, "../../../lib/opencv_objdetect242.lib")

#endif
