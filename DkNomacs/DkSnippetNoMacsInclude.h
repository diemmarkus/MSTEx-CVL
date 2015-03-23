/**************************************************
 * 	DkSnippetNoMacsInclude.h
 *
 *	Created on:	28.08.2012
 * 	    Author:	Markus Diem
 *				Stefan Fiel
 *				Angelika Garz
 * 				Florian Kleber
 *     Company:	Vienna University of Technology
 **************************************************/

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
