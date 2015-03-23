/**************************************************
 * 	DkModuleInclude.h
 *
 *	Created on:	27.05.2014
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#pragma once

//#pragma warning(disable: 4996)

#ifdef WIN32
	#ifdef DK_MODULE_EXPORTS
	#define DK_MODULE_API __declspec(dllexport)
	#else
	#define DK_MODULE_API __declspec(dllimport)
	#endif

#endif

#ifdef linux
	#ifdef DK_MODULE_EXPORTS
	#define DK_MODULE_API
	#else
	#define DK_MODULE_API
	#endif
#endif


