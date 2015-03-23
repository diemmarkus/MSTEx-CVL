/**
 *	Project DOCUMENT RECONSTRUCTION
 *	IPK Fraunhofer Gesellschaft
 *	Copyright 2006-2009 by IPK Berlin
 *
 *	\file		memmgr.h
 *	\author		B.Widdecke
 */

#pragma once

#ifndef _MEMMGR_H
#define _MEMMGR_H

#define MM_USE_INTERN_ALLOCATION 0
#define MM_LOG_ONLY_ALLOC_SIZE 0

//#define MM_LOG_OUT_OF_MEMORY 1
#define MM_LOG_MEMORY_USAGE 0
#define MM_COLLECT_MEMORY_USAGE 1
//#define MM_LOG_ONLEY_LEAKS 0
#define MM_LOG_FILE "C:\\VSProjects\\data\\memory.log"

void mm_init();
void mm_deinit();

//void save_memLog();

void LogMemoryUsage(char* filename, char* info, bool bLogOnlyInfo);
#endif //_MEMMGR_H
