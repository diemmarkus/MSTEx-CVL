/**
 *	Project DOCUMENT RECONSTRUCTION
 *	IPK Fraunhofer Gesellschaft
 *	Copyright 2006-2009 by IPK Berlin
 *
 *	\file		memmgr.cpp
 *	\author		B.Widdecke
 */
#include "precomp.h"
#include <malloc.h>
#include <stdio.h>

#include "mem/CMyCallStack.h"
#include "mem/memmgr.h"

#ifdef WIN32

typedef struct _MemoryInfo
{
	void* ptr;
	int size;
	PMyCallStackEntry pcs;
	struct _MemoryInfo *next;
}MemoryInfo, *PMemoryInfo;

CMyCallStack *pCS = NULL;
PMemoryInfo pAllocationList = 0;
//PMemoryInfo *ppCurrentAllocationPos = &pAllocationList;
int curMemState = 0;
int maxMem = 0;

bool bLogAllocation = true;
bool bLogDeallocation = true;
bool isMmInit = false;

#if MM_COLLECT_MEMORY_USAGE
typedef struct
{
	int size;
	int count;
	char lastUserText[MAX_NAME_LENGTH];
	PMyCallStackEntry pcs;
}CollectedMemoryInfo;

CollectedMemoryInfo collectedMemoryUsage = {0, 0, 0, 0};

bool EnableLFH();

bool IsCallStackEqual(PMyCallStackEntry pA, PMyCallStackEntry pB)
{
	while(pA && pB)
	{
		if (strcmp(pA->FileName, pB->FileName) ||
			strcmp(pA->FunctionName, pB->FunctionName) ||
			(pA->iLineNumber != pB->iLineNumber))
			return false;

		if ((pA->pNext != 0) ^ (pB->pNext != 0))
			return false;

		pA = pA->pNext;
		pB = pB->pNext;
	}

	return !((pA != 0) ^ (pB != 0));
}

PMyCallStackEntry CopyCallStack(PMyCallStackEntry pDest, PMyCallStackEntry pSrc)
{
	PMyCallStackEntry res;
	CMyCallStack::FreeCallStackEntries(pDest);

	if (!pSrc)
		return 0;

	res = pDest = (PMyCallStackEntry) malloc(sizeof(MyCallStackEntry));
	if (!pDest)
		return res;

	pDest->FileName[0] = 0;
	pDest->FunctionName[0] = 0;
	pDest->iLineNumber = 0;
	pDest->pNext = 0;

	while(pSrc && pDest)
	{
		strcpy_s(pDest->FileName, MAX_NAME_LENGTH, pSrc->FileName);
		strcpy_s(pDest->FunctionName, MAX_NAME_LENGTH, pSrc->FunctionName);
		pDest->iLineNumber = pSrc->iLineNumber;

		if (pSrc->pNext)
			pDest->pNext = (PMyCallStackEntry) malloc(sizeof(MyCallStackEntry));
		else
			pDest->pNext = 0;

		pSrc = pSrc->pNext;
		pDest = pDest->pNext;
	}

	return res;
}

#endif

void printDbg(char *filename, char *usertext, PMemoryInfo info, bool bNegativAllocation)
{
#if MM_COLLECT_MEMORY_USAGE
	if (!strcmp(collectedMemoryUsage.lastUserText, usertext) &&
		IsCallStackEqual(collectedMemoryUsage.pcs, info->pcs))
	{
		collectedMemoryUsage.count++;
		collectedMemoryUsage.size += info->size;
		return;
	}
#endif

	// diem: ignore memory usage < 1 KB
	if (collectedMemoryUsage.size > 1024) {

		FILE* file;
		fopen_s(&file, filename, "a");
		if (file)
		{
	#if MM_COLLECT_MEMORY_USAGE
			if (collectedMemoryUsage.count)
			{
				int correction = 0;
				if (info)
					correction = (bNegativAllocation ? info->size : -info->size);

				fprintf(file, "%s;% 9i;% 10i;% 9i; ", collectedMemoryUsage.lastUserText, collectedMemoryUsage.count, curMemState + correction, collectedMemoryUsage.size);

				PMyCallStackEntry curPos = collectedMemoryUsage.pcs;
				if (curPos)
				{
					do
					{
						fprintf(file, "%s(% 4i)%s", curPos->FunctionName ? curPos->FunctionName : "<na>", curPos->iLineNumber, curPos->pNext ? "->" : ";");
						curPos = curPos->pNext;
					}while(curPos);
				}
				else
					fprintf(file, "<na>;");
				fprintf(file, "\n");
			}
	#else
			fprintf(file, "%s;        1;% 10i;% 9i;", usertext, curMemState, info->size);
			if (info)
			{
				PMyCallStackEntry curPos = info->pcs;

				if (curPos)
				{
					do
					{
						fprintf(file, "%s(%i)%s", curPos->FunctionName ? curPos->FunctionName : "<na>", curPos->iLineNumber, curPos->pNext ? "->" : ";");
						curPos = curPos->pNext;
					}while(curPos);
				}
				else
					fprintf(file, "<na>;");
			}
			fprintf(file, "\n");
	#endif
			fclose(file);

		}
	}


#if MM_COLLECT_MEMORY_USAGE
	if (info)
	{
		collectedMemoryUsage.pcs = CopyCallStack(collectedMemoryUsage.pcs, info->pcs);
		strcpy_s(collectedMemoryUsage.lastUserText, MAX_NAME_LENGTH, usertext);
		collectedMemoryUsage.count = 1;
		collectedMemoryUsage.size = info->size;
	}
	else
	{
		CMyCallStack::FreeCallStackEntries(collectedMemoryUsage.pcs);
		collectedMemoryUsage.pcs = NULL;
		collectedMemoryUsage.lastUserText[0] = 0;
		collectedMemoryUsage.count = 0;
		collectedMemoryUsage.size = 0;
	}
#endif
}

/*#include <map>
#include <fstream>
typedef std::map< int, __int64 > MSL;
MSL memSizeMap;
MSL maxSizeMap;

void save_memLog()
{
	char fn[256];
	sprintf(fn, "C:\\log_schrift_global.csv");
	std::ofstream ofs(fn);

	MSL::const_iterator it = maxSizeMap.begin();
	while (it != maxSizeMap.end())
	{
		if (it->second > 0)
			ofs << it->first << ";" << it->second << ";" << std::endl;
		it++;
	}
}*/

__inline void* allocMemory(size_t size)
{
	void* res = HeapAlloc(GetProcessHeap(), 0, size);// malloc(size);

	/*static bool blockLog = false;
	if (pCS && !blockLog)
	{
		blockLog = true;
		memSizeMap[size]++;
		if (memSizeMap[size] > maxSizeMap[size])
			maxSizeMap[size] = memSizeMap[size];
		blockLog = false;
	}*/

	if (!res)
	{
		char tmp[256];
		sprintf_s(tmp, 256, "Speicher konnte nicht reserviert werden (%i Byte) -> Defragmentiere Speicher", size);
		LogMemoryUsage("C:\\writing_feature.log", tmp, false);

		HANDLE h = GetProcessHeap();
		UINT mSize = (UINT) HeapCompact(h, 0);
		
		sprintf_s(tmp, 256, "Heap defragmentiert, groesster Speicherblock: %i Byte -> realloc (Heap-Handle: %s)", mSize, h ? "ok" : "fail");
		LogMemoryUsage("C:\\writing_feature.log", tmp, true);

		void * t = 0;
		for (mSize = 1024; mSize < 0x7FFFFFFF; mSize += 1024)
		{
			if (t)
				HeapFree(GetProcessHeap(), 0, t);
			try
			{
				t = HeapAlloc(GetProcessHeap(), 0, mSize);
				if (!t)
					throw 1;
			}
			catch(...)
			{
				t = 0;
				break;
			}
		}

		sprintf_s(tmp, 256, "Heap defragmentiert, groesster Speicherblock: %i Byte -> realloc (Heap-Handle: %s)", mSize, h ? "ok" : "fail");
		LogMemoryUsage("C:\\writing_feature.log", tmp, true);

		res = HeapAlloc(GetProcessHeap(), 0, size);//malloc(size);
	}

	if (res && bLogAllocation)
	{
		curMemState += (int)size;
		if (curMemState > maxMem)
			maxMem = curMemState;

#if MM_LOG_ONLY_ALLOC_SIZE == 0
		PMemoryInfo tmp = (PMemoryInfo) malloc(sizeof(MemoryInfo));
		if (tmp)
		{
			tmp->ptr = res;
			tmp->size = (int)size;
			tmp->next = 0;
			if (pCS)
			{
				pCS->GetCallStack();
				tmp->pcs = pCS->GetAndRemoveCallStackEntries();
			}
			else
				tmp->pcs = 0;

#if MM_LOG_MEMORY_USAGE
			printDbg(MM_LOG_FILE, "new", tmp, false);
#endif

			tmp->next = pAllocationList;
			pAllocationList = tmp;
		}
#endif
	}

	if (!res)
	{
		char tmp[256];
		sprintf_s(tmp, 256, "Speicher konnte nicht reserviert werden (%i Byte)", size);
		LogMemoryUsage("C:\\writing_feature.log", tmp, false);
	}

	return res;
}

__inline void freeMemory(void* _Ptr)
{
	if (_Ptr)
	{
		/*if (pCS)
			memSizeMap[HeapSize(GetProcessHeap(), 0, _Ptr)]--;*/

		if (bLogDeallocation)
		{
			curMemState -= (int)HeapSize(GetProcessHeap(), 0, _Ptr);//_msize(_Ptr);

#if MM_LOG_ONLY_ALLOC_SIZE == 0
			PMemoryInfo lastPos = 0;
			PMemoryInfo curPos = pAllocationList;
			while (curPos)
			{
				if (curPos->ptr == _Ptr)
				{
#if MM_LOG_MEMORY_USAGE
					printDbg(MM_LOG_FILE, "del", curPos, true);
#endif
					if (lastPos)
						lastPos->next = curPos->next;
					else
						pAllocationList = curPos->next;

					if (curPos->pcs)
						CMyCallStack::FreeCallStackEntries(curPos->pcs);
					free(curPos);

					break;
				}

				lastPos = curPos;
				curPos = curPos->next;
			}
#endif
		}
		
		
 		HeapFree(GetProcessHeap(), 0, _Ptr);
		//free(_Ptr);
	}
}

void mm_init()
{
	EnableLFH();
	if (!pCS)
		pCS = new CMyCallStack();
	isMmInit = true;
}

void mm_deinit()
{
	isMmInit = false;
#if MM_COLLECT_MEMORY_USAGE
	printDbg(MM_LOG_FILE, "flush", NULL, 0);
#endif

	CMyCallStack *p = pCS;
	pCS = NULL;
	if (p)
		delete p;

}

#include <iomanip>
#include <fstream>
#include <psapi.h>
#pragma comment(lib, "psapi.lib")

void LogMemoryUsage(char* filename, char* info, bool bLogOnlyInfo)
{	
	extern bool bLogAllocation;
	extern bool bLogDeallocation;

	bool b1 = bLogAllocation;
	bool b2 = bLogDeallocation;
	bLogAllocation = false;
	bLogDeallocation = false;

	{
		std::ofstream ofs(filename, std::ios_base::app);

		PROCESS_MEMORY_COUNTERS pmc;
		pmc.cb = sizeof(pmc);
		GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));

		extern int maxMem;
		extern int curMemState;

		ofs << "-----------------------------------" << std::endl;
		if (bLogOnlyInfo)
			ofs << info << std::endl;
		else
		{
			ofs << info << ":" << std::endl;
			ofs << "Speicher       (DLL):     " << std::setw(9) << curMemState << std::endl;
			ofs << "Speicherspitze (DLL):     " << std::setw(9) << maxMem << std::endl;
			ofs << "Speicher       (Prozess): " << std::setw(9) << pmc.WorkingSetSize << std::endl;
			ofs << "Speicherspitze (Prozess): " << std::setw(9) << pmc.PeakWorkingSetSize << std::endl;
			ofs << "-----------------------------------" << std::endl;
		}
	}

	bLogAllocation = b1;
	bLogDeallocation = b2;
}

bool EnableLFH()
{
	ULONG info;
	if (HeapQueryInformation(GetProcessHeap(), HeapCompatibilityInformation, &info, sizeof(ULONG), 0))
	{
		char tmp [256];
		sprintf_s(tmp, 256, "Current Heap Mode: %i\nSetting Heap to 2", info);
		//LogMemoryUsage("C:\\writing_feature.log", tmp, true);

		info = 2;
		if (HeapSetInformation(GetProcessHeap(), HeapCompatibilityInformation, &info, sizeof(ULONG)))
		{
			sprintf_s(tmp, 256, "Setting Heap OK");
			//LogMemoryUsage("C:\\writing_feature.log", tmp, true);
		}
		else
		{
			sprintf_s(tmp, 256, "Setting Heap FAIL (Code: %i)", GetLastError());
			//LogMemoryUsage("C:\\writing_feature.log", tmp, true);
			return false;
		}
	}
	else
	{
		char tmp [256];
		sprintf_s(tmp, 256, "Error @ HeapQueryInformation (Code: %i)", GetLastError());
		//LogMemoryUsage("C:\\writing_feature.log", tmp, true);
		return false;
	}

	return true;
}

#if MM_USE_INTERN_ALLOCATION



void* operator new(size_t size)
{
	if (isMmInit)
		return allocMemory(size);
	else
		return malloc(size);
}

void* operator new[](size_t size)
{
	if (isMmInit)
		return allocMemory(size);
	else
		return malloc(size);
}

void operator delete(void* _Ptr)
{

	if (isMmInit)
		return freeMemory(_Ptr);
	else
		return free(_Ptr);
}

void operator delete[](void* _Ptr)
{
	if (isMmInit)
		return freeMemory(_Ptr);
	else
		return free(_Ptr);
}

#endif
#endif

// do nothing
#ifdef linux
void mm_init()}
void mm_deinit(){}

//void save_memLog(){}

void LogMemoryUsage(char* filename, char* info, bool bLogOnlyInfo) {}

#endif