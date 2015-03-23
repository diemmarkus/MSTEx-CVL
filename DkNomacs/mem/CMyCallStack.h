/**
 *	Project DOCUMENT RECONSTRUCTION
 *	IPK Fraunhofer Gesellschaft
 *	Copyright 2006-2009 by IPK Berlin
 *
 *	\file		CMyCallStack.h
 *	\author		B.Widdecke
 */

#include "StackWalker.h"

#define MAX_NAME_LENGTH 512

typedef struct _MyCallStackEntry
{
	int iLineNumber;
	char FunctionName[MAX_NAME_LENGTH];
	char FileName[MAX_NAME_LENGTH];

	struct _MyCallStackEntry *pNext;
}MyCallStackEntry, *PMyCallStackEntry;

class CMyCallStack : private StackWalker
{
private:
	bool bMainReached;
	bool bNewReached;
	PMyCallStackEntry pCallStackEntries;

	void AddCallStackEntry(CallstackEntry &entry);
	
public:
	CMyCallStack();

	void GetCallStack();
	void OnCallstackEntry(CallstackEntryType eType, CallstackEntry &entry);
	static void FreeCallStackEntries(PMyCallStackEntry pCallStackEntries);
	PMyCallStackEntry GetAndRemoveCallStackEntries(); 
};