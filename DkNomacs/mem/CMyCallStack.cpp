/**
 *	Project DOCUMENT RECONSTRUCTION
 *	IPK Fraunhofer Gesellschaft
 *	Copyright 2006-2009 by IPK Berlin
 *
 *	\file		CMyCallStack.cpp
 *	\author		B.Widdecke
 */
#include "precomp.h"
#include "CMyCallStack.h"

CMyCallStack::CMyCallStack()
{
	this->pCallStackEntries = NULL;
}

void CMyCallStack::AddCallStackEntry(CallstackEntry &entry)
{
	PMyCallStackEntry tmp = (PMyCallStackEntry) malloc(sizeof(MyCallStackEntry));
	if (tmp)
	{
		strcpy_s(tmp->FileName, MAX_NAME_LENGTH, entry.lineFileName);
		strcpy_s(tmp->FunctionName, MAX_NAME_LENGTH, entry.name);
		tmp->iLineNumber = entry.lineNumber;
		tmp->pNext = this->pCallStackEntries;
		this->pCallStackEntries = tmp;
	}
}

void CMyCallStack::GetCallStack()
{
	this->bMainReached = false;
	this->bNewReached = false;

	this->ShowCallstack();
	
	this->bMainReached = false;
	this->bNewReached = false;
}

void CMyCallStack::OnCallstackEntry(CallstackEntryType eType, CallstackEntry &entry)
{
	eType;

	if (!this->bMainReached && this->bNewReached)
	{
		if (!strcmp(entry.name, "main"))
			this->bMainReached = true;
		this->AddCallStackEntry(entry);
	}

	if (!this->bNewReached)
	{
		if (!strcmp(entry.name, "operator new")    || !strcmp(entry.name, "operator new[]") ||
			!strcmp(entry.name, "operator delete") || !strcmp(entry.name, "operator delete[]"))
			this->bNewReached = true;
	}
}

void CMyCallStack::FreeCallStackEntries(PMyCallStackEntry pCallStackEntries)
{
	if (pCallStackEntries)
	{
		CMyCallStack::FreeCallStackEntries(pCallStackEntries->pNext);
		free(pCallStackEntries);
	}
}

PMyCallStackEntry CMyCallStack::GetAndRemoveCallStackEntries()
{
	PMyCallStackEntry res = this->pCallStackEntries;
	this->pCallStackEntries = NULL;
	return res;
}