/***************************************************
 *   DkError.cpp
 *   
 *   Created on: 05.02.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/

#include "DkError.h"
#include "DkUtils.h"

using std::string;

DkException::DkException(const std::string & msg) 
    : errMsg(msg),  errFile( "" ), errLine(-1) {
		exceptionName = "DkException";
}

DkException::DkException(const std::string & msg, int line, const std::string & file ) 
    : errMsg(msg), errFile(file), errLine(line)  {
		exceptionName = "DkException";
}


DkException::~DkException() throw() {
}


const char* DkException::what() const throw() {
	return errMsg.c_str();
}

const string DkException::Msg() const {
	string s;

	s = "\n>> " + exceptionName + ": " + errMsg + 
		"\n   in line: " + DkUtils::stringify(errLine) + " in file: " + errFile + "\n\n";

    return s;
}

int DkException :: Line() const {
    return errLine;
}

const string & DkException :: File() const {
    return errFile;
}

//DkIllegalArgumentException::DkIllegalArgumentException(const std::string &msg) : DkException(msg) {
//	exceptionName = "DkIllegalArgumentException";
//}
//DkIllegalArgumentException::DkIllegalArgumentException(const std::string &msg, int line, const std::string &file) 
//	: DkException(msg, line, file) {
//	exceptionName = "DkIllegalArgumentException";
//}
