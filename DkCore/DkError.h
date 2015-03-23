/***************************************************
 *   DkError.h
 *   
 *   Created on: 05.02.2010
 *       Author: Markus Diem
 *               Florian Kleber
 *      Company: Vienna University of Technology
 ***************************************************/
#pragma once

#pragma warning(disable: 4251)

#include <exception>
#include <string>

#ifdef linux
	#ifdef DK_CORE_EXPORTS
	#define DK_CORE_API
	#else
	#define DK_CORE_API
	#endif
#endif


#ifdef WIN32
	#ifdef DK_CORE_EXPORTS
	#define DK_CORE_API __declspec(dllexport)
	#else
	#define DK_CORE_API __declspec(dllimport)
	#endif
#endif

/**
 * The base exception class DK_CORE_API for the error management.
 **/
class DK_CORE_API DkException : public std::exception {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkException(const std::string & msg = "");
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkException(const std::string & msg, int line, const std::string & file);
	/**
	 * Default destructor.
	**/
	virtual ~DkException() throw();
	/**
	 * Returns the error message.
	 * @return The error message.
	**/
	virtual const char *what() const throw () ;
	/**
	 * Returns the error message.
	 * @return The error message.
	**/
	virtual const std::string Msg() const;
	/**
	 * Returns the line number of the error.
	 * @return The line number of the error.
	**/
	int Line() const;
	/**
	 * Returns the filename of the error.
	 * @return The filename of the error occurrence.
	**/
	const std::string& File() const;

protected:
	std::string exceptionName;				/**< Name of the exception **/
	std::string errMsg;						/**< Error message **/
	std::string errFile;					/**< Error filename **/
	int errLine;							/**< line number **/

};

/**
 * Exception class DK_CORE_API for illegal arguments.
 **/
class DK_CORE_API DkIllegalArgumentException : public DkException {
	
public:
	//DkIllegalArgumentException();
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkIllegalArgumentException(const std::string & msg = "") : DkException(msg) {exceptionName = "DkIllegalArgumentException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkIllegalArgumentException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkIllegalArgumentException";};
};

/**
 * Exception class DK_CORE_API if cast errors occur.
 **/
class DK_CORE_API DkCastException : public DkException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkCastException(const std::string & msg = "")  : DkException(msg) {exceptionName = "DkCastException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkCastException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkCastException";};
};

/**
 * Exception class DK_CORE_API for OpenCV Mats.
 **/
class DK_CORE_API DkMatException : public DkException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkMatException(const std::string & msg = "")  : DkException(msg) {exceptionName = "DkMatException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkMatException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkMatException";};
};
/**
 * Exception class DK_CORE_API for files.
 **/
class DK_CORE_API DkFileException : public DkException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkFileException(const std::string & msg = "")  : DkException(msg) {exceptionName = "DkFileException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkFileException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkFileException";};
};
/**
 * Exception class DK_CORE_API if specified files are not found.
 **/
class DK_CORE_API DkFileNotFoundException : public DkFileException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkFileNotFoundException(const std::string & msg = "")  : DkFileException(msg) {exceptionName = "DkFileNotFoundException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkFileNotFoundException(const std::string & msg, int line, const std::string & file) : DkFileException(msg, line, file) {exceptionName = "DkFileNotFoundException";};
};
/**
 * Exception class DK_CORE_API if specified files are empty.
 **/
class DK_CORE_API DkFileEmptyException : public DkFileNotFoundException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkFileEmptyException(const std::string & msg = "")  : DkFileNotFoundException(msg) {exceptionName = "DkFileEmptyException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkFileEmptyException(const std::string & msg, int line, const std::string & file) : DkFileNotFoundException(msg, line, file) {exceptionName = "DkFileEmptyException";};
};
/**
 * Exception class DK_CORE_API for XML.
 **/
class DK_CORE_API DkXMLException : public DkException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkXMLException(const std::string & msg = "")  : DkException(msg) {exceptionName = "DkXMLException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkXMLException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkXMLException";};
};

/**
 * Exception class DK_CORE_API if indicess are out of bounds.
 **/
class DK_CORE_API DkIndexOutOfBoundsException : public DkException {

public:
	/**
	 * Default constructor.
	 * @param msg The error message
	**/
	DkIndexOutOfBoundsException(const std::string & msg = "")  : DkException(msg) {exceptionName = "DkIndexOutOfBoundsException";};
	/**
	 * Default constructor.
	 * @param msg The error message.
	 * @param line The line # of the error.
	 * @param file The filename of the error occurrence.
	**/
	DkIndexOutOfBoundsException(const std::string & msg, int line, const std::string & file) : DkException(msg, line, file) {exceptionName = "DkIndexOutOfBoundsException";};
};
