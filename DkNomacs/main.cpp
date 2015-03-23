/**************************************************
 * 	main.cpp
 *
 *	Created on:	16.08.2011
 * 	    Author:	Markus Diem
 *				Stefan Fiel
 *				Angelika Garz
 * 				Florian Kleber
 *     Company:	Vienna University of Technology
 **************************************************/

#if _MSC_VER==1700
	#include "DkNoMacs.h"		// init app
#else
	#include "DkNoMacs.h"		// init app
#endif // _MSC_VER == 1700

#if defined(_MSC_BUILD) && !defined(QT_NO_DEBUG_OUTPUT) // fixes cmake bug - really release uses subsystem windows, debug and release subsystem console
#pragma comment (linker, "/SUBSYSTEM:CONSOLE")
#endif


//#include "opencv/cv.h"
#include "DkBaseBerlinger.h"
#include "DkBase.h"
#include "DkUtils.h"
#include "opencv2/nonfree/nonfree.hpp"

int main(int argc, char *argv[]) {

	cv::initModule_nonfree();

	QApplication a(argc, argv);

	// register our organization
	QCoreApplication::setOrganizationName("Computer Vision Lab");
	QCoreApplication::setOrganizationDomain("http://caa.tuwien.ac.at/cvl/");
	QCoreApplication::setApplicationName("DkSnippet");

	try {
		DkBaseBerlinger* b = new DkBaseBerlinger();

		b->init();
		b->preLoad();
		
		if (DkBase::show) {
			b->scriptNomacs();
			//b->start();	// start thread
			qApp->exec();	// just if show?!
			//b->stop();	// stop thread when gui is closed...
			//b->wait();	// wait for the thread to stop
			DkUtils::printDebug(DK_MODULE, "thread finished...\n");
		}
		else
			b->script();

		b->postLoad();

		//mm_deinit();
		delete b;

	}
	catch(DkException iae) {
		printf("Error...");
		printf("%s\n", iae.Msg().c_str());
		return 1;
	}
	//catch(cv::Exception cvex) {
	//	printf("Error in function %s, in file %s: msg %s\n", cvex.func.c_str(), cvex.file.c_str(), cvex.err.c_str());
	//	return 2;
	//}

	return 0;

}
