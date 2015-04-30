/**************************************************
 * 	main.cpp
 *
 *	Created on:	16.08.2011
 * 	    Author:	Markus Diem
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkMSModule.h"

std::string helpText();

int main(int argc, char *argv[]) {

	if (argc != 3) {
		std::cout << "Wrong number of input arguments: " << argc-1 << " (2 expected)" << std::endl;
		std::cout << helpText();
		return 1;
	}

	try {

		std::string folderName(argv[1]);
	    std::replace(folderName.begin(), folderName.end(), '\\', '/');
		std::wstring folderNameW(folderName.begin(), folderName.end());
		std::string imageName(argv[2]);

		DkMSModule module(folderNameW);
		module.load();
		module.compute();
		module.saveImage(imageName);

	}
	catch(DkException iae) {
		printf("%s\n", iae.Msg().c_str());
		std::cout << helpText() << std::endl;
		return 1;
	}
	catch(cv::Exception cvex) {
		printf("Error in function %s, in file %s: msg %s\n", cvex.func.c_str(), cvex.file.c_str(), cvex.err.c_str());
		std::cout << helpText() << std::endl;
		return 2;
	}

	return 0;

}

std::string helpText() {

	std::string ht;
	ht += "Welcome to ViennaMS.\n\n";
	ht += "ViennaMS.exe <folder_name> <output_img_name>\n";
	ht += "  <folder_name> path to the MS folder.\n       The folder should contain 8 image files (FXXs.png) where XX is the channel number.\n";
	ht += "  <output_img_name> the output image name.\n\n";
	ht += "  Enjoy your day!\n";

	return ht;
}