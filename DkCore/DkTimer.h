/*******************************************************************************************************
 DkTimer.h
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Markus Diem <markus@nomacs.org>
 Copyright (C) 2014-2015 Fabian Hollaus <holl@caa.tuwien.ac.at>
 Vienna University of Technology, Computer Vision Lab
 This file is part of ViennaMS.

 ViennaMS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 ViennaMS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *******************************************************************************************************/

#pragma once
#include <time.h>

//#include "DkCoreIncludes.h"

#include "DkError.h"
#include "DkUtils.h"

/**
 * A small class DK_CORE_API which measures the time.
 * This class DK_CORE_API is designed to measure the time of a method, especially
 * intervals and the total time can be measured.
 **/
class DkTimer {

public:

	/**
	 * Initializes the class DK_CORE_API and stops the clock.
	 **/
	DkTimer() {
		firstTick = clock();
		lastTick = firstTick;
	};

	/**
	 * Default destructor.
	 **/
	~DkTimer() {};

	friend std::ostream& operator<<(std::ostream& s, DkTimer& r){

		// this makes the operator<< virtual (stroustrup)
		return r.put(s);
	};

	/**
	 * Returns a string with the total time interval.
	 * The time interval is measured from the time,
	 * the object was initialized.
	 * @return the time in seconds or milliseconds.
	 **/
	std::string getTotal() {
		lastTick = clock();
		double ct = (double) (lastTick-firstTick) / CLOCKS_PER_SEC;

		return stringifyTime(ct);
	};

	virtual std::ostream& put(std::ostream& s) {

		lastTick = clock();
		double ct = (double) (lastTick-firstTick) / CLOCKS_PER_SEC;


		return s << stringifyTime(ct);
	};

	/**
	 * Returns a string with the time interval.
	 * The time interval since the last call of stop(), getIvl()
	 * or getTotal().
	 * @return the time in seconds or milliseconds.
	 **/
	std::string getIvl() {
		clock_t tmp = clock();
		double ct = (double) (tmp-lastTick) / CLOCKS_PER_SEC;
		lastTick = tmp;

		return stringifyTime(ct);
	};

	
	/**
	 * Converts time to std::string.
	 * @param ct current time interval
	 * @return std::string the time interval as string
	 **/ 
	std::string stringifyTime(double ct) {

		std::string msg = " ";

		if (ct < 1)
			msg += DkUtils::stringify(ct*1000) + " ms";
		else if (ct < 60)
			msg += DkUtils::stringify(ct) + " sec";
		else if (ct < 3600) {
			double m = cvFloor(ct/60.0f);
			msg += DkUtils::stringify(m) + " min " + DkUtils::stringify(ct-m*60, 0) + " sec";
		}
		else {
			double h = cvFloor(ct/3600.0f);
			msg += DkUtils::stringify(h) + " hours " + DkUtils::stringify((ct-h*3600.0f)/60.0f, 0) + " min";
		}

		return msg;

	};

	/**
	 * Stops the clock.
	 **/
	void stop() {
		lastTick = clock();
	};

	/**
	 * Returns the current time.
	 * @return double current time in seconds.
	 **/ 
	double static getTime() {
		return (double) clock();
	};

	double getTotalSec() {
		lastTick = clock();
		double ct = (double) (lastTick-firstTick) / CLOCKS_PER_SEC;

		return ct;
	}

protected:
	clock_t firstTick;	/**< the first tick**/
	clock_t	lastTick;	/**< the last tick**/

};




/**
 * A small class DK_CORE_API which measures the time.
 * This class DK_CORE_API is designed to measure the time of a method, especially
 * intervals and the total time can be measured.
 **/
class DkIvlTimer : public DkTimer {

public:

	/**
	 * Initializes the class DK_CORE_API and stops the clock.
	 **/
	DkIvlTimer() : DkTimer() {
		timeIvl = 0;
	};

	/**
	 * Default destructor.
	 **/
	~DkIvlTimer() {};

	/**
	 * Divides the time interval by the specified value.
	 * @param val the number of calls
	 **/ 
	void operator/= (const int &val) {

		timeIvl /= (clock_t)val;
	};


	/**
	 * Returns a string with the time interval.
	 * The time interval of all start() stop() calls.
	 * @return the time in seconds or milliseconds.
	 **/
	std::string getIvl() {
		
		double ct = (double) (timeIvl) / CLOCKS_PER_SEC;
		
		// return the interval in ms or sec depending on the interval's length
		return stringifyTime(ct);
	};

	/**
	 * Starts the clock.
	 **/ 
	void start() {
		lastTick = clock();
	};

	/**
	 * Stops the clock.
	 **/
	void stop() {
		clock_t cTime = clock();
		timeIvl += cTime-lastTick;
		lastTick = cTime;
	};

private:
	clock_t timeIvl;

};

#if defined(DK_DEBUG) && defined(WIN32)
#include <windows.h>
#include <stdio.h>
#include <psapi.h>
#pragma comment (lib, "Psapi.lib")

class DkMem {

public:
	DkMem() {
		firstMem = getCurMem();	// hopefully this does not throw an error
		lastMem = firstMem;
	};

	friend std::ostream& operator<<(std::ostream& s, DkMem& dm){

		// this makes the operator<< virtual (stroustrup)
		return dm.toString(s);
	};

	virtual std::ostream& toString(std::ostream& s) {

		return s << stringifyMemory(getDiffMemory()) << " [total: " << stringifyMemory(getMemory()) << "]";
	};

	std::string stringifyMemory(double mem) {

		std::string answer;
		if (abs(mem) < 0x400) {
			answer = DkUtils::stringify(mem, 2) + " bytes";
		}
		else if (abs(mem) < 0x100000) {
			answer = DkUtils::stringify(mem/0x400, 2) + " KB";
		}
		else if (abs(mem) < 0x40000000) {
			answer = DkUtils::stringify(mem/0x100000, 2) + " MB";
		}
		else if (abs(mem) < 0x10000000000) {
			answer = DkUtils::stringify(mem/0x40000000, 2) + " GB";
		}
		else
			answer = "> 999 GB man";

		return answer;

	};


	double getFirstMemory() {
		return firstMem;
	};
	
	double getDiffMemory() {
		
		double cm = getCurMem();
		double diff = cm-lastMem;
		lastMem = cm;

		return diff;
	};

	double getMemory() {

		return getCurMem();
	};


protected:
	
	double getCurMem() {

		HANDLE hProcess;
		PROCESS_MEMORY_COUNTERS pmc;

		hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, GetCurrentProcessId());

		if (hProcess == NULL)
			return 0;

		// returns false if it did not work... (check it?)
		GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc));
		CloseHandle(hProcess);

		return (double)pmc.WorkingSetSize;

	};
	
	double firstMem;
	double lastMem;
};

#else

class DkMem {

public:
	DkMem() {};

	friend std::ostream& operator<<(std::ostream& s, DkMem& dm){

		// this makes the operator<< virtual (stroustrup)
		return dm.toString(s);
	};

	virtual std::ostream& toString(std::ostream& s) {

		return s << "memory check not implemented";
	};

	double getFirstMemory() {
		return 0;
	};

	double getDiffMemory() {

		return 0;
	};

	double getMemory() {

		return 0;
	};

};

#endif