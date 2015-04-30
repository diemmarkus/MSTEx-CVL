/**************************************************
 * 	DkAttr.cpp
 *
 *	Created on:	21.03.2011
 * 	    Author:	Markus Diem
 *				Stefan Fiel
 *				Angelika Garz
 * 				Florian Kleber
 *     Company:	Vienna University of Technology
 **************************************************/

#include "DkAttr.h"


void DkXmlData::setFilename(std::string filename) {
	this->filename = filename;
}

std::string DkXmlData::toString() {


	std::string me = "filename: " + filename;
	me += "\n";

	return me;
}

DkXmlData* DkXmlData::setAttributes(std::string attr, std::map<std::string, std::string> &val, bool &isKnown) {

	//return false
	//attributes of DkXmlData are always set in the header...
	isKnown =  (val.empty() && !val.empty());

	return this;
}

void DkXmlData::setHeight(int height) {

	this->height = height;
}

void DkXmlData::setWidth(int width) {

	this->width = width;
}

void DkXmlData::getRootElement(std::string &attr, std::map<std::string, std::string> &val) {

	std::string str1 = "Page";
	std::map<std::string, std::string> valMap;
	std::stringstream i;

	valMap["imageFilename"] = this->filename;
	i << this->width;
	valMap["imageWidth"] = i.str();
	i.str("");
	i << this->height;
	valMap["imageHeight"] = i.str();

	attr = str1;
	val = valMap;

}

std::string DkXmlData::getKeyName() {
	return keyName;
}

DkXmlData* DkXmlData::getAttributes(std::string &attr, std::map<std::string, std::string> &val, int &depth) {

	std::string str1;
	depth = -1;
	std::map<std::string, std::string> valMap;

	attr = str1;
	val = valMap;

	return 0;
}


void DkXmlData::resetXmlCounter() {
	attrNumber = 0;
	cnt = 0;
	tmpChildSize = 0;
	childcnt.clear();
}

DkFontAttr::DkFontAttr(const DkFontAttr &a) : DkAttr(a), DkXmlData(a) {

	// font Attr
	this->fontSize = a.fontSize;
	this->attrAngleRad = a.attrAngleRad;
	this->fontAngleRad = a.fontAngleRad;
	this->accuracy = a.accuracy;
	this->classVoting = a.classVoting.clone();
	this->fontType = a.fontType;
	this->minAreaRot = (a.minAreaRot) ? new DkRect(*a.minAreaRot) : 0;
	this->minAreaCorners = (a.minAreaCorners) ? new DkRectCorners(*a.minAreaCorners) : 0;
	this->featureVec = a.featureVec.clone();
	this->featureDim = a.featureDim;
	this->childs = a.childs;
	this->color[0] = a.color[0];
	this->color[1] = a.color[1];
	this->color[2] = a.color[2];
	this->colorHLS = a.colorHLS;
	this->lineFreq = a.lineFreq;
	this->formatting = a.formatting;
	this->formAcc = a.formAcc;
	this->underlinedBy = a.underlinedBy;
	this->medianWordHeight = a.medianWordHeight;
	
	// debug visualization!
	this->upperPoints = a.upperPoints;
	this->lowerPoints = a.lowerPoints;
	this->text = a.text;
	
	this->paperType = a.paperType;
	this->bgColor = a.bgColor;
	this->cluster = a.cluster;

	this->xmlType = a.xmlType;
	this->filename = a.filename;
	this->width = a.width;
	this->height = a.height;

	this->attrNumber = a.attrNumber;
	this->cnt = a.cnt;
	this->childcnt = a.childcnt;
	this->tmpChildSize = a.tmpChildSize;

	this->text = a.text;
	this->sortVal = a.sortVal;
	this->numberLabel = a.numberLabel;
}

bool DkFontAttr::operator< (const DkFontAttr &attr) const {

	DkRect *mr, *ar;
	if (minAreaRot && attr.minAreaRot && !minAreaRot->isEmpty()) {
		mr = minAreaRot;
		ar = attr.minAreaRot;
	}
	else if (minAreaRect && attr.minAreaRect) {
		mr = minAreaRect;
		ar = attr.minAreaRect;
	}
	else
		return false;

	// if we know the bounding rectangle invariant to rotation -> use that one for sorting
	if (type == DK_TEXT_LINE_ATTR && fabs(mr->center.y - ar->center.y) > max(mr->size.height, ar->size.height)*0.5f ||
		fabs(mr->center.x - ar->center.x) < max(mr->size.width, ar->size.width)*0.5f)
			return mr ->center.y < ar->center.y;
	else 
		return mr->center.x < ar->center.x;
		
};

DkFontAttr* DkFontAttr::getAttributes(std::string &attr, std::map<std::string, std::string> &val, int &depth) {
	
	std::string str1;
	std::map<std::string, std::string> valMap;
	std::stringstream i;


	if (cnt == 0) {

		str1 = keyName;
		i << type;
		valMap["attrType"] = i.str();
		i.str("");
		
		if (area != -1)	 {
			i << area;
			valMap["area"] = i.str();
			i.str("");
		}
		if (orientation != -1.0f) {
			i << orientation;
			valMap["orientation"] = i.str();
			i.str("");
		}

		if (attrAngleRad != DBL_MIN) {
			i << attrAngleRad;
			valMap["attrOrientation"] = i.str();
			i.str("");
		} else if (minAreaRect) {
			i << minAreaRect->angle;
			valMap["attrOrientation"] = i.str();
			i.str("");
		}

		if (fontSize != -1) {
			i << fontSize;
			valMap["fontSize"] = i.str();
			i.str("");
		}
		if (numberLabel != -1) {
			i << numberLabel;
			valMap["numberLabel"] = i.str();
			i.str("");
		}

		if (fontAngleRad != DBL_MIN) {
			i << fontAngleRad;
			valMap["fontAngleRad"] = i.str();
			i.str("");
		}
		if (accuracy != 0) {
			i << accuracy;
			valMap["accuracy"] = i.str();
			i.str("");
		}
		if (!text.empty()) {
			valMap["text"] = text;
		}
		if (fontType != DK_NOT_CLASSIFIED) {
			i << fontType;
			valMap["fontType"] = i.str();
			i.str("");
		}
		if (paperType != DK_TEX_UNDEFINED) {
			i << paperType;
			valMap["paperType"] = i.str();
			i.str("");
		}

		if (color[0] != 0 && color[1] != 0 && color[2] != 0) {
			std::string col;
			i << color[0];
			col = i.str() + ","; i.str("");
			i << color[1];
			col += (i.str() + ","); i.str("");
			i << color[2];
			col += i.str();

			valMap["colorRGB"]  = col;
			i.str("");
		}//also color?

		if (!bgColor.isEmpty()) {
			std::string col;
			i << bgColor.r;
			col = i.str() + ","; i.str("");
			i << bgColor.g;
			col += (i.str() + ","); i.str("");
			i << bgColor.b;
			col += i.str();

			valMap["bgColor"]  = col;
			i.str("");
		}

		if (formatting!=0) {
			i << formatting;
			valMap["formatting"] = i.str();
			i.str("");
		}
		if (formAcc!=0) {
			i << formAcc;
			valMap["formAcc"] = i.str();
			i.str("");
		}
		if (medianWordHeight!=0) {
			i << medianWordHeight;
			valMap["medianWordHeight"] = i.str();
			i.str("");
		}
		if (rectType) {
			i << rectType;
			valMap["rectType"] = i.str();
			i.str("");
		}
		if (lineFreq!=0) {
			i << lineFreq;
			valMap["lineFreq"] = i.str();
			i.str("");
		}
		if (!classVoting.empty()) { 

			std::string cvs;
			i << getClassVotingAt(DK_NO_TEXT);
			cvs = i.str() + ","; i.str("");
			i << getClassVotingAt(DK_PRINT);
			cvs += (i.str() + ","); i.str("");
			i << getClassVotingAt(DK_MANUSCRIPT);
			cvs += i.str();

			valMap["classVoting"]  = cvs;
			i.str("");
		}
		if (!featureVec.empty()) {
			std::string fd;
			i << featureVec.cols;
			fd = i.str();
			valMap["featureDimX"] = fd;
			i.str("");

			i << featureVec.rows;
			fd = i.str();
			valMap["featureDimY"] = fd;
			i.str("");

			std::string featureString;
			float* fPtr = featureVec.ptr<float>();
			for (int cIdx = 0; cIdx < featureVec.cols*featureVec.rows; cIdx++) {
				i << fPtr[cIdx];
				featureString += i.str() + ",";
				i.str("");	// clear i
			}

			valMap["featureVec"] = featureString;
		}

		if (minAreaRect)
			cnt++;
		else if (bbRect)
			cnt=3;
		else { 
			cnt = 5;
			childcnt.push_back((int)childs.size());
			tmpChildSize = (int)childs.size();
		}
		
		if (minAreaRect || !bbRect)
			depth = XML_PUSH; 
		
	} else if (cnt==1) {
		str1 = "minAreaRect";
		childcnt.push_back(4);
		tmpChildSize = 4;
		cnt++;

		depth = XML_PUSH;
	} else if (cnt==2) {

		if (childcnt[(int)childcnt.size() - 1] - 1 >= 0) {
			int childIdx = tmpChildSize - childcnt[0];
			depth = XML_STAY;

			std::vector<DkVector> tmpVec = DkRectCorners(*minAreaRect).getCorners();
			str1 = "Point";
			i << tmpVec[childIdx].x;
			valMap["x"] = i.str();
			i.str(""); i << tmpVec[childIdx].y;
			valMap["y"] = i.str();

			childcnt[0] = childcnt[0] - 1;
		} else {
			depth = XML_POP;
			childcnt.pop_back();
			if (bbRect) cnt++;
			else {
				cnt = 5;
				childcnt.push_back((int)childs.size());
				tmpChildSize = (int)childs.size();
			}
		}
	} else if (cnt==3) {
		str1 = "BBRect";
		childcnt.push_back(2);
		tmpChildSize = 2;
		cnt++;
		
		depth = XML_PUSH;
	} else if (cnt==4) {
		if (childcnt[(int)childcnt.size() - 1] - 1 >= 0) {
			int childIdx = tmpChildSize - childcnt[0];
			depth = XML_STAY;

			if (childIdx==0) {
				str1 = "Point";
				i << bbRect->uc.x;
				valMap["x"] = i.str();
				i.str(""); i << bbRect->uc.y;
				valMap["y"] = i.str();

				childcnt[0] = childcnt[0] - 1;
			} else {
				str1 = "Point";
				i << bbRect->lc.x;
				valMap["x"] = i.str();
				i.str(""); i << bbRect->lc.y;
				valMap["y"] = i.str();

				childcnt[0] = childcnt[0] - 1;
			}
		} else {
			depth = XML_POP;
			childcnt.pop_back();
			cnt++;

			childcnt.push_back((int)childs.size());
			tmpChildSize = (int)childs.size();
		}
	} else if (cnt==5) {
		if (childcnt[0] - 1 >= 0) {
			int childIdx = tmpChildSize - childcnt[0];
			childcnt[0] = childcnt[0] - 1;

			return &childs[childIdx];
			childs[childIdx].getAttributes(str1, valMap, depth);
		} else {
			depth = XML_POP;
			childcnt.pop_back();
			cnt++;
		}
	}

	attr = str1;
	val = valMap;

	return 0;
}

DkFontAttr* DkFontAttr::setAttributes(std::string attr, std::map<std::string, std::string> &val, bool &isKnown) {
	isKnown = false;
	std::map<std::string, std::string>::iterator it;
	//std::stringstream i;
	DkFontAttr *ptr = 0;
	DkFontAttr tmp;


	if (attr.compare(keyName) == 0) {
		isKnown = true; 
		//check if the attributes of the current element are already set?
		if (empty()) ptr = this;
		else ptr = &tmp;

		for (it=val.begin(); it!=val.end(); it++) {
			if (it->first.compare("attrType") == 0) { std::stringstream i(it->second); i >>ptr->type; continue; }
			if (it->first.compare("attrOrientation") == 0) { std::stringstream i(it->second); i >>ptr->attrAngleRad; continue; }
			if (it->first.compare("area") == 0) { std::stringstream i(it->second); i >>ptr->area; continue; }
			if (it->first.compare("orientation") == 0) { std::stringstream i(it->second); i >> ptr->orientation; continue;}
			if (it->first.compare("fontSize") == 0) { std::stringstream i(it->second); i >> ptr->fontSize; continue;}
			if (it->first.compare("numberLabel") == 0) { std::stringstream i(it->second); i >> ptr->numberLabel; continue;}
			if (it->first.compare("fontAngleRad") == 0) { std::stringstream i(it->second); i >> ptr->fontAngleRad; continue;}
			if (it->first.compare("accuracy") == 0) { std::stringstream i(it->second); i >> ptr->accuracy; continue;}
			if (it->first.compare("fontType") == 0) { std::stringstream i(it->second); i >> ptr->fontType; continue;}
			if (it->first.compare("text") == 0) { std::stringstream i(it->second); i >> ptr->text; continue;}
			if (it->first.compare("colorRGB") == 0) {
				char placeholder;
				std::stringstream i(it->second);
				i >> ptr->color[0] >> placeholder >> ptr->color[1] >> placeholder >> ptr->color[2];
				ptr->colorHLS = DkIP::convertRGBtoIHLS(DkVector3((float)ptr->color[0], (float)ptr->color[1], (float)ptr->color[2]));
				continue;
			}
			if (it->first.compare("formatting") == 0) { std::stringstream i(it->second); i >> ptr->formatting; continue;}
			if (it->first.compare("formAcc") == 0) { std::stringstream i(it->second); i >> ptr->formAcc; continue;}
			if (it->first.compare("medianWordHeight") == 0) { std::stringstream i(it->second); i >> ptr->medianWordHeight; continue;}
			if (it->first.compare("rectType") == 0) { std::stringstream i(it->second); i >> ptr->rectType; continue;}
			if (it->first.compare("lineFreq") == 0) { std::stringstream i(it->second); i >> ptr->lineFreq; continue;}
			if (it->first.compare("paperType") == 0) { std::stringstream i(it->second); i >> ptr->paperType; continue;}
			if (it->first.compare("bgColor") == 0) { 
				char placeholder;
				std::stringstream i(it->second);
				i >> ptr->bgColor.r >> placeholder >> ptr->bgColor.g >> placeholder >> ptr->bgColor.b;
			}
			if (it->first.compare("classVoting") == 0) { 
				
				if (ptr->classVoting.empty())
					ptr->classVoting = Mat(1, 3, CV_32FC1);
				float* cvPtr = ptr->classVoting.ptr<float>();
				char placeholder;
				std::stringstream i(it->second);
				i >> cvPtr[DK_NO_TEXT] >> placeholder >> cvPtr[DK_PRINT] >> placeholder >> cvPtr[DK_MANUSCRIPT];
			}
			if (it->first.compare("featureDimX") == 0) {
				 std::stringstream i(it->second); 
				 i >> ptr->featureDim.x; 
			}
			if (it->first.compare("featureDimY") == 0) {
				std::stringstream i(it->second); 
				i >> ptr->featureDim.y; 
			}
			if (it->first.compare("featureVec") == 0) {
				
				if (!ptr->featureDim.isEmpty()) {
					
					ptr->featureVec = Mat(ptr->featureDim.getCvSize(), CV_32FC1);
					float* fPtr = ptr->featureVec.ptr<float>();
					
					std::stringstream i(it->second); 
					std::string val;	// that ain't save
					
					for (int cIdx = 0; cIdx < ptr->featureVec.cols*featureVec.rows; cIdx++) {
						std::getline(i, val, ',');

						if (val.empty()) {
							wout << "could not parse the feature vector correctly..." << dkendl;
							break;
						}

						fPtr[cIdx] = (float)atof(val.c_str());
					}
				}
			}
		}

		//check if AttrRegion is new Element
		if (ptr!=this) {
			this->childs.push_back(tmp);
			return &childs[childs.size()-1];
		} else return this;

	} else if (attr.compare("minAreaRect") == 0) {
		isKnown = true;
		if (minAreaRect) { delete minAreaRect; minAreaRect=0;}	//should not happen

		childcnt.push_back(0);
		childcnt.push_back(4);

		//tmp
		if (minAreaCorners) delete minAreaCorners; //should not happen
		minAreaCorners = new DkRectCorners();

	} else if (attr.compare("BBRect") == 0) {
		isKnown = true;
		if(bbRect) delete bbRect;
		bbRect = new DkBox();
		
		childcnt.push_back(0);
		childcnt.push_back(2);

	} else if (attr.compare("Point") == 0) {
		isKnown = true;
		float x=-1.0f;
		float y=-1.0f;
		for (it=val.begin(); it!=val.end(); it++) {
			if (it->first.compare("x") == 0) { std::stringstream i(it->second); i >> x;}
			if (it->first.compare("y") == 0) { std::stringstream i(it->second); i >> y;}
		}


		if (childcnt.size() == 2 && childcnt[1] == 4) {
			if (childcnt[0]==0) {
				minAreaCorners->c.x = x;
				minAreaCorners->c.y = y;
				childcnt[0]++;
			} else if (childcnt[0] == 1) {
				minAreaCorners->d.x = x;
				minAreaCorners->d.y = y;
				childcnt[0]++;
			} else if (childcnt[0] == 2) {
				minAreaCorners->a.x = x;
				minAreaCorners->a.y = y;
				childcnt[0]++;
			} else if (childcnt[0] == 3) {
				minAreaCorners->b.x = x;
				minAreaCorners->b.y = y;
				minAreaRect = new DkRect(minAreaCorners->getDkRect());
				delete minAreaCorners;
				minAreaCorners = 0;
				childcnt.pop_back();
				childcnt.pop_back();
			}
		} else if (childcnt.size() == 2 && childcnt[1] == 2) {
			if (childcnt[0] == 0) {
				bbRect->uc.x = x;
				bbRect->uc.y = y;
				childcnt[0]++;
			} else if (childcnt[0] == 1) {
				bbRect->lc.x = x;
				bbRect->lc.y = y;
				childcnt.pop_back();
				childcnt.pop_back();
			} 
		} else {
			wout << "WARNING: unknown state in DkFontAttr::setAttributes\n" << dkendl;
		}
	}

	return this;
}

void DkFontAttr::updateAttrs(DkFontAttr *parent, float weightParent) {

	if (parent != 0) {

		if (weightParent != FLT_MAX) {
			
			this->classVoting = (this->classVoting + (weightParent * parent->classVoting))/(weightParent+1.0f);

			double minC, maxC;
			Point minIdx, maxIdx;
			minMaxLoc(this->classVoting, &minC, &maxC, &minIdx, &maxIdx);

			// save the classification results
			this->fontType = maxIdx.x;
			this->accuracy = (float)maxC;
		}
		else {
			this->classVoting = parent->classVoting;
			this->fontType = parent->fontType;
			this->accuracy = parent->accuracy;
		}
	}

	// update my children
	for (unsigned int idx = 0; idx < childs.size(); idx++)
		childs[idx].updateAttrs(this, weightParent);

}

void DkFontAttr::computeAttrs(bool recursive, double angle) {

	// words must not be re-computed
	if (type == DK_FONT_ATTR)
		return;

	if (recursive) {
		// compute the attributes of my children
		for (unsigned int idx = 0; idx < childs.size(); idx++)
			childs[idx].computeAttrs(recursive, angle);
	}

	for (unsigned int idx = 0; idx < childs.size(); idx++) {
		this->rectType = childs[idx].rectType;
		if (childs[idx].rectType == DK_PROFILE_BOX)
			break;
	}

	float meanFontSize = 0;
	float numElements = 0;
	float col[] = {0,0,0};
	int sumArea = 0;
	float wSumArea = 0.0f;	// weighted sum
	double accCos = 0, accSin = 0;
	double rectCos = 0, rectSin = 0;
	float meanLineFreq = 0;
	float sumAcc = 0;

	Mat classVotingAcc = Mat(classVoting.size(), classVoting.type());
	classVotingAcc.setTo(0);

	vector<Point2f> corners, cornersT;
	vector<DkVector> upperCorners, lowerCorners;
 
	//mout << "I have: " << childs.size() << " children..." << dkendl;

	// accumulate all attributes
	for(unsigned int idx = 0; idx < childs.size(); idx++) {

		// rect types might be varying...
		vector<Point2f> cTmp = childs[idx].getMinAreaCorners()->getPoints32f();
		corners.insert(corners.end(), cTmp.begin(), cTmp.end());

		if (this->type == DK_TEXT_LINE_ATTR) {
			DkRectCorners* rc = childs[idx].getMinAreaCorners();
			
			DkVector leftP, rightP;
			rc->getUpperPoints(leftP, rightP, angle);
			DkLine l(leftP, rightP);
			l.sampleLine(upperCorners);
			
			rc->getLowerPoints(leftP, rightP, angle);
			l = DkLine(leftP, rightP);
			l.sampleLine(lowerCorners);
		}

		if (childs[idx].minAreaRot != 0) {
			cTmp = DkRectCorners(*childs[idx].getMinAreaRot()).getPoints32f();
			cornersT.insert(cornersT.end(), cTmp.begin(), cTmp.end());
		}

		// class voting
		Mat cvTmp = childs[idx].classVoting;
		
		//// ok, here we should know if the libsvm is loaded or not
		//float weight = childs[idx].accuracy;

		//float* cvTmpPtr = cvTmp.ptr<float>();

		//// >DIR: weighting positive and negativ weights differently improves the class voting - however it needs extensive testing... [12.11.2012 markus]
		//// TODO: the log function improves the results - however, we should additionally weight negative classes lower
		//// for voting, increase importance of positive weights
		//for (int cIdx = 0; cIdx < cvTmp.cols; cIdx++) {
		//	if (cvTmpPtr[cIdx] > 0) cvTmpPtr[cIdx] = (float)log(cvTmpPtr[cIdx]*(DK_EULER-1)+1);	// log [0 1] -> [0 1]
		//	if (cvTmpPtr[cIdx] < 0) cvTmpPtr[cIdx] = cvTmpPtr[cIdx] * 0.75f;
		//}

		if (classVotingAcc.empty())
			classVotingAcc = cvTmp * (float)childs[idx].area;
		else
			classVotingAcc += (cvTmp * (float)childs[idx].area /** weight*/);

		// font size
		meanFontSize += childs[idx].fontSize;
		meanLineFreq += childs[idx].lineFreq*(float)childs[idx].area;

		// font angle, rectangle angle
		accCos += cos(childs[idx].fontAngleRad)*childs[idx].area;
		accSin += sin(childs[idx].fontAngleRad)*childs[idx].area;

		// mean rectangle angle for line frequency
		if (type == DK_TEXT_BLOCK_ATTR) {
			rectCos += cos(childs[idx].getMinAreaRect().angle)*childs[idx].area;
			rectSin += sin(childs[idx].getMinAreaRect().angle)*childs[idx].area;
		}

		// font color
		col[0] += childs[idx].color[0]*childs[idx].area;
		col[1] += childs[idx].color[1]*childs[idx].area;
		col[2] += childs[idx].color[2]*childs[idx].area;

		sumAcc += childs[idx].accuracy*childs[idx].area;
		sumArea += childs[idx].area;
		wSumArea += childs[idx].area/**weight*/;
		numElements++;

	}

	this->upperPoints = upperCorners;
	this->lowerPoints = lowerCorners;

	// re-compute the minimum area rectangle -> it changes if the children were deleted
	if (!corners.empty()) {
		if (type < DK_TEXT_LINE_ATTR || childs.size() <= 1)
			this->setMinAreaRect(DkRect(cv::minAreaRect(Mat(corners))));	// set it, since areaRectCorners must be re-computed too
		else if (this->rectType == DK_PROFILE_BOX && type == DK_TEXT_LINE_ATTR) 
			this->setMinAreaRect(estimateBox(upperCorners, lowerCorners));
		else
			this->setMinAreaRect(computeMinBBox(corners));
	}
	if (!cornersT.empty()) {
		DkRect tmpRect;
		if (type < DK_TEXT_LINE_ATTR)
			tmpRect = DkRect(cv::minAreaRect(Mat(cornersT)));
		else if (this->rectType == DK_PROFILE_BOX && type == DK_TEXT_LINE_ATTR) 
			this->setMinAreaRect(estimateBox(upperCorners, lowerCorners));
		else
			tmpRect = computeMinBBox(cornersT, true);

		this->setMinAreaRot(tmpRect);
	}

	// avoid 0 divisions
	float sumAreaF = (wSumArea != 0) ? wSumArea : FLT_EPSILON;

	classVotingAcc /= sumAreaF;
	this->setClassVoting(classVotingAcc); // assigns new fontType, classVoting
	this->accuracy = sumAcc/sumAreaF;	// new accuracy is not the max accuracy anymore but the mean accuracy of all children

	// font size, angle & color
	fontSize = (numElements > 0) ? meanFontSize/numElements : 0;
	fontAngleRad = atan(accSin/(accCos+FLT_EPSILON));
	color[0] = col[0]/sumAreaF;
	color[1] = col[1]/sumAreaF;
	color[2] = col[2]/sumAreaF;
	colorHLS = DkIP::convertRGBtoIHLS(DkVector3(color[0], color[1], color[2]));

	area = sumArea;

	if (type != DK_FONT_ATTR) {

		// compute the median word height
		std::list<float> wHeights;

		for (unsigned int idx = 0; idx < childs.size(); idx++) {
			wHeights.push_back(childs[idx].getMinAreaRect().size.height);
		}
		medianWordHeight = (float)DkMath::statMoment(&wHeights, 0.5);
	}

	if (type == DK_TEXT_BLOCK_ATTR) {
		computeLineFrequencyAndFormatting();
	}
	else if (type == DK_GLOBAL_ATTR) {
		lineFreq = meanLineFreq/sumAreaF;
	}	
}

void DkFontAttr::computeLineFrequencyAndFormatting() {

	int n = 180;
	Mat orHist = Mat(1, n, CV_32FC1);
	orHist = Scalar(0);
	computeMeanAngle(&this->childs, orHist);
	double maxIdx = DkIP::findIplMaximumSymmetric(orHist);
	double meanAngle = maxIdx/(n-1)*CV_PI;

	double aDiffNorm = abs(DkMath::normAngleRad(minAreaRect->angle, 0, CV_PI) - meanAngle);
	double aDiffInv  = abs((CV_PI-DkMath::normAngleRad(minAreaRect->angle, 0, CV_PI)) - meanAngle);


	// compute the median word height
	std::list<float> wHeights;
	for (unsigned int idx = 0; idx < childs.size(); idx++) {
		wHeights.push_back(childs[idx].getMinAreaRect().size.height);
	}
	float medianWordHeight = (float)DkMath::statMoment(&wHeights, 0.5);

	vector<DkFontAttr>::iterator childCmpItr;
	vector<DkFontAttr> tmpChilds(childs); 
	for (unsigned int idx = 0; idx < tmpChilds.size(); idx++) {
		childCmpItr = tmpChilds.begin();
		DkFontAttr child = tmpChilds[idx];

		// does the rotated rect exist?
		if (!child.getMinAreaRot())
			return;

		// compare the current element to all others
		for(unsigned cmpIdx = 0; cmpIdx < tmpChilds.size(); cmpIdx++) {

			if (idx == cmpIdx) {
				childCmpItr++;
				continue;
			}

			DkFontAttr cmpChild = tmpChilds[cmpIdx];

			// if the difference of the centers of the two lines are within a 1/4 of the heights
			if (abs(child.getMinAreaRot()->center.y - cmpChild.getMinAreaRot()->center.y) < (child.getMinAreaRot()->size.height+cmpChild.getMinAreaRot()->size.height)/3) {

				vector<Point2f> cornersT = DkRectCorners(*child.getMinAreaRot()).getPoints32f();
				vector<Point2f> cTmpT = DkRectCorners(*cmpChild.getMinAreaRot()).getPoints32f();
				cornersT.insert(cornersT.end(), cTmpT.begin(), cTmpT.end());
				DkRect newRect(cv::minAreaRect(Mat(cornersT)));

				if (newRect.size.height > medianWordHeight*3.5) { // if new size is to large do not merge
					childCmpItr++; 
					continue;
				}

				child.setMinAreaRot(newRect);
				child.setArea(child.getArea()+cmpChild.getArea());

				//// debug (without transformation)
				//cornersT = child.getMinAreaCorners()->getPoints();
				//cTmpT = cmpChild.getMinAreaCorners()->getPoints();
				//cornersT.insert(cornersT.end(), cTmpT.begin(), cTmpT.end());
				//child.setMinAreaRect(DkRect(cv::minAreaRect(Mat(cornersT))));

				//child.childs[0].setMinAreaRect(child.getMinAreaRect());
				//child.childs[0].setArea(child.childs[0].getArea()+ cmpChild.childs[0].getArea());
				childCmpItr = tmpChilds.erase(childCmpItr);
				//(*otherChildItr).fontType = -1;
				if (cmpIdx < idx) idx -= 1;
				cmpIdx--;

			} else {
				childCmpItr++;
			}
		}
		tmpChilds[idx] = child;
	}
	if (min(aDiffNorm, aDiffInv) < CV_PI*0.25) {
		lineFreq = minAreaRect->size.height/(float)tmpChilds.size();			
	} else {
		lineFreq = minAreaRect->size.width/(float)tmpChilds.size();
	}

	if (tmpChilds.size() > 2) {
		vector<DkFontAttr>::iterator child;
		std::list<float> left;
		std::list<float> right;

		float meanLineArea = 0;
		for (child = tmpChilds.begin(); child != tmpChilds.end(); child++) {
			meanLineArea += (*child).getArea();
		}
		meanLineArea /= tmpChilds.size();
		for (child = tmpChilds.begin(); child != tmpChilds.end(); child++) {
			if ((*child).getArea() < meanLineArea/3)
				continue;
			DkRect* rect = (*child).getMinAreaRot();
			left.push_back(rect->center.x - rect->size.width/2);
			right.push_back(rect->center.x + rect->size.width/2);
		}

		double q25Left = DkMath::statMoment(&left,0.25,0);
		double q75Left = DkMath::statMoment(&left,0.75,0);


		double q25Right = DkMath::statMoment(&right,0.25,0);
		double q75Right = DkMath::statMoment(&right,0.75,0);


		double threshold = 15; // mean-2*standard deviation of difference of quantils for printed text
		if (fontType == DK_MANUSCRIPT) 
			threshold = 40;

		if (q75Right-q25Right > threshold && q75Left-q25Left > threshold) {
			formatting=DK_CENTERED;
			double accCentered = std::min((q75Left-q25Left),(q75Right-q25Right))/std::max((q75Left-q25Left),(q75Right-q25Right));
			formAcc=int(accCentered*100);			
		} else if (q75Right-q25Right > threshold) {
			formatting=DK_LEFT_ALIGNED;
			double accAligned = std::min(0.5/50*(q75Right-q25Right-threshold)+0.5,double(1)); // if q75Right-q25Right = threshold accuracy is 50%
			formAcc=int(accAligned*100);
		} else if (q75Left-q25Left > threshold) {
			formatting=DK_RIGHT_ALIGNED;
			double accAligned = std::min(0.5/50*(q75Left-q25Left-threshold)+0.5,double(1)); // if q75Left-q25Left = threshold accuracy is 50%
			formAcc=int(accAligned*100);
		} else {
			formatting=DK_JUSTIFIED;

			double accJustified = (std::min(1-0.5/5*(q75Left-q25Left-threshold)+0.5,double(1)) + std::min(1-0.5/5*(q75Right-q25Right-threshold)+0.5,double(1)))/2;
			formAcc=int(accJustified*100);
		}
		//childs=tmpChilds; // for debug image
	}
}

DkRect DkFontAttr::estimateBox(vector<DkVector> upperPoints, vector<DkVector> lowerPoints, double angle) {

	if (upperPoints.empty() && lowerPoints.empty()) {
		upperPoints = this->upperPoints;
		lowerPoints = this->lowerPoints;
	}

	if (upperPoints.empty() || lowerPoints.empty()) {
		wout << "cannot estimate box, if I don't have any points" << dkendl;
		return getMinAreaRect();
	}

	dout << "number of points: " << upperPoints.size() << dkendl;

	DkRect rect;
	rect.angle = angle;

	if (angle == DBL_MAX) {
		Mat upperMat((int)upperPoints.size(), 2, CV_32FC1);
		Mat lowerMat((int)lowerPoints.size(), 2, CV_32FC1);

		for (int rIdx = 0; rIdx < upperMat.rows; rIdx++) {
			float* ptrM = upperMat.ptr<float>(rIdx);

			ptrM[0] = upperPoints[rIdx].x;
			ptrM[1] = upperPoints[rIdx].y;
		}

		for (int rIdx = 0; rIdx < lowerMat.rows; rIdx++) {
			float* ptrM = lowerMat.ptr<float>(rIdx);

			ptrM[0] = lowerPoints[rIdx].x;
			ptrM[1] = lowerPoints[rIdx].y;
		}

		upperMat.push_back(lowerMat);

		rect.angle = DkMath::computePcaAngle(upperMat);
	}

	std::list<float> upperRotPointsX;
	std::list<float> upperRotPointsY;
	std::list<float> lowerRotPointsX;
	std::list<float> lowerRotPointsY;

	for (unsigned int idx = 0; idx < upperPoints.size(); idx++) {
		DkVector v = upperPoints[idx];
		v.rotate(rect.angle);
		upperRotPointsX.push_back(v.x);
		upperRotPointsY.push_back(v.y);
	}
	
	for (unsigned int idx = 0; idx < lowerPoints.size(); idx++) {
		DkVector v = lowerPoints[idx];
		v.rotate(rect.angle);
		lowerRotPointsX.push_back(v.x);
		lowerRotPointsY.push_back(v.y);
	}

	// compute median & sort values
	upperRotPointsX.sort();
	lowerRotPointsX.sort();

	float upperMedianY = (float)DkMath::statMoment(&upperRotPointsY, 0.5f);
	float lowerMedianY = (float)DkMath::statMoment(&lowerRotPointsY, 0.5f);

	float minX = (upperRotPointsX.front() + lowerRotPointsX.front())*0.5f;
	float maxX = (upperRotPointsX.back() + lowerRotPointsX.back())*0.5f;
	
	rect.size = DkVector(maxX-minX, lowerMedianY-upperMedianY);		// negative y coords
	DkVector center(rect.size.width*0.5f + minX, rect.size.height*0.5f+upperMedianY);
	center.rotate(-rect.angle);
	rect.center = center;
	
	// guarantee that width, height is always positive
	if (rect.size.height < 0)	rect.size.height *= -1.0f;
	if (rect.size.width < 0)	rect.size.width *= -1.0f;

	return rect;

}

DkRect DkFontAttr::computeMinBBox(vector<Point2f> points, bool minAreaT) {

	int n = 180;

	double orWord = 0.0;
	int rectType = DK_MIN_AREA_RECT;

	if (!childs.empty())
		rectType = childs[0].rectType;

	// TODO: profile box 
	if (rectType == DK_MIN_AREA_RECT || rectType == DK_PROFILE_BOX) {
		// compute the mean angle of all rectangles which are within the current one
		Mat orHist = Mat(1, n, CV_32FC1);
		orHist.setTo(0);

		// compatibility
		vector<DkFontAttr> attrs;
		attrs.push_back(*this);
		computeMeanAngle(&attrs, orHist, minAreaT);
		double maxIdx = DkIP::findIplMaximumSymmetric(orHist);
		orWord = maxIdx/n*CV_PI;
	}
	else if (rectType == DK_ROT_BBOX) {

		// I have children -> otherwise isMinAreaRect cannot be false
		orWord = childs[0].minAreaRect->angle;
	}

	// find min and max x,y coordinates
	DkVector ulc = DkVector(FLT_MAX, FLT_MAX);
	DkVector lrc = DkVector(-FLT_MAX, -FLT_MAX);

	for (unsigned int idx = 0; idx < points.size(); idx++) {
		DkVector p = DkVector(points[idx]);
		p.rotate(orWord);

		ulc = ulc.getMinVec(p);
		lrc = lrc.getMaxVec(p);
	}

	//// the old one
	//DkRectCorners bbRect = DkRectCorners(ulc, DkVector(lrc.x, ulc.y), lrc, DkVector(ulc.x, lrc.y));
	//bbRect.rotateAroundOrigin(-orWord);

	//return bbRect.getDkRect();

	// ought to be much more efficient...
	DkBox bbBox = DkBox(ulc, lrc-ulc);
	DkRect bbRect = DkRect(bbBox);
	bbRect.center.rotate(-orWord);
	bbRect.angle += orWord;

	return bbRect;
}

void DkFontAttr::computeMeanAngle(vector<DkFontAttr> *words, Mat& orHist, bool minAreaT) const {

	vector<DkFontAttr>::iterator wIter = words->begin();

	float* ptrOr = orHist.ptr<float>();

	while (wIter != words->end()) {

		DkFontAttr *w = &(*wIter);

		// skip my children
		if (w->type == DK_FONT_ATTR) {
			wIter++;		
			continue;
		}

		// first filter my children
		if (!w->childs.empty()) {
			computeMeanAngle(&w->childs, orHist);	// TODO: may cause a bug... (if minAreaT != default value?!)
		}

		if (w->type > DK_TEXT_LINE_ATTR) {
			wIter++;
			continue;
		}

		double angle = (!minAreaT || !w->getMinAreaRot()) ? w->getMinAreaRect().angle : w->getMinAreaRot()->angle;
		angle = DkMath::normAngleRad(angle, 0, CV_PI);
		int angleIdx = cvFloor(angle/CV_PI * orHist.cols) % orHist.cols;	// 180 becomes 0
		float angleWeight = (float)(angle/CV_PI * orHist.cols) - cvFloor(angle/CV_PI * orHist.cols);

		if (angleIdx < 0 || angleIdx >= orHist.cols) {
			DkUtils::printDebug(DK_WARNING, "[DkTextCluster] index out of bounds (mean angle estimation): %i\n", angleIdx);
			wIter++;
			continue;
		}

		// interpolate angle weight
		ptrOr[angleIdx] += w->area * (1.0f-angleWeight);
		ptrOr[(angleIdx+1) % orHist.cols] += w->area * angleWeight;
		wIter++;
	}
}


bool DkFontAttr::filter(DkFontAttr *fa, bool contains) {

	// if contains is true, filter all words which do not have the same fontType as fa
	if (fa->fontType != DK_NOT_CLASSIFIED && 
		(contains && fontType != fa->fontType ||
		!contains && fontType == fa->fontType)) {
			return true;
	}
	// if accuracy > 0 -> delete all attributes with a lower accuracy
	// if it is < 0 -> delete all attributes with a higher accuracy
	else if (fa->accuracy > 0 && accuracy < fa->accuracy ||
		fa->accuracy < 0 && accuracy > fabs(fa->accuracy)) {
			return true;
	}
	//else if (fontAngleRad != DBL_MIN &&
	//	(fa->fontAngleRad > 0 && fontAngleRad < fa->fontAngleRad ||
	//	fa->fontAngleRad < 0 && fontAngleRad > fabs(fa->fontAngleRad))) {
	//		return true;
	//}
	// 5° freedom
	else if (fa->minAreaRect != 0 && minAreaRect != 0 && 
		fabs(DkMath::normAngleRad(fa->minAreaRect->angle, -CV_PI*0.5, CV_PI*0.5) - 
		DkMath::normAngleRad(minAreaRect->angle, -CV_PI*0.5, CV_PI*0.5)) > fa->fontAngleRad &&
		minAreaRect->size.height/minAreaRect->size.width < 0.5) {	// angle filter only applies for non-square boxes
		return true;
	}
	else if (fa->minAreaRect != 0 && minAreaRect != 0 && 
		fa->minAreaRect->size.height/fa->minAreaRect->size.width < 
		minAreaRect->size.height/minAreaRect->size.width) {
		return true;
	}
	else if (fa->lineFreq != -1 &&
		(fa->lineFreq > 0 && lineFreq < fa->lineFreq ||
		fa->lineFreq < 0 && lineFreq > fabs(fa->lineFreq))) {
			return true;
	}
	else if (fa->fontSize != -1 &&
		(fa->fontSize > 0 && fontSize < fa->fontSize ||
		fa->fontSize < 0 && fontSize > fabs(fa->fontSize))) {
			return true;
	}
	else if (fa->area != -1 &&
		(fa->area > 0 && area < fa->area ||
		fa->area < 0 && area > abs(fa->area))) {
			return true;
	}
	else {
		if (fa->minAreaRect && minAreaRect)
			mout << "not filtered, filter angle: " << DK_RAD2DEG*fa->minAreaRect->angle << " my angle: " << DK_RAD2DEG*minAreaRect->angle << dkendl;
		return false;
	}

}

void DkFontAttr::computeTransformed(DkVector centerR, DkVector centerO, double angle) {

	// transform rotated rect
	DkVector cr(minAreaRect->center.x, minAreaRect->center.y);

	cr.x = centerR.x-cr.x;
	cr.y = cr.y-centerR.y;
	cr.rotate(angle);
	cr.x = centerO.x - cr.x;
	cr.y = centerO.y + cr.y;

	if (minAreaRot) delete minAreaRot;

	minAreaRot = new DkRect(*minAreaRect);

	minAreaRot->center.x = cr.x;
	minAreaRot->center.y = cr.y;
	minAreaRot->angle += (float)angle;
}

std::string DkFontAttr::toString() {

	std::string me = "";
	std::string tab = "";
	std::string dots = "";
	switch (type) {
	case DK_FONT_ATTR:
		tab = "      ";
		me += tab + "WORD";
		dots = "------";
		break;
	case DK_TEXT_LINE_ATTR:
		tab = "    ";
		me += tab + "LINE BLOCK";
		break;
	case DK_TEXT_BLOCK_ATTR:
		tab = "  ";
		me += tab + "TEXT BLOCK";
		break;
	case DK_GLOBAL_ATTR:
		tab = " ";
		me += tab + "GLOBAL ATTRIBUTES";
		break;
	}

	switch (fontType) {
	case DK_PRINT:
		me += " (print: ";
		dots += "---------------------";
		break;
	case DK_MANUSCRIPT:
		me += " (manuscript: ";
		dots += "----------------";
		break;
	case DK_NO_TEXT:
		me += " (no text: ";
		dots += "------------------";
		break;
	default:
		me += " (UNKNOWN: " + DkUtils::stringify(fontType) + " ";
		dots += "-----------------";
	}

	me += DkUtils::stringify(accuracy*100.0f, 1) + "%) ";
	if (minAreaRect != 0)
		me + dots + minAreaRect->center.toString();
	me += "\n";
	if (minAreaRect == 0)
		me += tab + "font angle: " + DkUtils::stringify(fontAngleRad*DK_RAD2DEG, 2);
	else
		me += tab + minAreaRect->toString();
	me +=		" | size: " + DkUtils::stringify(fontSize, 2);
	me +=		" | area: " + DkUtils::stringify(area);
	me +=		" | color: " +	DkUtils::stringify(color[0], 0) + " " +
		DkUtils::stringify(color[1], 0) + " " +
		DkUtils::stringify(color[2], 0);

	// maybe unsave -> 2 classes
	try {

		if (!classVoting.empty()) {
			float* wPtr = classVoting.ptr<float>();
			me +=	" | voting: (" + DkUtils::stringify(wPtr[DK_NO_TEXT], 3) + " " +
				DkUtils::stringify(wPtr[DK_PRINT], 3) + " " +
				DkUtils::stringify(wPtr[DK_MANUSCRIPT], 3) + ")\n";
		}
	}
	catch(...) {
		me += "\n";
	}

	if (type == DK_TEXT_BLOCK_ATTR || type == DK_GLOBAL_ATTR)
		me += tab + "line frequency: " + DkUtils::stringify(lineFreq) + " formatting: " + DkUtils::stringify(formatting) + " formatting acc: " + DkUtils::stringify(formAcc) + "\n";

	me += tab + "\n";
	//me += getMinAreaRect().toString() + "\n";

	return me;
}

