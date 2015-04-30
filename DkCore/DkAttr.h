/*******************************************************************************************************
 DkAttr.h
 Created on:	30.04.2015
 
 MSTEx is a binarization software for multi-spectral images.
 It was submitted to the MultiSpectral Text Extraction Contest
 @ ICDAR 2015 (http://www.synchromedia.ca/competition/ICDAR/mstexicdar2015.html)
 
 Copyright (C) 2014-2015 Markus Diem <markus@nomacs.org>
 Copyright (C) 2014-2015 Fabian Hollaus <holl@caa.tuwien.ac.at>
 Copyright (C) 2014-2015 Florian Kleber <kleber@caa.tuwien.ac.at>
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

#include <list>


#include "DkMath.h"
#include "DkIP.h"



enum text_classes{DK_NOT_CLASSIFIED = -1, DK_NO_TEXT, DK_PRINT, DK_MANUSCRIPT, DK_SIGNATURE, DK_IMAGE, DK_OTHER};
enum elem_type{DK_ATTR=0, DK_FONT_ATTR, DK_TEXT_LINE_ATTR, DK_TEXT_BLOCK_ATTR, DK_GLOBAL_ATTR};
enum form_type{DK_FORM_UNDEFINED=0, DK_JUSTIFIED, DK_RIGHT_ALIGNED, DK_LEFT_ALIGNED, DK_CENTERED};
enum train_type{DK_DATA = 0, DK_GT, DK_TEX, DK_TXT, DK_PAGE, DK_ROT, DK_FORM};
enum paper_type{DK_TEX_UNDEFINED = -1, DK_VOID, DK_LINED, DK_CHECKED};

enum xml_depth{XML_POP = -1, XML_STAY = 0, XML_PUSH = 1};

class DkXmlData {

protected:

	std::string keyName;	/**< is the name of the basic element*/
	std::string filename;	/**< the filename of the current entry*/
	int width;				/**< width dimension of the image*/
	int height;				/**< height dimension of the image*/
	int xmlType;				/**< the type @deprecated.*/

	int attrNumber;			/**< the number of attributes. needed for the xml-parser.*/
	int cnt;				/**< current attribute number which is executed by getAttributes*/

	/**< size of the vector gives number of childs of current element.*/
	/**< the value is the number of elements of the child.*/
	/**< e.g. Textregion has one child (Coords) with 1 element (Coords). Coords has one child with n points*/
	std::vector<int> childcnt;
	int tmpChildSize;

public:

	DkXmlData() {
		//this->filename		= "";
		width = 0;
		height = 0;
		this->xmlType			= DK_DATA;

		attrNumber = 0;		//no attributes to write since filename, width and height is in the page header
		cnt = 0;
		childcnt.clear();
		tmpChildSize = 0;
	};
	/**
	 * Default Constructor
	 **/ 
	DkXmlData(std::string filename) {
		this->filename		= filename;
		width = 0;
		height = 0;
		this->xmlType			= DK_DATA;

		attrNumber = 0;		//no attributes to write since filename, width and height is in the page header
		cnt = 0;
		childcnt.clear();
		tmpChildSize = 0;
	};

	/** 
	 * Default destructor.
	 **/
	virtual ~DkXmlData() {};

	/**
	 * Checks whether the current instance has the same filename as str.
	 * This method is particularly useful if an instance needs to be found
	 * within a container (std::list or std::vector).
	 * @param str a string with a given filename.
	 * @return true if the filename is the same as str.
	 **/
	virtual bool operator== (const std::string &str) {
		return (this->filename.compare(str) == 0) ? true : false;
	};
	/**
	 * Checks whether two instances are the same or not.
	 * This method is particularly useful if an instance needs to be found
	 * within a container (std::list or std::vector).
	 * @param o another instance.
	 * @return true if the filename of both instances are the same.
	 **/
	virtual bool operator== (const DkXmlData &o) {
		printf("comparing: %s to %s\n", this->filename.c_str(), o.getFilename().c_str());
		return (this->filename.compare(o.getFilename()) == 0) ? true : false;
	};

	friend std::ostream& operator<<(std::ostream& s, DkXmlData& r){

		// this makes the operator<< virtual (stroustrup)
		return s << r.toString();
	};

	/**
	 * Returns true if the default constructor was called.
	 * @return true if the default constructor was called.
	 **/
	virtual bool empty() const {
		return filename.empty();	// no filename assigned -> default constructor
	};

	/**
	 * Sets the Attributes of the class. Called by DkBaseXmlParser.
	 * @param attr the elements name.
	 * @param val the attribute names and its values.
	 * @return true if attr is a known attribute of the class.
	 **/
	virtual DkXmlData* setAttributes(std::string attr, std::map<std::string, std::string> &val, bool &isKnown);
	/**
	 * Called by XmlParser to write the class as xml.
	 * attrNr is the current element, attr is the element name, and val are the attribute names
	 * and values.
	 * @param attr the elements name.
	 * @param val the attribute names and its values.
	 * @return number of children.
	 **/
	virtual DkXmlData* getAttributes(std::string &attr, std::map<std::string, std::string> &val, int &depth);

	virtual void getRootElement(std::string &attr, std::map<std::string, std::string> &val);
	
	virtual std::string getKeyName();
	
	/**
	 * Returns the number of xml attributes in the class.
	 * @return number of xml attributes.
	 **/
	virtual void resetXmlCounter();

	void setFilename(std::string filename);
	/**
	 * Returns the class type.
	 * @return the type.
	 **/
	/**
	 * Assign the image width
	 * @param width the image width.
	 **/
	void setWidth(int width);
	/**
	 * Assign the image height
	 * @param height the image height.
	 **/
	void setHeight(int height);


	/**
	 * The object's image width.
	 * @return width of the image in px.
	 **/
	int getWidth() const			{return this->width;};

	/**
	 * The object's image height.
	 * @return height of the image in px.
	 **/
	int getHeight() const			{return this->height;};

	virtual int getXmlType() {
		return xmlType;
	};

	virtual std::string toString();

	/**
	 * The object's filename.
	 * @return a string with the current filename.
	 **/
	std::string getFilename() const	{return this->filename;};
};






/**
 * The DkAttr class is used for labeled binary images and holds the attributes for each blob.
 * Its the base class for the labeling class DkBlobs containing the area, orientation, etc.
 * To save memory the bounding box, as well as the minimum area rectangle are allocated
 * on demand.
 **/
class DkAttr {
public:
	
	enum {
		DK_MIN_AREA_RECT = 0,
		DK_ROT_BBOX,
		DK_PROFILE_BOX,
	};

	int type;		/**< type of the attribute, like text, font, block, etc. **/
	int rectType;	/**< saves the method that computed the rectangles.**/

	/**
	 * Default constructor.
	**/
	DkAttr() : rectType(DK_MIN_AREA_RECT), type(DK_ATTR), area(-1), orientation(-1.0f), bbRect(0), minAreaRect(0), contour(0), contourIdx(-1) {};

	/**
	 * Default copy constructor.
	**/
	DkAttr(const DkAttr &a) {

		this->area = a.area;
		this->contourIdx = a.contourIdx;
		this->orientation = a.orientation;
		this->bbRect = (a.bbRect) ? new DkBox(*a.bbRect) : 0;
		this->minAreaRect = (a.minAreaRect) ? new DkRect(*a.minAreaRect) : 0;
		this->contour = (a.contour) ? new vector<Point>(*a.contour) : 0;
		this->type = a.type;
		this->rectType = a.rectType;
	};


	/**
	 * Default destructor.
	 **/
	virtual ~DkAttr() { 
		if (bbRect) delete bbRect; bbRect = 0; 
		if (minAreaRect) delete minAreaRect; minAreaRect = 0;
		if (contour) delete contour; contour = 0;
	};
	
	/**
	 * Releases the rectangles (bbRect, minAreaRect)
	 **/
	virtual void clear() {
		if (bbRect) delete bbRect; bbRect = 0; 
		if (minAreaRect) delete minAreaRect; minAreaRect = 0;
		if (contour) delete contour; contour = 0;
	};

	/**
	 * Default assignment operator.
	 **/
	DkAttr& operator= (const DkAttr& a) {
		
		if (this == &a)
			return *this;

		this->clear();

		this->area = a.area;
		this->orientation = a.orientation;
		this->contourIdx = a.contourIdx;
		this->bbRect = (a.bbRect) ? new DkBox(*a.bbRect) : 0;
		this->minAreaRect = (a.minAreaRect) ? new DkRect(*a.minAreaRect) : 0;
		this->contour = (a.contour) ? new vector<Point>(*a.contour) : 0;
		this->type = a.type;
		this->rectType = a.rectType;
		
		return *this;
	}

	bool operator< (const DkAttr &attr) const {

		if (fabs(minAreaRect->center.y - attr.minAreaRect->center.y) > max(minAreaRect->size.height, attr.minAreaRect->size.height)*0.5f)
			return minAreaRect->center.y < attr.minAreaRect->center.y;
		else
			return minAreaRect->center.x < attr.minAreaRect->center.x;
	};

	static bool sortAttrX(DkAttr& attr1, DkAttr& attr2) {
		
		return attr1.getMinAreaRect().center.x < attr2.getMinAreaRect().center.x;
	};


	/** 
	 * Sets the area property of the blob.
	 * @param area area of the property in pixel
	 */
	void setArea(int area) {this->area = area;};

	/** 
	 * Sets the moment based orientation property of the blob.
	 * @param o orientation of the property in degree
	 */
	void setOrientation(float o) {this->orientation = o;};

	/** 
	 * Sets the bounding box of a blob.
	 * @param rect Bounding Box of a blob as OpenCv Rect 
	 */
	void setBbRect(DkBox rect) {

		if (bbRect != 0) delete bbRect;
		bbRect = new DkBox(rect);
	};

	/** 
	 * Sets the minimum area rectangle of the blob.
	 * @param r The minimum area rectangle of the blob as OpenCV RotatedRect
	 */
	virtual void setMinAreaRect(DkRect r) {
		
		if (minAreaRect !=  0) delete minAreaRect;
		minAreaRect = new DkRect(r);
	};
	/** 
	 * Sets the contour index of the current blob. This allows the DkBlobs class to reference
	 * the blob contour and to draw the blob in an image (see OpenCV findContours).
	 * @param idx Contour index of the blob
	 */
	void setContourIdx(int idx) {contourIdx = idx;};

	void setContour(vector<Point> newContour) {

		if (contour != 0) delete contour;
		// TODO: compress here...
		contour = new vector<Point>(newContour);
	};

	/** 
	 * Allows to check if the minimum area rectangle has been calculated.
	 * @return value is 0 if the min area rectangle has not yet been calculated, 1 otherwise
	 */
	int calcMinAreaRect() { if (minAreaRect) return 1; else return 0;};
	/** 
	 * Allows to check if the bounding box has been calculated.
	 * @return value is 0 if the bounding box has not yet been calculated, 1 otherwise
	 */
	int calcBbRect() {return (bbRect != 0);};
	/** 
	 * Return the area value in pixel of the blob.
	 * @return Area in pixel
	 */
	int getArea() const {return area;};
	/** 
	 * Return the moment based orientation of the blob.
	 * @return Orientation in degree
	 */
	float getOrientation() {return orientation;};
	/** 
	 * Return the contour index of the blob.
	 * @return contour index (see OpenCV findContours)
	 */
	int getContourIdx() {return contourIdx;};

	/** 
	 * Return the outer contour of the blob or 0 if no contour was computed.
	 * @return contour (see OpenCV findContours)
	 */
	vector<Point>* getContour() {
		return contour;
	};

	/** 
	 * Return the bounding box of the blob.
	 * @return bounding Box.
	 */
	DkBox getBbRect() {
		
		if (!bbRect && minAreaRect) {
			DkRectCorners rc = DkRectCorners(*minAreaRect);
			bbRect = new DkBox(rc);
		}
		else if (!bbRect)
			bbRect = new DkBox();
		
		return *bbRect;
	};

	/**
	 * Returns an extended bounding box.
	 * The bounding box is expanded in all directions by
	 * the factor offset/2. Afterwards it is clipped to
	 * the image size.
	 * @param imgSize the image's size.
	 * @param offset the expanding offset.
	 * @return the expanded and clipped bounding box.
	 **/
	DkBox getExpandedBbRect(Size imgSize, float offset) {
		
		DkBox boxExp = DkBox(*bbRect);
		boxExp.expand(offset);
		boxExp.clip(DkVector(imgSize));

		return boxExp;
	};

	/** 
	 * Return the Minimum Area Rectangle of the blob.
	 * @return minimum area rectangle as OpenCV RotatedRect
	 */
	DkRect getMinAreaRect() {return (*minAreaRect);};

protected:
	int area;						/**< blob area **/
	float orientation;				/**< moment based orientation in rad **/
	DkBox *bbRect;					/**< bounding box **/
	DkRect *minAreaRect;			/**< minimum area rectangle **/
	vector<Point> *contour;			/**< the blob's contour**/ // >DIR:  [2.2.2012 kleber]
	int contourIdx;					//Contour Index // >DIR:  [2.2.2012 kleber]

};


/**
 * The DkColorAttr class is an extended class of DkAttr. In addition to the blob properties
 * of DkAttr the Mean Color of each blob is an additional attribute.
 **/
class DkColorAttr : public DkAttr {

protected:
	DkVector3 lab;

public:
	/**
	 * Default constructor.
	**/
	DkColorAttr() : DkAttr(), r(0), g(0), b(0), h(-1), l(-1), s(-1), center() {};
	/**
	 * Default destructor.
	**/
	virtual ~DkColorAttr() {};

	//DkColorAttr& DkColorAttr::operator= (const DkColorAttr& a) {
	//	if (this == &a)
	//		return *this;
	//	DkAttr::operator= (a);
	//	this->r = a.r;
	//	this->g = a.g;
	//	this->b = a.b;
	//	this->stdR = a.stdR;
	//	this->stdG = a.stdG;
	//	this->stdB = a.stdB;
	//	return *this;
	//}

	/** 
	 * Sets the mean color of the blob [0 255].
	 * @param red Red component of the color
	 * @param green Green component of the color
	 * @param blue Blue component of the color
	 */
	void setRGB(unsigned char red, unsigned char green, unsigned char blue) {r = red; g = green; b = blue;};
	/** 
	 * Sets the standard deviation of the mean color of the blob.
	 * @param r Standard Dev of the R component of the color
	 * @param g Standard Dev of the G component of the color
	 * @param b Standard Dev of the B component of the color
	 */
	void setStdRGB(unsigned char r, unsigned char g, unsigned char b) {stdR = r; stdG = g; stdB = b;};

	DkVector3 getLabVector() {
		
		if (lab.isEmpty()) {
			lab = DkVector3(r, g, b);
			lab = DkIP::convertRGBtoLab(lab);
		}

		return lab;
	}

	unsigned char r;		/**< mean r value [0 255] **/
	unsigned char g;		/**< mean g value [0 255] **/
	unsigned char b;		/**< mean b value [0 255] **/

	float h;				/**< mean h value **/
	float l;				/**< mean l value **/
	float s;				/**< mean s value **/

	DkVector center;		/**< center of the bounding box **/

	unsigned char stdR;		/**< standard deviation of R **/
	unsigned char stdG;		/**< standard deviation of G **/
	unsigned char stdB;		/**< standard deviation of B **/
};

/**
 * The DkFontAttr class is an extended class of DkAttr. In addition to the blob properties
 * of DkAttr the font attributes like size, type, color, etc. are stored for each blob.
 **/
class DkFontAttr : public DkAttr, public DkXmlData {

protected:

	DkRect *minAreaRot;				/**< the rotated minimum area rectangle**/ // >DIR:  [2.2.2012 kleber]
	DkRectCorners *minAreaCorners;	/**< The minimum area rectangle, where each corner is stored.**/ // >DIR:  [2.2.2012 kleber]
									/**< is also used as tmp variable for minAreaRect in setAttributes (xml read)**/
	Mat classVoting;				/**< Class voting**/

	DkVector featureDim;			/**< number of "character" features**/ // >DIR:  [2.2.2012 kleber]

	// font
	Mat featureVec;			/**< Shape context features, or gradient feature.**/ // >DIR:  [2.2.2012 kleber]


public:

	// all attributes	
	int numberLabel;			/**< class label of digit recognition TODO: make own class **/
	double sortVal;			/**< Sort value **/
	float fontSize;			/**< Word's font size.**/
	double fontAngleRad;	/**< Slant angle.**/
	float accuracy;			/**< Classification accuracy.**/
	int fontType;			/**< Word's class (e.g. print, manuscript).**/
	float color[3];			/**< Word's color.**/
	DkVector3 colorHLS;		/**< iHLS colors.**/
	int formatting;			/**< Text Block's formatting (eg right-aligned, justified)**/
	int formAcc;			/**< Accuracy of formatting**/
	double attrAngleRad;	/**< alternative angle - needed for global attribute (bei unklarheit markus fragen :)**/
	std::string text;

	int paperType;			/**< paper type (DkTexture)**/
	DkVector3 bgColor;		/**< background color**/
	int cluster;			/**< cluster ID**/

	std::vector<DkLineExt> underlinedBy;						/**< is the word underlined**/
	std::vector<DkLineDotted> underlinedByDottedLine;			/**< is the word underlined**/
	std::vector<DkVector> upperPoints;
	std::vector<DkVector> lowerPoints;

	float medianWordHeight; /**< median word height; type > DK_TEXT_LINE **/
	
	// text block, text line
	vector<DkFontAttr> childs;	/**< The element's children (e.g. text lines or words).**/

	// text line
	float lineFreq;			/**< Line frequency of a TextBlock**/

	/**
	 * Default constructor.
	 **/
	DkFontAttr() 
		: DkAttr(), DkXmlData(), 
		minAreaRot(0),
		attrAngleRad(DBL_MIN),
		minAreaCorners(0), 
		featureVec(Mat()), 
		fontSize(-1), 
		fontAngleRad(DBL_MIN), 
		accuracy(0), 
		fontType(DK_NOT_CLASSIFIED), 
		colorHLS(DkVector3()),
		formatting(0),
		formAcc(0),
		paperType(DK_TEX_UNDEFINED),
		bgColor(DkVector3()),
		cluster(-1),
		medianWordHeight(0),
		lineFreq(-1),
		sortVal(0),
		numberLabel(-1),
		text("")
		{
			
			//classVoting = Mat(1, DK_MANUSCRIPT+1, CV_32FC1);
			classVoting.setTo(0);
			color[0] = 0;
			color[1] = 0;
			color[2] = 0;
			this->type = DK_FONT_ATTR;
			underlinedBy = std::vector<DkLineExt>();
			underlinedByDottedLine = std::vector<DkLineDotted>();
			keyName = "AttrRegion";
	};

	DkFontAttr(const DkDescriptor &desc) : DkAttr(), DkXmlData(), 
			minAreaRot(0),
			attrAngleRad(DBL_MIN),
			minAreaCorners(0), 
			fontSize(-1), 
			fontAngleRad(DBL_MIN), 
			accuracy(0), 
			colorHLS(DkVector3()),
			formatting(0),
			formAcc(0),
			paperType(DK_TEX_UNDEFINED),
			bgColor(DkVector3()),
			cluster(-1),
			medianWordHeight(0),
			lineFreq(-1),
			sortVal(0),
			numberLabel(-1),
			text("") {

		this->fontType = desc.fontType;
		setFeatureVec(desc.feature);

		DkBox b = desc.getBBox();
		setMinAreaRect(DkRect(b));
		setBbRect(b);
		keyName = "AttrRegion";
	};

	/**
	 * Copy constructor.
	 **/
	DkFontAttr(const DkFontAttr &a);

	/**
	 * Default assignment operator.
	 **/
	DkFontAttr& operator= (const DkFontAttr& a) {
		
		if (this == &a)
			return *this;

		this->clear();

		DkAttr::operator= (a);
		DkXmlData::operator= (a);

		// font attrs
		this->fontSize = a.fontSize;
		this->fontAngleRad = a.fontAngleRad;
		this->accuracy = a.accuracy;
		this->classVoting = a.classVoting.clone();
		this->fontType = a.fontType;
		this->attrAngleRad = a.attrAngleRad;

		this->minAreaCorners = (a.minAreaCorners) ? new DkRectCorners(*a.minAreaCorners) : 0;
		this->minAreaRot = (a.minAreaRot) ? new DkRect(*a.minAreaRot) : 0;
		
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
		this->paperType = a.paperType;
		this->bgColor = a.bgColor;
		this->cluster = a.cluster;
				
		this->underlinedBy = a.underlinedBy;
		this->underlinedByDottedLine = a.underlinedByDottedLine;

		this->medianWordHeight = a.medianWordHeight;

		this->xmlType = a.xmlType;
		this->filename = a.filename;
		this->width = a.width;
		this->height = a.height;

		this->upperPoints = a.upperPoints;
		this->lowerPoints = a.lowerPoints;

		this->attrNumber = a.attrNumber;
		this->cnt = a.cnt;
		this->childcnt = a.childcnt;
		this->tmpChildSize = a.tmpChildSize;
		this->text = a.text;
		this->sortVal = a.sortVal;
		this->numberLabel = a.numberLabel;


		return *this;
	}

	/**
	 * Default destructor.
	 **/
	virtual ~DkFontAttr() {
		if (minAreaCorners) delete minAreaCorners; minAreaCorners = 0;
		if (minAreaRot) delete minAreaRot; minAreaRot = 0;
	};

	friend std::ostream& operator<<(std::ostream& s, DkFontAttr& r){

		// this makes the operator<< virtual (stroustrup)
		return r.put(s);
	};

	virtual std::ostream& put(std::ostream& s) {

		return s << toString();
	};


	bool operator< (const DkFontAttr &attr) const;

	virtual bool empty() {
		return fontType==DK_NOT_CLASSIFIED;
	}


	/**
	 * Releases all rectangles (e.g. bbRect, minAreaRect, minAreaCorners).
	 **/
	void clear() {
		DkAttr::clear();
		if (minAreaCorners) delete minAreaCorners; minAreaCorners = 0;
		if (minAreaRot) delete minAreaRot; minAreaRot = 0;
	}


	inline virtual DkFontAttr* getAttributes(std::string &attr, std::map<std::string, std::string> &val, int &depth);
	inline virtual DkFontAttr* setAttributes(std::string attr, std::map<std::string, std::string> &val, bool &isKnown);

	/**
	 * Top-down propagation of class voting.
	 * @param parent the attribute's parent (or 0 if it is the root)
	 * @param weightParent the parent's weight (if FLT_MAX solely the class voting of the parent is considered)
	 **/ 
	void updateAttrs(DkFontAttr *parent = 0, float weightParent = 1.0f);

	/**
	 * Computes the text element's attributes.
	 * The computation is based on an element's children.
	 **/
	void computeAttrs(bool recursive = true, double angle = 0.0);

	/**
	 * Computes the line Frequency and the formatting of all TEXT_BLOCKS
	 * line blobs are clustered if they have similar y coords also to get a better line number count
	 **/
	void computeLineFrequencyAndFormatting();

	DkRect estimateBox(vector<DkVector> upperPoints, vector<DkVector> lowerPoints, double angle = DBL_MAX);
	DkRect computeMinBBox(vector<Point2f> points, bool minAreaT = false);
	void computeMeanAngle(vector<DkFontAttr> *words, Mat& orHist, bool minAreaT = false) const;
	
	bool filter(DkFontAttr *fa, bool contains = true);

	/**
	 * Transforms the element's rectangles and stores them to minAreaRot.
	 * This method rotates the element.
	 * @param centerR the rotation center of rotated image
	 * @param centerO the rotation center of the input image
	 * @param angle the rotation angle
	 **/
	void computeTransformed(DkVector centerR, DkVector centerO, double angle);

	/**
	 * Converts the attribute's fields to a string.
	 * @return a string containing all fields of an attribute.
	 **/
	std::string toString();

	/** 
	 * Sets the minimum area rectangle of the blob.
	 * It also re-computes the minAreaCorners.
	 * @param r The minimum area rectangle of the blob as OpenCV RotatedRect
	 */
	void setMinAreaRect(DkRect r) {
		
		if (minAreaRect !=  0) delete minAreaRect;
		minAreaRect = new DkRect(r);
		computeMinAreaCorners();
	};

	/**
	 * Returns the minimum area rectangle.
	 * @return the minimum area rectangle as corner coordinates.
	 **/
	DkRectCorners* getMinAreaCorners() {

		if (minAreaCorners == 0 && minAreaRect)
			minAreaCorners = new DkRectCorners(*minAreaRect);

		return minAreaCorners;
	};

	/**
	 * Computes the minimum area rectangle.
	 **/
	void computeMinAreaCorners() {

		if (minAreaCorners != 0) delete minAreaCorners;
		minAreaCorners = new DkRectCorners(*minAreaRect);
	};

	void setClassVoting(Mat classVoting) {

		this->classVoting = classVoting;

		if (classVoting.empty()) {
			wout << "you gave me an empty class voting mat" << dkendl;
		}
		
		double min, max;
		Point minIdx, maxIdx;
		minMaxLoc(classVoting, &min, &max, &minIdx, &maxIdx);

		this->fontType = maxIdx.x;
		this->accuracy = (float) max;
	};

	void setFeatureVec(Mat featureVec) {

		this->featureVec = featureVec;
		this->featureDim = DkVector(featureVec.size());
	};

	void setMinAreaRot(DkRect r) {
		if (this->minAreaRot != 0) delete minAreaRot;
		this->minAreaRot = new DkRect(r);
	};

	DkRect* getMinAreaRot() {

		return minAreaRot;
	};

	/**
	 * Returns the class voting of the current blob.
	 * The probabilities range from -1 to 1. if all values are < 0
	 * the classifier did not choose any of the three classes.
	 * @return a 1x3 Mat with the probabilities of each class.
	 **/
	Mat getClassVoting() {

		return classVoting;
	};

	/**
	 * Returns the class voting of the specified class for the current blob.
	 * The probabilities range from -1 to 1.
	 * @param fontClass the class index. 
	 * @return the word's probability of being the specified class.
	 **/
	float getClassVotingAt(int fontClass) const {

		if (fontClass < 0 || fontClass >= classVoting.cols) {

			std::string msg = "[DkFontAttr] class index out of bounds: " + DkUtils::stringify(fontClass) + " number of classes: " + DkUtils::stringify(classVoting.cols);
			throw DkIndexOutOfBoundsException(msg, __LINE__, __FILE__);
		}

		const float* cPtr = classVoting.ptr<float>();

		return cPtr[fontClass];

	};

	
	/**
	 * Sets the class voting of the specified class for the current blob.
	 * The probabilities range from -1 to 1.
	 * @param val the probability which shall be assigned.
	 * @param fontClass the class index. 
	 **/
	void setClassVotingAt(float val, int fontClass) {

		if (classVoting.empty()) {
			classVoting = Mat(1, DK_MANUSCRIPT+1, CV_32FC1);
			classVoting.setTo(0);
		}

		if (fontClass < 0 || fontClass >= classVoting.cols) {

			std::string msg = "[DkFontAttr] class index out of bounds: " + DkUtils::stringify(fontClass) + " number of classes: " + DkUtils::stringify(classVoting.cols);
			throw DkIndexOutOfBoundsException(msg, __LINE__, __FILE__);
		}

		float* cPtr = classVoting.ptr<float>();
		cPtr[fontClass] = val;

		double min, max;
		Point minIdx, maxIdx;
		minMaxLoc(classVoting, &min, &max, &minIdx, &maxIdx);

		this->fontType = maxIdx.x;
		this->accuracy = (float) max;

	};

	/**
	 * Returns the class voting of the current blob.
	 * @return a 1x3 Mat with the probabilities of each class.
	 * The probabilities range from -1 to 1. if all values are < 0
	 * the classifier did not choose any of the three classes.
	 * NOTE: the array must be deleted by the caller.
	 **/
	float* getClassVotingArr() {

		if (classVoting.empty() || classVoting.cols == 0)
			return 0;
		
		float* cV = new float[classVoting.cols];
		const float* cvPtr = classVoting.ptr<float>();

		for (int idx = 0; idx < classVoting.cols; idx++)
			*cV++ = *cvPtr++;
		
		cV -= classVoting.cols;	// set pointer to the start of the array again

		return cV;
	};

	Mat getFeatureVec() const {

		return featureVec;
	};

	void releaseFeature() {

		featureVec.release();
	};


	static std::vector<DkLine> computeParagraph(std::vector<DkFontAttr> &words, DkVector imgSize, double angle = 0.0, int numParagraphs = -1, std::vector<DkLine>* rightLines = 0) {

		// fast rotation -> we just need the rotation center
		DkVector centerR = DkIP::calcRotationSize(angle, imgSize) * 0.5f;
		DkVector centerO = imgSize*0.5f;

		// note: this is wrong, if the angle is > +/-90°
		Mat colHist = Mat(1, cvRound(imgSize.width * 0.1f), CV_32FC1);
		colHist.setTo(0);
		float* cPtr = colHist.ptr<float>();

		// init col Hist end
		Mat colHistEnd = (rightLines) ? Mat(1, cvRound(imgSize.width * 0.1f), CV_32FC1) : Mat(1,1, CV_32FC1);
		colHistEnd.setTo(0);
		float* cPtrEnd = colHistEnd.ptr<float>();

		for (unsigned int idx = 0; idx < words.size(); idx++) {

			DkFontAttr w = words[idx];
			w.computeTransformed(centerO, centerR, -angle);

			DkRectCorners rc(*w.getMinAreaRot());

			DkVector left = rc.d + (rc.a - rc.d) * 0.5f;
			cPtr[cvRound(left.x/imgSize.width * colHist.cols)]++;

			if (rightLines) {
				DkVector right = rc.c + (rc.b - rc.c) * 0.5f;
				cPtrEnd[cvRound(right.x/imgSize.width * colHist.cols)]++;
			}
		}

		std::list<float> colMax;
		DkIP::findLocalMaxima(colHist, &colMax, 2.5f, 0.7f, numParagraphs);

		std::vector<DkLine> maxLines;
		std::list<float>::iterator cmIter = colMax.begin();

		for (unsigned int idx = 0; idx < colMax.size(); idx++) {

			float xC = (*cmIter/colHist.cols * imgSize.width - (centerR.x-centerO.x));

			DkVector x(xC, 0);
			DkVector y(xC, imgSize.height);

			DkLine cLine(x,y);
			cLine.rotateLine(centerR, -angle);

			maxLines.push_back(cLine);
			cmIter++;
		}

		if (rightLines) {

			std::list<float> colMaxEnd;
			DkIP::findLocalMaxima(colHistEnd, &colMaxEnd, 2.5f, 0.8f, numParagraphs);

			std::list<float>::iterator cmIter = colMaxEnd.begin();

			for (unsigned int idx = 0; idx < colMaxEnd.size(); idx++) {

				float xC = (*cmIter/colHistEnd.cols * imgSize.width - (centerR.x-centerO.x));

				DkVector x(xC, 0);
				DkVector y(xC, imgSize.height);

				DkLine cLine(x,y);
				cLine.rotateLine(centerR, -angle);

				rightLines->push_back(cLine);
				cmIter++;
			}
		}

		return maxLines;

	};

};

/**
 * The DkNeighborAttr class is an extended class of DkAttr. In addition to the blob properties
 * of DkAttr also the neighborhood DkAttr can be specified.
 **/
class DkNeighborAttr : public DkAttr {

public:
	/**
	 * Default constructor.
	 **/
	DkNeighborAttr() : DkAttr() {}; 

	/**
	 * Copy constructor.
	 **/

	DkNeighborAttr(const DkNeighborAttr &a) : DkAttr(a) {neighbors=a.neighbors; childs=a.childs; onLine=a.onLine;};

	virtual ~DkNeighborAttr() {childs.clear(); neighbors.clear();};

	/**
	* Returns a vector of the neighbors of the current DKNeighborAttr
	* @return a vector of the neighbors of the current DKNeighborAttr
	**/
	vector<DkNeighborAttr> getNeighbors() {return neighbors;};
	
	/**
	* Adds a neighbor to the current DKNeighborAttr, an existing neighbor will be overwritten
	* @param neighbor The DkNeighborAttr which should be added as neighbor
	**/
	void addNeighbor(DkNeighborAttr neighbor) {
														if (find(neighbors.begin(),neighbors.end(),neighbor)==neighbors.end()) {
															neighbors.push_back(neighbor);
														} else {
															replace(neighbors.begin(), neighbors.end(), (*find(neighbors.begin(),neighbors.end(),neighbor)), neighbor);
														};
													};

	/**
	* Removes the given neighbor from this DkNeighborAttr
	* @param neighbor The DkNeighborAttr which should be deleted as neighbor
	**/
	void removeNeighbor(DkNeighborAttr neighbor) {neighbors.erase(find(neighbors.begin(),neighbors.end(),neighbor));};

	/**
	* Returns a vector of the childs of the current DKNeighborAttr
	* @return a vector of the childs of the current DKNeighborAttr
	**/
	vector<DkNeighborAttr> getChilds() {return childs;};
	/**
	* Adds a child to the current DKNeighborAttr
	* @param child The DkNeighborAttr which should be added as child
	**/
	void addChild(DkNeighborAttr child) {if (find(childs.begin(),childs.end(),child)==childs.end()) childs.push_back(child);};
	/**
	* Removes the given childs from this DkNeighborAttr
	* @param child The DkNeighborAttr which should be deleted as child
	**/
	void removeChild(DkNeighborAttr child) {childs.erase(find(childs.begin(),childs.end(),child));};

	/**
	* Returns a vector (of lenght one) on which DkNeighborAttr the current one is lying on
	* @return a vector (of lenght one) on which the current DkNeighborAttr is lying on
	**/	
	vector<DkNeighborAttr> getOnLine() {return onLine;};
	/**
	* sets the given DKNeighborAttr as line of current DKNeighborAttr
	* @param line The line on which the current DKNeighborAttr is lying on
	**/
	void setOnLine(DkNeighborAttr line) {onLine.push_back(line);};

	/**
	* Returns true if the current DKNeighborAttr is empty
	* @return returns true if the current DKNeighborAttr is empty, false otherwise
	**/
	bool empty() {return (area==-1 && orientation==-1.0f && neighbors.empty() && childs.empty() && contourIdx==-1);};

	/**
	* The comparision opertor. Two DKNeighborAttrs are equal if the area and the contourIdx are equal
	* @return true if the area and contourIdx are equals
	**/
	bool operator==(const DkNeighborAttr &a) {return (this->area==a.area && this->contourIdx==a.contourIdx);}
	/**
	* The smaller opertor. One DKNeighborAttrs is smaller if the area is smaller
	* @return true if the area of the current DkNeighborAttr is smaller then the given one
	**/
	bool operator<(const DkNeighborAttr &a) const { return this->area<a.area;};


private:
	vector<DkNeighborAttr> onLine;		/**< a vector (of size one) which holds the DkNeighborAttr of the line the current is lying on**/
	vector<DkNeighborAttr> neighbors;	/**< which DkNeighborAttr are neighbors of the current one**/
	vector<DkNeighborAttr> childs;		/**< which DkNeighborAttr are childs of the current one**/
};

/**
 * The DkCentAttr class is an extended class of DkAttr. In addition to the blob properties
 * of DkAttr, the centroid each blob is an additional attribute.
 **/
class DkCentAttr : public DkAttr {

public:
	/**
	 * Default constructor.
	**/
	DkCentAttr() : DkAttr() {};
	/**
	 * Default destructor.
	**/
	virtual ~DkCentAttr() {};
	/** 
	 * Sets the centroid coordinates of a blob
	 * @param c centroid coordinates
	 */
	void setCentroid(DkInterestPoint c) {centroid = c;};
	/** 
	 * Return the centroid coordinates of the blob.
	 * @return centroid coordinates
	 */
	DkInterestPoint getCentroid() {return centroid;};
protected:
	DkInterestPoint centroid;			/**< centroid coordinates of a blob **/
};
