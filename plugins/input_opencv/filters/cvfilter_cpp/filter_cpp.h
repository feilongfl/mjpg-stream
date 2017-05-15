#pragma once

#ifndef FILTER_CPP_H
#define FILTER_CPP_H

struct HSVRange {
	int iLowH;
	int iHighH;
	int iLowS;
	int iHighS;
	int iLowV;
	int iHighV;
};

struct lines_s {
	Vec2f line;
	float rho;
	float theta;
	Point pt1;
	Point pt2;
};

struct lines_s4 {
	lines_s lineUp, lineDown, lineLeft, lineRight;
};

struct lines_s4v {
	vector<lines_s> lineUps, lineDowns, lineLefts, lineRights;
};


#endif//FILTER_CPP_H
