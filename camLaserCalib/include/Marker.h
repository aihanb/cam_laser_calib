/************************************************************************
* Copyright(c) 2013  Yang Xian
* All rights reserved.
*
* File:	Marker.h
* Brief: 标签类,实现marker图和对应ID之间的转换
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2013/1/25 11:45
* History:
************************************************************************/
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "Marker.h"

#include <vector>
using namespace std;

using namespace cv;

class Marker
{
public:
	Marker(void);
	~Marker(void);	
	static int decode(Mat& _input, int& _numRotation);
	float calPerimeter();
private:
	static Mat rotate(const Mat& _input);
	static int hammDistMarker(const Mat& _code);
	static int code2ID(const Mat& _code);
private:
	static const int m_idVerify[4][3];	// marker 设计的id
public:
	int m_id;	// marker 解码的id
	vector<Point2f> m_points;	// marker的contour信息
	Matx33f m_rotation;
	Vec3f m_translation;
};
