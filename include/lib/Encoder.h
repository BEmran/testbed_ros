#pragma once
#include "libusdusb4.h"
#include "stdio.h"
#include <iostream>
#include <unistd.h>
#define QUAD_X1 1
#define QUAD_X2 2
#define QUAD_X4 3

class Encoder
{
public:
	Encoder();
	Encoder(short deviceNum, short enNum, long cpr, short _quadMode);
	~Encoder();
	void init() const;
	void Reset();

	long getCount();
	float getAngleDeg();
	float getAngleRad();
	float getRPM();
		
	long getMaxCount() const;
	float getResDeg() const;
	float getResRad() const;

	
private:
	short QuadMode;
	short EnNum;
	short DeviceNum;
	float AngleDeg;
	float AngleRad;
	float Count2Deg;
	float Count2Rad;
	float RPM;
	long Count;
	long CPR;
	long MaxCount;

	void updateEncoder();
	void AdjustCount(unsigned tmp);
};

