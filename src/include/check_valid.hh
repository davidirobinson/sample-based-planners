//
// check_valid.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <math.h>
#include <vector>

#include <types.hh>

#include <opencv2/opencv.hpp>


struct BresenhamParams
{
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;

	BresenhamParams(int p1x, int p1y, int p2x, int p2y)
	{
		UsingYIndex = 0;

		if (fabs(static_cast<double>(p2y - p1y) / static_cast<double>(p2x - p1x)) > 1)
			UsingYIndex++;

		if (UsingYIndex)
		{
			Y1 = p1x;
			X1 = p1y;
			Y2 = p2x;
			X2 = p2y;
		}
		else
		{
			X1 = p1x;
			Y1 = p1y;
			X2 = p2x;
			Y2 = p2y;
		}

		if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			Flipped = 1;
			Y1 = -Y1;
			Y2 = -Y2;
		}
		else
			Flipped = 0;

		if (X2 > X1)
			Increment = 1;
		else
			Increment = -1;

		DeltaX = X2 - X1;
		DeltaY = Y2 - Y1;
		IncrE = 2 * DeltaY * Increment;
		IncrNE = 2 * (DeltaY - DeltaX) * Increment;
		DTerm = (2 * DeltaY - DeltaX) * Increment;
		XIndex = X1;
		YIndex = Y1;
	}


};

void cont_xy_to_cell(const double x, const double y, short unsigned &pX, short unsigned &pY, const size_t x_size, const size_t y_size);

void get_current_point(const BresenhamParams &params, int &x, int &y);

bool get_next_point(BresenhamParams &params);

bool is_valid_line_segment(const double x0, const double y0, const double x1, const double y1, const Map &map);

bool is_valid_arm_config(const ArmConfiguration &config, const Map &map, const double arm_link_length);
