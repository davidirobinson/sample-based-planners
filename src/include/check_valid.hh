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
	int x1, y1;
	int x2, y2;
	int increment;
	int using_y_index;
	int delta_x, delta_y;
	int d_term;
	int increment_e, increment_ne;
	int x_index, y_index;
	int flipped;

	BresenhamParams(int p1x, int p1y, int p2x, int p2y)
	{
		using_y_index = 0;

		if (fabs(static_cast<double>(p2y - p1y) / static_cast<double>(p2x - p1x)) > 1)
			using_y_index++;

		if (using_y_index)
		{
			y1 = p1x;
			x1 = p1y;
			y2 = p2x;
			x2 = p2y;
		}
		else
		{
			x1 = p1x;
			y1 = p1y;
			x2 = p2x;
			y2 = p2y;
		}

		if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			flipped = 1;
			y1 = -y1;
			y2 = -y2;
		}
		else
			flipped = 0;

		if (x2 > x1)
			increment = 1;
		else
			increment = -1;

		delta_x = x2 - x1;
		delta_y = y2 - y1;
		increment_e = 2 * delta_y * increment;
		increment_ne = 2 * (delta_y - delta_x) * increment;
		d_term = (2 * delta_y - delta_x) * increment;
		x_index = x1;
		y_index = y1;
	}


};

void cont_xy_to_cell(const double x, const double y, short unsigned &pX, short unsigned &pY, const size_t x_size, const size_t y_size);

void get_current_point(const BresenhamParams &params, int &x, int &y);

bool get_next_point(BresenhamParams &params);

bool is_valid_line_segment(const double x0, const double y0, const double x1, const double y1, const Map &map);

bool is_valid_arm_config(const ArmConfiguration &config, const Map &map, const double arm_link_length);
