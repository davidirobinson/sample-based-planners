//
// check_valid.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <check_valid.hh>


void cont_xy_to_cell(const double x, const double y, short unsigned &pX, short unsigned &pY, const size_t x_size, const size_t y_size)
{
	constexpr double cellsize = 1.0;

	//take the nearest cell
	pX = static_cast<int>(x / static_cast<double>(cellsize));
	if(x < 0)
		pX = 0;
	if(pX >= x_size)
		pX = x_size - 1;

	pY = static_cast<int>(y / static_cast<double>(cellsize));
	if(y < 0)
		pY = 0;
	if(pY >= y_size)
		pY = y_size - 1;
}

void get_current_point(const BresenhamParams &params, int &x, int &y)
{
	if (params.UsingYIndex)
	{
		y = params.XIndex;
		x = params.YIndex;
		if (params.Flipped)
			x = -x;
	}
	else
	{
		x = params.XIndex;
		y = params.YIndex;
		if (params.Flipped)
			y = -y;
	}
}

bool get_next_point(BresenhamParams &params)
{
	if (params.XIndex == params.X2)
		return false;

	params.XIndex += params.Increment;

	if (params.DTerm < 0 || (params.Increment < 0 && params.DTerm <= 0))
	{
		params.DTerm += params.IncrE;
	}
	else
	{
		params.DTerm += params.IncrNE;
		params.YIndex += params.Increment;
	}
	return true;
}

bool is_valid_line_segment(const double x0, const double y0, const double x1, const double y1, const Map &map)
{
	int nX, nY;
    short unsigned int nX0, nY0, nX1, nY1;

	// Make sure the line segment is inside the map
	if(x0 < 0 || x0 >= map.size_x ||
	   x1 < 0 || x1 >= map.size_x ||
	   y0 < 0 || y0 >= map.size_y ||
	   y1 < 0 || y1 >= map.size_y)
	{
		return false;
	}

	cont_xy_to_cell(x0, y0, nX0, nY0, map.size_x, map.size_y);
	cont_xy_to_cell(x1, y1, nX1, nY1, map.size_x, map.size_y);

	// Iterate through the points on the segment
	BresenhamParams params(nX0, nY0, nX1, nY1);
	do
	{
		get_current_point(params, nX, nY);
		if (map.data.at(nX).at(nY) == MapState::Occupied)
		    return false;
	}
	while (get_next_point(params));

	return true;
}

bool is_valid_arm_config(const ArmConfiguration &config, const Map &map, const double arm_link_length)
{
 	// Iterate through all the links starting with the base
    double x0, y0, x1, y1;
	x1 = y1 = 0;

	for (const auto &angle : config.angles)
	{
		// Compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + arm_link_length * cos(angle);
		y1 = y0 + arm_link_length * sin(angle);

		// Check the validity of the corresponding line segment
		if (!is_valid_line_segment(x0, y0, x1, y1, map))
			return false;
	}
	return true;
}
