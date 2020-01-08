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


/**
 * Data structure containing the internal state parameters for the Bresenham algorithm
 */
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

	BresenhamParams(
		const int p1x,
		const int p1y,
		const int p2x,
		const int p2y)
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

/**
 * Discretize a continuous coordinate into a cell index
 * @param x Continuous X coordinate
 * @param y Continuous Y coordinate
 * @param px Discretized X coordinate to return by reference
 * @param py Discretized Y coordinate to return by reference
 * @param x_size Width of the map
 * @param y_size Height of the map
 */
void cont_xy_to_cell(
	const double x,
	const double y,
	short unsigned &px,
	short unsigned &py,
	const size_t x_size,
	const size_t y_size);

/**
 * Returns current x, y coordinate in line traversal
 * @param params Bresenham params data structure to manage state of line traversal
 * @param x Returns current X coordinate by reference
 * @param y Returns current X coordinate by reference
 */
void get_current_point(
	const BresenhamParams &params,
	int &x,
	int &y);

/**
 * Updates Bresenham params state to next x, y coordinate in line traversal
 * @param params Bresenham params data structure to manage state of line traversal
 * @return False if end of line traversal is reached
 */
bool get_next_point(BresenhamParams &params);

/**
 * Determines whether a line segment is in collision with a map
 * @param x0 X coordinate for beginning of line segment
 * @param y0 Y coordinate for beginning of line segment
 * @param x1 X coordinate for end of line segment
 * @param y1 Y coordinate for end of line segment
 * @param map Map to check for collisions within
 * @return bool True if line segment is not in collision with map
 */
bool is_valid_line_segment(
	const double x0,
	const double y0,
	const double x1,
	const double y1,
	const Map &map);

/**
 * Determines whether an arm configuration is in collision with a map by checking
 * each segment of the arm
 * @param config Arm configuration to check for collisions with
 * @param map Map to check for collisions within
 * @param arm_link_length Length of each arm segment
 * @return bool True if arm is not in collision with map
 */
bool is_valid_arm_config(
	const ArmConfiguration &config,
	const Map &map,
	const double arm_link_length);
