//
// check_valid.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <math.h>
#include <vector>

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

const int LINKLENGTH_CELLS = 10;

inline int get_map_index(int x, int y, int x_size, int y_size);

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);

void get_current_point(bresenham_param_t *params, int *x, int *y);

int get_next_point(bresenham_param_t *params);

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*map, int x_size, int y_size);

bool IsValidArmConfiguration(std::vector<double> angles, double* map, int x_size, int y_size);