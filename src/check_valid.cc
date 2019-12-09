//
// check_valid.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <check_valid.hh>


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
	{
		(params->UsingYIndex)++;
	}

	if (params->UsingYIndex)
	{
		params->Y1=p1x;
		params->X1=p1y;
		params->Y2=p2x;
		params->X2=p2y;
	}
	else
	{
		params->X1=p1x;
		params->Y1=p1y;
		params->X2=p2x;
		params->Y2=p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0)
	{
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	}
	else
	{
		params->Flipped = 0;
	}

  	if (params->X2 > params->X1)
  	{
    	params->Increment = 1;
  	}
  	else
	{
    	params->Increment = -1;
	}

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
		*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
		*y = -*y;
	}
}

int get_next_point(bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;

	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
	{
		params->DTerm += params->IncrE;
	}
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, const Map &map)
{
	bresenham_param_t params;
	int nX, nY;
    short unsigned int nX0, nY0, nX1, nY1;

	// Make sure the line segment is inside the map
	if(x0 < 0 || x0 >= map.size_x ||
	   x1 < 0 || x1 >= map.size_x ||
	   y0 < 0 || y0 >= map.size_y ||
	   y1 < 0 || y1 >= map.size_y)
	{
		return 0;
	}

	ContXY2Cell(x0, y0, &nX0, &nY0, map.size_x, map.size_y);
	ContXY2Cell(x1, y1, &nX1, &nY1, map.size_x, map.size_y);

	// Iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do
	{
		get_current_point(&params, &nX, &nY);
		if (map.data.at(nX).at(nY) == MapState::Occupied)
		    return 0;
	}
	while (get_next_point(&params));

	return 1;
}

bool IsValidArmConfiguration(const ArmConfiguration &config, const Map &map)
{
 	// Iterate through all the links starting with the base
    double x0, y0, x1, y1;
	x1 = y1 = 0;

	for (const auto &angle : config.angles)
	{
		// Compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS * cos(angle);
		y1 = y0 + LINKLENGTH_CELLS * sin(angle);

		std::cout << "checking " << x0 << "," << y0 << " to " << x1 << "," << y1 << std::endl;

		// Check the validity of the corresponding line segment
		if (!IsValidLineSegment(x0, y0, x1, y1, map))
			return false;
	}
	return true;
}
