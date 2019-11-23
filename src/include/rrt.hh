//
// rrt.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct RRTOptions
{
    explicit RRTOptions(const Json::Value &json)
    {
    }
};

class RRT : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);
};
