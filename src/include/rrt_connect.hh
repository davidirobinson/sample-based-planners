//
// rrt_connect.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct RRTConnectOptions
{
    explicit RRTConnectOptions(const Json::Value &json)
    {
    }
};

class RRTConnect : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);

	private:
		bool connect(tree &T, const Config &new_config);
};
