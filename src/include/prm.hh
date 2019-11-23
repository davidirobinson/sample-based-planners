//
// prm.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct PRMOptions
{
    explicit PRMOptions(const Json::Value &json)
    {
    }
};

class PRM : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);

	private:
		const double PRM_thresh = 1.5;
		const int num_PRM_samples = 4000;

		bool dijkstra(tree &T, Config start, Config goal);
};
