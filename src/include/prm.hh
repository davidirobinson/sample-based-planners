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
	PlannerOptions general;
	double thresh = 1.5;
	size_t num_samples = 4000;

    explicit PRMOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
		thresh = json["planner"]["prm"]["thresh"].asDouble();
		num_samples = json["planner"]["prm"]["num_samples"].asUInt64();
	}
};

class PRM : public Planner
{
	public:
		PRM(
			const PRMOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();

	private:
		bool dijkstra(Tree &T, ArmConfiguration start, ArmConfiguration goal);

		const PRMOptions prm_opts_;
};
