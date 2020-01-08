//
// types.hh
//
// Author: David Robinson
// Date: 2019-12-06
//

#pragma once

#include <opencv2/opencv.hpp>

/**
 * Enum to represent possible map states
 */
enum class MapState
{
	Occupied,
	Free
};

/**
 * Data structure to hold information about the map to plan within
 */
struct Map
{
	cv::Mat image;
	size_t size_x;
	size_t size_y;
	std::unordered_map<size_t, std::unordered_map<size_t, MapState>> data;
};

/**
 * Enum to represent possible planners to use
 */
enum class PlannerType
{
    RRT,
    RRTConnect,
    RRTStar,
    PRM
};

/**
 * Data structure containing information to describe an arm configuration
 */
struct ArmConfiguration
{
	size_t id;
	size_t parent_id;
	std::vector<double> angles;
	double cost;
	std::vector<size_t> edges;

	ArmConfiguration() :
		id(0),
		parent_id(-1),
		cost(0.0)
	{
	}

	ArmConfiguration(const std::vector<double> &angles) :
		angles(angles),
		id(0),
		parent_id(-1),
		cost(0.0)
	{
	}
};

using Tree = std::unordered_map<size_t, ArmConfiguration>;

inline std::ostream &operator<<(std::ostream &stream, const ArmConfiguration &arm_config)
{
    for (const auto &angle : arm_config.angles)
        stream << angle * 180 / M_PI << " ";

    return stream;
}

inline bool operator==(const ArmConfiguration &a, const ArmConfiguration &b)
{
	if (a.angles.size() != b.angles.size())
        return false;

	for (size_t i = 0; i < a.angles.size(); i++)
	{
		if (fabs(a.angles[i] - b.angles[i]) > 0.001)
			return false;
	}

	return true;
}
