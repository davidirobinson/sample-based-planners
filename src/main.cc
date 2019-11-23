//
// main.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <planner.hh>
#include <prm.hh>
#include <rrt_connect.hh>
#include <rrt_star.hh>
#include <rrt.hh>

#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <json/json.h>


void print_usage(char **argv)
{
    std::cerr << "USAGE: " << argv[0] << " -c <path-to-config> -m <path-to-map>" << std::endl;
}

Json::Value read_json(const std::string &path)
{
    std::ifstream ifs(path);
    if (ifs.fail())
        throw std::runtime_error(path + " does not exist");

    Json::Value root;
    ifs >> root;
    return root;
}

int main(int argc, char *argv[])
{
    std::string config_path;
    std::string map_path;

    int option = -1;
    while ((option = getopt(argc, argv, "c:m:h")) != -1)
    {
        switch (option)
        {
            case 'c':
                config_path = std::string(optarg);
                break;
            case 'm':
                map_path = std::string(optarg);
                break;
            case 'h':
                print_usage(argv);
                std::exit(EXIT_FAILURE);
            default:
                break;
        }
    }

    if (config_path.empty() || map_path.empty())
    {
        print_usage(argv);
        std::exit(EXIT_FAILURE);
    }

    const auto config_json = read_json(config_path);
    const auto options = PlannerOptions(config_json);

    std::cout << std::endl << "Loaded " << options.arm_dof << " dof arm..." << std::endl;

    std::cout << "Start angles " << (options.planner_units == PlannerUnits::Degrees ? "(degrees): " : "(rads): ");
    for (const auto &angle : options.arm_start_rads)
        std::cout << (options.planner_units == PlannerUnits::Degrees ? angle * 180 / M_PI : angle) << " ";
    std::cout << std::endl;

    std::cout << "End angles " << (options.planner_units == PlannerUnits::Degrees ? "(degrees): " : "(rads): ");
    for (const auto &angle : options.arm_end_rads)
        std::cout << (options.planner_units == PlannerUnits::Degrees ? angle * 180 / M_PI : angle) << " ";
    std::cout << std::endl;


    // TODO: Load map as greyscale image
    // const auto map_image = cv::imread()

    // TODO: Check start and end have no collisions!

    // Choose planner
    Planner *planner;
    switch (options.planner_type)
    {
        case PlannerType::RRT:
            std::cout << "RRT Planner" << std::endl;
            planner = new RRT;
            break;
        case PlannerType::RRTConnect:
            std::cout << "RRT-Connect Planner" << std::endl;
            planner = new RRTConnect;
            break;
        case PlannerType::RRTStar:
            std::cout << "RRT* Planner" << std::endl;
            planner = new RRTStar;
            break;
        case PlannerType::PRM:
            std::cout << "PRM Planner" << std::endl;
            planner = new PRM;
            break;
        default:
            throw std::runtime_error("Unsupported planner type");
    }

    // TODO: Compute Plan

    // TODO: Playback finished plan

    std::exit(EXIT_SUCCESS);
}