//
// main.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <string>
#include <iostream>
#include <unistd.h>

void print_usage()
{
    std::cerr << "USAGE: ./main -c <path-to-config> -m <path-to-map>" << std::endl;
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
                print_usage();
                std::exit(EXIT_FAILURE);
            default:
                break;
        }
    }

    if (config_path.empty() || map_path.empty())
    {
        print_usage();
        std::exit(EXIT_FAILURE);
    }

    // TODO: Choose planner

    // TODO: Compute plan

    // TODO: Playback finished plan

    std::exit(EXIT_SUCCESS);
}