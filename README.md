# Sample-Based Planners

Planning for a high-DOF planar arm with Probabalistic Roadmaps (PRMs) and Rapidly-Exploring Random Tree (RRTs).

## Usage

### Dependencies

libjsoncpp & OpenCV >= 3.0

### Build

    mkdir build && cd build/
    cmake .. && make -j<#-cores>

### Run

    cd build/src/
    ./sample_based_planner -c ../../config.json -m ../../test/map_01.pgm -v

## TODO

- Create visualizer for arm
- Load map into a sparse representation
- Check for collision in start and goal config
- Execute plan & verify success
- Animate arm trajectory for successful plan
- Update Docs
