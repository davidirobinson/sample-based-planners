# Sample-Based Planners

Planning for a high-DOF planar arm with Probabilistic Roadmaps (PRMs) and Rapidly-Exploring Random Trees (RRTs)

## Usage

### Dependencies

- [OpenCV (3.x)](https://opencv.org/)
- [JsonCpp](https://open-source-parsers.github.io/jsoncpp-docs/doxygen/index.html)

### Build

    mkdir -p build && cd build/
    cmake .. && make -j<#-cores>

### Run

    cd build/
    ./src/sample_based_planner -c ../config.json -m ../test/map_01.pgm -v
