# Efficient Route Planning Techniques

This repository contains implementations of several advanced route planning algorithms including Contraction Hierarchies (CH), Customizable Contraction Hierarchies (CCH), and Points-of-Interest routing (POI-CH). The project demonstrates significant performance improvements over standard Dijkstra's algorithm for route planning on large road networks.

## Features

- **Regular Contraction Hierarchies (CH)**: Fastest point-to-point routing with optimal preprocessing
- **Customizable Contraction Hierarchies (CCH)**: Fast route planning with quick metric updates
- **POI Routing**: Find routes that visit at least one Point of Interest
- **Interactive Visualizations**: Web-based tools to visualize paths and compare algorithms
- **Comprehensive Benchmarking**: Performance comparison with regular Dijkstra's algorithm

## Getting Started

### Prerequisites

- C++17 compatible compiler (GCC 8+, Clang 6+, or MSVC 2019+)
- CMake 3.12 or higher
- Python 3.x (for visualization server)
- Web browser with JavaScript enabled
- For CCH: KaHIP node ordering tool

### Building the Project

#### Using PowerShell script (Windows)

The repository includes a PowerShell script that simplifies the build process:

```powershell
# Basic build and run
.\compile.ps1

# Build in Release mode
.\compile.ps1 -BuildType Release

# Clean and rebuild
.\compile.ps1 -Clean

# Build only (no execution)
.\compile.ps1 -BuildOnly
```

#### Manual Build

```bash
# Create and enter build directory
mkdir build && cd build

# Configure project
cmake ..

# Build
cmake --build .

# Run the executable
./ch_demo
```

## Running Experiments

The project includes three main types of experiments:

### Experiment 1: Contraction Hierarchies

Compares standard Dijkstra's algorithm with Contraction Hierarchies:

```bash
./ch_demo part1 <osm_numbers>
```

Example:
```bash
./ch_demo part1 1 2 3 4 5
```

This will:
1. Load OSM datasets 1-5
2. Preprocess them using CH
3. Run comparison queries
4. Generate visualizations for OSM5

### Experiment 2: Customizable Contraction Hierarchies

Tests the three-phase CCH algorithm:

```bash
# Default paths (recommended)
./ch_demo part2 --default

# Custom paths
./ch_demo part2 <kahip_path> <graph_file_path> <ordering_file_path> [linux|windows] [output_file] [osm_numbers]
```

Example with default paths:
```bash
./ch_demo part2 --default
```

### Experiment 3: POI Routing

Demonstrates routing with Points of Interest constraints:

```bash
./ch_demo part3 [osm_number]
```

Example:
```bash
./ch_demo part3 5
```

This will:
1. Load OSM5 dataset
2. Randomly designate 5% of nodes as POIs
3. Compute paths that visit at least one POI
4. Start interactive visualization

## Visualizations

The project generates several visualizations in your web browser:

### CH Visualizations (Experiment 1)

After running Experiment 1, visualizations for OSM5 will automatically open in your browser:
- Original graph structure with shortcut edges
- Example paths comparing Dijkstra vs CH
- Expanded (unpacked) CH paths

### POI Routing Visualizations (Experiment 3)

After running Experiment 3, an interactive visualization will open allowing you to:
- Select source and target nodes on the map
- Compare regular shortest path vs POI-visiting path
- See POI nodes highlighted on the map

## Project Structure

- `src/` - Source files
  - `main.cpp` - Entry point
  - `experiment1.cpp` - CH experiments
  - `experiment2.cpp` - CCH experiments
  - `experiment3.cpp` - POI routing experiments
  - `unified_contraction_hierarchy.cpp` - CH implementation
  - `customizable_contraction_hierarchy.cpp` - CCH implementation
  - `data_parser.cpp` - Graph loading utilities
- `include/` - Header files
- `RoadNetworks/` - OSM datasets
- `results/` - Generated output and visualizations
  - `part_1/` - CH results
  - `part_2/` - CCH results
  - `part_3/` - POI routing results
- `visualization/` - Web-based visualization files

## Algorithms

### Contraction Hierarchies (CH)

CH is a preprocessing-based speedup technique that:
1. Assigns a rank to each node
2. Contracts nodes in order of increasing rank
3. Adds shortcuts to preserve shortest paths
4. Enables bidirectional search that only traverses from lower to higher ranked nodes

### Customizable Contraction Hierarchies (CCH)

CCH follows a three-phase approach:
1. **Preprocessing**: Metric-independent phase using nested dissection ordering
2. **Customization**: Fast metric-dependent phase using triangle inequality
3. **Query**: Bidirectional upward/downward search similar to CH

### POI Routing

POI routing is implemented as a specialized CH variant that:
1. Designates certain nodes as Points of Interest
2. Modifies node prioritization to favor POIs during contraction
3. Uses a modified query algorithm that ensures at least one POI is visited

## Troubleshooting

### KaHIP for CCH

If you encounter issues with KaHIP for CCH:
1. Make sure the KaHIP node ordering executable is accessible
2. Use the `--default` flag for Experiment 2 to use included executables(not available)
3. Check that paths are correctly specified for your OS

### Visualization Issues

If visualizations don't automatically open:
1. Check the server messages in the console
2. Manually open a browser and navigate to http://localhost:8000/visualization/ (port 8080 for part 3)
3. Ensure you have permission to start a local server

## Data Sources

No road network datasets included in this repository.

## License

This project is available under the MIT License - see the LICENSE file for details.
