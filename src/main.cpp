#include "experiment1.hpp"
#include "experiment2.hpp"
#include "experiment3.hpp"
#include <iostream>
#include <string>
#include <vector>

void print_usage() {
    std::cout << "Usage:" << std::endl;
    std::cout << "  ./ch_demo part1 <osm_numbers>               - Run Contraction Hierarchies experiment" << std::endl;
    std::cout << "  ./ch_demo part2 --default                   - Run CCH with default paths" << std::endl;
    std::cout << "  ./ch_demo part2 <kahip_path> <graph_file_path> <ordering_file_path> [linux|windows] [output_file] [osm_numbers]" << std::endl;
    std::cout << "                                               - Run CCH with custom paths" << std::endl;
    std::cout << "  ./ch_demo part3 [osm_number]                - Run POI Routing experiment (default: osm5)" << std::endl;
    std::cout << "  ./ch_demo all                               - Run all experiments sequentially" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  ./ch_demo part1 1 2 3 4 5                   - Run CH on OSM datasets 1-5" << std::endl;
    std::cout << "  ./ch_demo part2 --default                   - Run CCH with default settings" << std::endl;
    std::cout << "  ./ch_demo part3 5                           - Run POI routing on OSM dataset 5" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage();
        return 1;
    }

    std::string command = argv[1];

    // Run all experiments
    if (command == "all") {
        std::cout << "Running all experiments sequentially..." << std::endl;
        
        // Run part1 experiments
        part_1_experiments("results/part_1/experiments_output_1_4.txt", {1, 2, 3, 4});
        part_1_experiments("results/part_1/experiments_output_5.txt", {5});
        part_1_experiments("results/part_1/experiments_output_6_11.txt", {6, 7, 8, 9, 10, 11});
        
        // Run part2 experiments (if available)
        try {
            part_2_experiments("results/part_2/experiments_output.txt", {1, 2, 3, 4, 5}, true);
        } catch (const std::exception& e) {
            std::cerr << "Warning: part2 experiments failed: " << e.what() << std::endl;
        }
        
        // Run part3 experiment
        part_3_experiments("results/part_3/experiments_output_5.txt", 5);
        
        return 0;
    }

    // Experiment 1: Contraction Hierarchies
    if (command == "part1") {
        if (argc < 3) {
            std::cout << "Error: No OSM numbers provided for part1 experiment" << std::endl;
            print_usage();
            return 1;
        }

        std::vector<int> osm_numbers;
        for (int i = 2; i < argc; i++) {
            try {
                osm_numbers.push_back(std::stoi(argv[i]));
            } catch (const std::exception& e) {
                std::cout << "Error: Invalid OSM number: " << argv[i] << std::endl;
                return 1;
            }
        }

        std::string output_file = "results/part_1/experiments_output.txt";
        part_1_experiments(output_file, osm_numbers);
    }
    // Experiment 2: Customizable Contraction Hierarchies
    else if (command == "part2") {
        if (argc < 3) {
            std::cout << "Error: Insufficient arguments for part2 experiment" << std::endl;
            print_usage();
            return 1;
        }

        if (std::string(argv[2]) == "--default") {
            // Run with default paths
            std::vector<int> osm_numbers = {1, 2, 3, 4, 5};
            if (argc > 3) {
                osm_numbers.clear();
                for (int i = 3; i < argc; i++) {
                    try {
                        osm_numbers.push_back(std::stoi(argv[i]));
                    } catch (const std::exception& e) {
                        std::cout << "Error: Invalid OSM number: " << argv[i] << std::endl;
                        return 1;
                    }
                }
            }
            part_2_experiments("results/part_2/experiments_output.txt", osm_numbers, true);
        } else {
            // Run with custom paths
            if (argc < 5) {
                std::cout << "Error: Insufficient arguments for custom paths" << std::endl;
                print_usage();
                return 1;
            }

            // Parse command-line arguments
            part_2_experiments(argc, argv);
        }
    }
    // Experiment 3: POI Routing
    else if (command == "part3") {
        int osm_number = 5; // Default OSM number
        
        if (argc > 2) {
            try {
                osm_number = std::stoi(argv[2]);
            } catch (const std::exception& e) {
                std::cout << "Error: Invalid OSM number: " << argv[2] << std::endl;
                return 1;
            }
        }

        std::string output_file = "results/part_3/experiments_output_" + std::to_string(osm_number) + ".txt";
        part_3_experiments(output_file, osm_number);
    }
    else {
        std::cout << "Error: Unknown command: " << command << std::endl;
        print_usage();
        return 1;
    }

    return 0;
}