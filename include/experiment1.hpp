#ifndef EXPERIMENT1_HPP
#define EXPERIMENT1_HPP
#include "unified_contraction_hierarchy.hpp"
#include "data_parser.hpp"
#include "dijkstra.hpp"
#include <chrono>
#include <random>
#include <cstdlib>

#define UCH UnifiedContractionHierarchy

/**
 * @brief Runs experiments comparing Dijkstra's algorithm with Contraction Hierarchies
 * 
 * This function performs a series of experiments to evaluate the performance of 
 * Contraction Hierarchies (CH) compared to standard Dijkstra's algorithm:
 * 1. Runs CH preprocessing on multiple road networks and measures time and memory overhead
 * 2. Compares query performance between Dijkstra and CH-Dijkstra on random source-target pairs
 * 3. Verifies correctness by ensuring both algorithms return identical shortest path distances
 * 4. Generates visualizations for OSM5 dataset
 * 
 * @param output_file_base Base path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param mode Mode for the Contraction Hierarchy (REGULAR_CH or POI_CH)
 */
void part_1_experiments(
    std::string output_file_base = "results/part_1/experiment_1.txt",
    std::vector<int> osm_numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    UCH::Mode mode = UCH::Mode::REGULAR_CH
);

#endif // EXPERIMENT1_HPP