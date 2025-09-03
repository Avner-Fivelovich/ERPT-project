#ifndef EXPERIMENT3_HPP
#define EXPERIMENT3_HPP
#include "unified_contraction_hierarchy.hpp"
#include "data_parser.hpp"
#include "dijkstra.hpp"
#include "httplib.h"
#include <chrono>
#include <random>
#include <cstdlib>

#define UCH UnifiedContractionHierarchy

/**
 * @brief Main function to run POI routing experiments using Contraction Hierarchies
 * 
 * This function implements part 3 of the assignment, which combines:
 * 1. Finding shortest paths that visit at least one POI (Point of Interest)
 * 2. Creating an interactive web application to visualize these paths
 * 
 * The experiment:
 * - Loads a graph from OpenStreetMap data
 * - Designates random nodes as POIs (5% of all nodes)
 * - Preprocesses the graph using Contraction Hierarchies
 * - Runs test queries to find paths that pass through at least one POI
 * - Writes results to an output file
 * - Creates an interactive visualization
 * 
 * @param output_file Path to write experiment results
 * @param osm_number OSM file number to use (default: 5)
 */
void part_3_experiments(std::string output_file = "results/part_3/experiment_3.txt", int osm_number = 5);

/**
 * @brief Generates JSON data for the interactive map visualization
 * 
 * Creates a JSON file containing:
 * - All graph nodes with their coordinates
 * - Regular edges and shortcut edges
 * - POI nodes information
 * 
 * @param poi_query The POI routing object containing the processed graph
 * @return bool Success status
 */
bool experimentInteractiveMap(UCH& poi_query);

/**
 * @brief Starts a local web server to host the interactive visualization
 * 
 * This creates a web server that:
 * - Serves the visualization HTML/JS/CSS files
 * - Provides API endpoints for path finding requests
 * - Opens the visualization in the default browser
 * 
 * @param poi_query The POI routing object to handle path queries
 * @param port The port to run the server on
 * @return bool Success status
 */
bool startVisualizationServer(UCH& poi_query, int port = 8080);

/**
 * @brief API handler for path comparison requests
 * 
 * Handles requests to compute and compare:
 * - Regular shortest path between two nodes
 * - Shortest path that passes through at least one POI
 * 
 * @param req HTTP request object
 * @param res HTTP response object
 * @param poi_query The POI routing object to process the query
 */
void handleComparePaths(const httplib::Request& req, httplib::Response& res, UCH& poi_query);

#endif // EXPERIMENT3_HPP