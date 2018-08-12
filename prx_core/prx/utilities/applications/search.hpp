#pragma once

#ifndef PRX_UTIL_SEARCH_HPP
#define	PRX_UTIL_SEARCH_HPP

#include <fstream>
#include <ros/ros.h>

namespace prx
{
    namespace util
    {
        class cell_t
        {
          public:
            cell_t(int r, int c, int empty_state); // row, column, whether its empty or not
            virtual ~cell_t();

            int row = -1;
            int column = -1;   
            int empty = 1; // 1 is empty, 0 is non-empty(blocked)

            std::string to_string();
        };

        class node_t
        {
          public:
            node_t(cell_t* c, node_t* par); // the cell it refers to, parent node
            virtual ~node_t();

            cell_t* cell = NULL;
            node_t* parent = NULL;
            
            // g
            int cost = 0;
            // h
            int get_heur(cell_t* goal);

            // f
            int get_cost_and_heur(cell_t* goal);

            std::vector< std::pair<int, int> > produce_path();

            // get Manhattan distance between two coordinates
            int manhattan_dist(cell_t* a, cell_t* b);
            int manhattan_dist(int a_i, int a_j, int b_i, int b_j);
        };

        class search_t
        {
          private:
            int rows = 0;
            int columns = 0;

            // the 2D array of cells in the environment/graph
            cell_t*** cells = NULL;

            // perform a-star search
            std::vector< std::pair<int, int> > a_star(int initial_i, int initial_j, int goal_i, int goal_j);

            // return node in nodes (list) if the node is referencing the target cell, otherwise NULL
            node_t* find_node_by_cell(std::list< node_t* > nodes, cell_t* cell);

            // find the least est. cost node in a collection of nodes
            node_t* least_est_cost_node(std::list< node_t* > nodes, cell_t* goal);
            
            // given a cell's coordinates, return a vector containing left, right, up, down cells, if they exist
            // (vector type since it may have less than 4 cells)
            std::vector< cell_t* > get_adjacent_cells(cell_t* cell);
            std::vector< cell_t* > get_adjacent_cells(int i, int j);
            
            // READ file data and fill cells member with data
            void set_cells_from_file(std::string file_path);

          public:
            search_t();
            virtual ~search_t();

            std::vector< std::pair<int, int> > search(std::string file_path, 
                int initial_i, int initial_j, int goal_i, int goal_j);
        };


    }
}

#endif