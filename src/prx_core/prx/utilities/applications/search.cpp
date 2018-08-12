#include "prx/utilities/applications/search.hpp"

namespace prx
{

    namespace util
    {
        cell_t::cell_t(int r, int c, int empty_state)
        {
            row = r;
            column = c;
            empty = empty_state;
        }

        cell_t::~cell_t()
        {}

        std::string cell_t::to_string()
        {
            // row, column
            std::string r = std::to_string(this->row);
            std::string c = std::to_string(this->column);
            return "(" + r + ", " + c + ")";
        }

        node_t::node_t(cell_t* c, node_t* par)
        {
            cell = c;
            parent = par;
        }

        node_t::~node_t()
        {
            cell = NULL;
        }

        int node_t::get_heur(cell_t* goal)
        {
            // heuristic calculation
            return manhattan_dist(this->cell, goal);
        }

        int node_t::get_cost_and_heur(cell_t* goal)
        {
            return this->cost + this->get_heur(goal);
        }

        int node_t::manhattan_dist(cell_t* a, cell_t* b)
        {
            return manhattan_dist(a->row, a->column, b->row, b->column);
        }

        int node_t::manhattan_dist(int a_i, int a_j, int b_i, int b_j)
        {
            // does not take into account action costs which are calculated in nodes

            // edge case: a and b reference same cell
            if (a_i == b_i && a_j == b_j)
                return 0;
            else
                return std::abs(a_j - a_i) + std::abs(b_j - b_i);
        }

        std::vector< std::pair<int, int> > node_t::produce_path()
        {
            // (adds full path from start to end node included)

            std::vector< std::pair<int, int> > path;
            node_t* ptr = this;
            // traverse backwards
            while (ptr != NULL)
            {
                path.push_back(std::make_pair(ptr->cell->row, ptr->cell->column));
                ptr = ptr->parent;
            }

            std::reverse(path.begin(), path.end()); 
            return path;
        }

        search_t::search_t()
        {}

        search_t::~search_t()
        {
            for (int i=0; i < rows; i++)
            {
                for (int j=0; j < columns; j++)
                {
                    free(cells[i][j]);
                }
                free(cells[i]);
            }
        }

        std::vector< std::pair<int, int> > search_t::search(std::string file_path,
            int initial_i, int initial_j, int goal_i, int goal_j)
        {
        	std::vector< std::pair<int, int> > path;

            // for(int i=initial_i; i<=goal_i; ++i)                                      //################
            //     path.push_back(std::make_pair(i,initial_j));                          //################            
            // for(int i=initial_i; i>=goal_i; --i)                                      //################
            //     path.push_back(std::make_pair(i,initial_j));                          //################            
            // for(int j=initial_j+1; j<=goal_j; ++j)                                    //################
            //     path.push_back(std::make_pair(goal_i,j));                             //################        
            // for(int j=initial_j-1; j>=goal_j; --j)                                    //################
            //     path.push_back(std::make_pair(goal_i,j));                             //################ 

            // READ data from file and map it to 2D array
            if (this->cells == NULL)
            {
                set_cells_from_file(file_path);
            }

            path = a_star(initial_i, initial_j, goal_i, goal_j);

            // debug - output path
            // std::cout << "Generated path SOLUTION: \n";
            // for (auto &pair : path)
            // {
            //     std::cout << "(" << pair.first << ", " << pair.second << ") -> ";
            // }
            // std::cout << "\n";

            return path;
        }

        std::vector< std::pair<int, int> > search_t::a_star(int initial_i, int initial_j, int goal_i, int goal_j)
        {
            std::vector< std::pair<int, int> > path;

            // BEGIN A-STAR ALGORITHM
            // create open and visited collections
            std::list< node_t* > open; // nodes which occupy the current search space
            std::list< node_t* > visited; // cells already visited before

            cell_t* goal = this->cells[goal_i][goal_j];

            cell_t* initial_cell = this->cells[initial_i][initial_j];
            node_t* initial_node = new node_t(initial_cell, NULL);
            open.push_back(initial_node); // add initial node

            while (!open.empty())
            {
                // search all open node for the least estimated cost
                node_t* least = least_est_cost_node(open, goal);
                open.remove(least);
                visited.push_back(least);

                if (least == NULL) 
                    std::cout << "Least should not be NULL\n";
                // debug
                // else
                //     std::cout << "Least is referencing cell: " << least->cell->to_string() << "\n";

                if (least->cell->row == goal_i && least->cell->column == goal_j) // check if least is goal
                {
                    // if goal, make sure there is no open node with a lower cost
                    // (not implementing for now)
                    path = least->produce_path();
                    return path;
                }

                // produce successors
                std::vector< cell_t* > adjacent = get_adjacent_cells(least->cell);
                for (auto &adj_cell : adjacent)
                {
                    node_t* successor = new node_t(adj_cell, least);
                    successor->cost = least->cost + 1; // 1 as action cost

                    // node in open list which references the same cell
                    node_t* open_node_same_cell = find_node_by_cell(open, adj_cell);
                    // node in visited list which references the same cell
                    node_t* visited_node_same_cell = find_node_by_cell(visited, adj_cell);

                    if (open_node_same_cell != NULL) 
                    {
                        if (open_node_same_cell->cost <= successor->cost)
                            continue;
                        // cannot do better cost than current open node, so skip successor

                        // open remove?
                    }
                    else if (visited_node_same_cell != NULL)
                    {
                        if (visited_node_same_cell->cost <= successor->cost)
                            continue;
                        // cannot do better cost than current visited node, so skip successor
                        else
                            visited.remove(visited_node_same_cell);
                    }

                    open.push_back(successor);

                }

                
            }

            std::cout << "ERROR: Unexpected code reached\n";

            return path;
        }

        node_t* search_t::find_node_by_cell(std::list< node_t* > nodes, cell_t* cell)
        {
            for (auto &node : nodes)
            {
                if (node->cell == cell)
                    return node;
            }
            return NULL;
        }

        node_t* search_t::least_est_cost_node(std::list< node_t* > nodes, cell_t* goal)
        {
            node_t* least = NULL;
            for (auto &node : nodes)
            {
                if (least == NULL || node->get_cost_and_heur(goal) < least->get_cost_and_heur(goal))
                    least = node;
            }
            return least;
        }

        std::vector< cell_t* > search_t::get_adjacent_cells(cell_t* cell)
        {
            return get_adjacent_cells(cell->row, cell->column);
        }
        
        std::vector< cell_t* > search_t::get_adjacent_cells(int i, int j)
        {
            std::vector< cell_t* > adjacent;
            
            int left = j-1;
            if (left >= 0 && left < rows)
            {
                cell_t* cell = this->cells[i][left];
                if (cell->empty == 1)
                    adjacent.push_back( cell );
            }
            int right = j+1;
            if (right >= 0 && right < rows)
            {
                cell_t* cell = this->cells[i][right];
                if (cell->empty == 1)
                    adjacent.push_back( cell );
            }
            int up = i-1;
            if (up >= 0 && up < rows)
            {
                cell_t* cell = this->cells[up][j];
                if (cell->empty == 1)
                    adjacent.push_back( cell );
            }
            int down = i+1;
            if (down >= 0 && down < rows)
            {
                cell_t* cell = this->cells[down][j];
                if (cell->empty == 1)
                    adjacent.push_back( cell );
            }

            //debug
            // for (auto &cell : adjacent)
            // {
            //     std::cout << "adj i: " << cell->row << ", adj j: " << cell->column << "\n";
            // }
            // std::cout << "end adj for: " << i << ", " << j << "\n";

            return adjacent;
        }

        void search_t::set_cells_from_file(std::string file_path)
        {
            std::ifstream maze_file (file_path);
            if (maze_file.is_open())
            {
                int i = 0;
                std::string line;
                while (getline(maze_file, line))
                {
                    if (i == 0)
                    {
                        this->rows = stoi(line);
                        this->cells = (cell_t***) malloc(this->rows * sizeof(cell_t**));
                    }
                    else if (i == 1)
                    {
                        this->columns = stoi(line);
                        for(int r = 0; r < this->rows; r++)
                            this->cells[r] = (cell_t**) malloc(this->columns * sizeof(cell_t*));
                    }
                    else
                    {
                        int x = 0;
                        std::string raw_num; // string representation of a number
                        std::stringstream ss (line);
                        while (getline(ss, raw_num, ' '))
                        {
                            this->cells[i-2][x] = new cell_t(i-2, x, stoi(raw_num));
                            x++;
                        }
                    }
                    i++;
                }
                
                // debug saved map
                // for (int o=0; o < this->rows; o++)
                // {
                //     for (int p=0; p < this->columns; p++)
                //         std::cout << this->cells[o][p]->empty << " ";
                //     std::cout << "\n";
                // }
                maze_file.close();
            }
        }
    }
}
