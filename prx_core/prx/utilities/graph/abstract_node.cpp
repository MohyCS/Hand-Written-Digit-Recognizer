/**
 * @file abstract_node.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "prx/utilities/graph/abstract_node.hpp"
#include <fstream>

#include <boost/graph/adjacency_list.hpp>

namespace prx
{
    namespace util
    {

        void abstract_node_t::serialize(std::ofstream& output_stream, const space_t* point_space)
        {
            //    PRX_ERROR_S ("Node serialization");
            PRX_ASSERT(point);
            output_stream << node_id << " " << point_space->print_point(point);
        }

        // Accepts an ifstream and deserializes a node to output_node

        void abstract_node_t::deserialize(std::ifstream& input_stream, const space_t* point_space)
        {
            // PRX_DEBUG_COLOR("Node deserialization", PRX_TEXT_LIGHTGRAY);
            double value;
            char trash;
            input_stream >> node_id;
            std::vector<double> vals;
            for( unsigned int i = 0; i < point_space->get_dimension(); i++ )
            {
                input_stream >> value;
                // PRX_DEBUG_S("Value " << value);
                vals.push_back(value);
                if( i < point_space->get_dimension() - 1 )
                    input_stream >> trash;
                // PRX_DEBUG_S("Trash : " << trash);
            }

            point_space->set_from_vector(vals, point);

        }

    }
}
