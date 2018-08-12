/**
 * @file distance_metric.cpp
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

#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    namespace util
    {

        pluginlib::ClassLoader<distance_metric_t> distance_metric_t::loader("prx_core", "prx::util::distance_metric_t");

        distance_metric_t::distance_metric_t( )
        {
            nr_points = 0;
            function_name = "default_euclidean_t";
            space = NULL;
            distance_function = NULL;
            function = NULL;
            precision = 4;
        }

        void distance_metric_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            function_name = parameters::get_attribute_as<std::string>("distance_function",reader,template_reader,"default_euclidean_t");
            precision = parameters::get_attribute_as<int>("precision",reader,template_reader,4);
            PRX_DEBUG_COLOR("Distance_function: " << function_name, PRX_TEXT_GREEN);
        }

        unsigned distance_metric_t::get_nr_points( )
        {
            return nr_points;
        }

        void distance_metric_t::link_space( const space_t* inspace )
        {
            // PRX_PRINT("=========================================================", PRX_TEXT_CYAN);
            // PRX_PRINT(" Linking space to distance metric [" << this << "]", PRX_TEXT_GREEN);
            // PRX_PRINT("=========================================================", PRX_TEXT_CYAN);
            
            // PRX_PRINT("SPACE [" << space << "]  FUNCTION [" << function << "]", PRX_TEXT_LIGHTGRAY);
            
            //If the space is NULL, but we have been given a distance function
            bool overwrite = true;
            if( space == NULL && function != NULL )
            {
                //Then we need to not overwrite things
                overwrite = false;
            }
            // PRX_PRINT("Overwriting? :" << (overwrite ? "YES":"NO" ), PRX_TEXT_RED + (overwrite ? 0 : 1 ) );
            
            if (space != inspace)
            {
                this->clear();
                space = inspace;
                unsigned dim = space->get_dimension();
                PRX_ASSERT( dim != 0 );
                
                gamma_val = exp(1)*(1+1.0/dim)+.001;
                space_measure = 1;
                const std::vector<bounds_t*>& sp_bounds = space->get_bounds();
                for( unsigned i=0; i<space->get_dimension(); ++i )
                    space_measure *= sp_bounds[i]->get_upper_bound() - sp_bounds[i]->get_lower_bound();

                if(function_name=="default")
                {
                    function_name = "default_euclidean_t";
                }
                if( overwrite )
                {
                    function = distance_function_t::get_loader().createUnmanagedInstance(function_name);
                }
                function->link_space(inspace);
                distance_function = function->dist;
            }

            // PRX_PRINT("SPACE [" << space << "]  FUNCTION [" << function << "]", PRX_TEXT_LIGHTGRAY);

        }
        
        void distance_metric_t::link_distance_function( distance_function_t* input_function )
        {
            // PRX_PRINT("Linking in new distance function [" << input_function << "] overwriting old function [" << function << "]", PRX_TEXT_CYAN);
            function = input_function;
            distance_function = function->dist;
        }

        distance_function_t* distance_metric_t::get_distance_function()
        {
            return function;
        }

        double distance_metric_t::get_precision( double in, int in_precision )
        {
          double out = in;
          long int power = std::pow(10,in_precision);
          out = out * power;
          out = std::abs(out);
          out+=0.5;
          long int equivalent = (int) out;
          out = equivalent / (double)power;
          if(in < 0)
            out = 0 - out;
          return out;
        }

        long int distance_metric_t::get_rounded_double(double in)
        {
            double out = get_precision(in, precision);
            long int ret = (long int)(out*std::pow(10, precision));
            return ret;
        }

        std::vector<long int> distance_metric_t::get_key(std::vector<double> in)
        {
            std::vector< long int > key;
            for( double in_double : in)
            {
                key.push_back(get_rounded_double(in_double));
            }
            return key;
        }

        void distance_metric_t::add_point_to_map( const abstract_node_t* node)
        {
            std::vector<double> state_vec;
            space->copy_point_to_vector(node->point, state_vec);
            
            auto key = get_key(state_vec);

            find_query_map[key] = node;
        }


        const abstract_node_t* distance_metric_t::find_query( space_point_t* query_point )
        {
            if(find_query_map.empty())
            {
                PRX_ERROR_S("The distance metric has not filled in the lookup map for the find query.");
                return NULL;
            }

            std::vector<double> state_vec;
            space->copy_point_to_vector(query_point, state_vec);

            auto key = get_key(state_vec);

            auto found = find_query_map.find(key);
            if(found == find_query_map.end())
                return NULL;
            else
                return found->second;
        }

        const abstract_node_t* distance_metric_t::find_query( const abstract_node_t* query_node )
        {
            return find_query(query_node->point);
        }

        const abstract_node_t* distance_metric_t::find_state_query( space_point_t* query_point )
        {
            auto found = find_query(query_point);

            if (found == NULL)
            {
                found = single_query(query_point, NULL);
            }

            return found;
        }


        pluginlib::ClassLoader<distance_metric_t>& distance_metric_t::get_loader()
        {
            return loader;
        }

    }
}
