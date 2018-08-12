/**
 * @file application.hpp
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
#pragma once

#ifndef PRX_UTIL_APPLICATION_HPP
#define PRX_UTIL_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"
#include <pluginlib/class_loader.h>
#include "prx/utilities/math/config.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/math/geometry_info.hpp"
#include "prx/utilities/applications/search.hpp"

#include <ros/ros.h>

namespace prx
{
    namespace util
    {


        /**
         * An organizer class for our application. It contains the simulator and keeps all the extra 
         * informations that we may need for our application.
         * 
         * @brief <b> An organizer class for our applications. </b>
         * 
         * @authors Rahul Shome 
         */
        class util_application_t
        {

          public:
            util_application_t();
            virtual ~util_application_t();

            /**
             * Initializes from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             */
            virtual void init(const util::parameter_reader_t * const reader);
            
            /**
             * Updates visualization about the position of the robot
             */
            void update_visualization();

            /**
             * This is a needed function to mimic the simulation application. Currently only calls tf_broadcasting.
             */
            virtual void info_broadcasting(const ros::TimerEvent& event);

            void build_environment(std::string filename, std::string block_filename);

            //Function that runs the automaton loop
            void execute();

            //All possible automaton states
            enum STATE
            {
                START, SENSE, PLAN, MOVE
            } agent_state;

            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<util_application_t>& get_loader();


            static pluginlib::ClassLoader<util_application_t> loader;

          protected:

            //Attributes necessary for the automaton control loop
            ros::NodeHandle node_handle;
            tf_broadcaster_t* tf_broadcaster;
            std::vector<std::pair<std::string,config_t>> node_configs;
            util::config_t robot_configuration;
            util::config_t goal_configuration;
            int current_goal_x, current_goal_y;
            double _x,_y,_z,_qx,_qy,_qz,_qw;
            int start_i,start_j;
            space_t* state_space;
            int path_counter;
            bool consumed_waypoint;


            int current_goal_i, current_goal_j; //The currently sensed goal
            std::vector< std::pair<int, int> > current_path; //The currently computed path
            std::string environment_file; //The file that points to the maze
            int r,c; //Rows and columns in the maze
            std::vector<std::vector<int>> maze; //The maze 2D data structure with r rows and c columns

            std::pair<int, int> pose_from_indices(int i, int j); //Helper functions to go between array indices to poses in the environment
            std::pair<int, int> indices_from_pose(int x, int y); //Helper functions to go between poses in the environment to array indices
            void move(); //Moves the agent along the current path
            
            //Sense the scene and return the array indices of the next goal
            virtual std::pair<int, int> sense(std::string sensing_image);

            //Plan from (initial_i, initial_j) to (goal_i, goal_j) in the maze and return the sequence of maze indices
            std::vector< std::pair<int, int> > plan(int initial_i, int initial_j, int goal_i, int goal_j );

            search_t* searcher;
            std::map<int, std::pair<int, int>> digit_to_position;
        };


    }
}

#endif