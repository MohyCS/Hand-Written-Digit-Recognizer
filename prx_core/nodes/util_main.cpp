/**
 * @file param_dump.cpp
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

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
// #include <boost/assign/list_of.hpp>


//Addons
#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/applications/application.hpp"

//Sim
#include "prx/utilities/parameters/parameter_reader.hpp"

// #include "prx/utilities/definitions/random.hpp"

#include <fstream>

// #include <boost/bind.hpp>
// #include <boost/program_options.hpp>
// using namespace boost::program_options;

using namespace prx::util;
using namespace std;


int main(int ac, char* av[])
{
    // global_reader = NULL;
    std::string node_name;
    std::string file_name;
    if (ac == 1)
    {
        PRX_FATAL_S("You must specify the node name in args i.e. args=utilities ");
    }
    else
    {
        node_name = av[1];
        // This skips the first 8 characters, which by default in ros is: __name:=
        std::string test = node_name.substr(0,8);
        if (test == "__name:=")
        {
            node_name = node_name.substr(8);
        }
    }
    PRX_INFO_S ("Util node name : " << node_name);
        
    // Initialize ROS. Call this node "simulation" when it spawns.
    ros::init(ac, av, node_name);
    ros::NodeHandle main_node_handle;

    global_storage = new parameter_storage_t( "" );

    parameter_reader_t* reader;

    reader = new parameter_reader_t(node_name,global_storage);
     
    
    util_application_t* app = reader->create_from_loader<util_application_t > ("application");


    if(ros::param::has("prx/env_file"))
    {
        ros::param::get("prx/env_file", file_name);
        PRX_PRINT("The argument file is: "<<file_name, PRX_TEXT_MAGENTA);
    }
    else
    {
        PRX_FATAL_S("Environment file needs to be specified ");
    }
    std::string block_filename = "block.yaml";
    if(ros::param::get("prx/block_filename", block_filename))
    {
        PRX_PRINT("Block Filename read as "<<block_filename, PRX_TEXT_GREEN);
    }
    else
    {
        PRX_PRINT("Block Filename NOT FOUND; Using "<<block_filename, PRX_TEXT_RED);
    }
    app->build_environment(file_name,block_filename);
    auto ret = std::system("rosparam delete prx/parameter_mutex");
    app->init(reader);

    
    // ros::Timer sim_timer = main_node_handle.createTimer(ros::Duration(1), &util_application_t::frame, app);
    ros::Timer comm_timer = main_node_handle.createTimer(ros::Duration(1), &util_application_t::info_broadcasting, app);
    app->execute();
    

    ros::getGlobalCallbackQueue()->callAvailable();
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
        
        
    

    return 0;
}