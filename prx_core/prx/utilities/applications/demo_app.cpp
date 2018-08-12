/**
 * @file application.cpp
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

#include "prx/utilities/applications/demo_app.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "prx/utilities/communication/tf_broadcaster.hpp"

#include <fstream>
#include "prx/utilities/definitions/sys_clock.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx/utilities/definitions/random.hpp"
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS(prx::util::demo_application_t, prx::util::util_application_t)

namespace prx
{

    namespace util
    {

        demo_application_t::demo_application_t()
        {}

        demo_application_t::~demo_application_t()
        {}

        std::pair<int, int> demo_application_t::sense(std::string sensing_image)
        {
            int digit = 0;
            //Currently senses a random digit in the image.
            //#######################################################
            // digit = uniform_int_random(0,9);                      //#                                 
            //#######################################################


             //You can invoke your code using an std::system call, or write your code in C++ and include it here, or invoke your code through ROS
            //###############################################################
            //###############################################################
            //###############################################################
            //###############################################################
            //#########################ASSIGNMENT 2##########################
            //###############################################################
            //###############################################################
            //###############################################################
            //###############################################################

            //SENSE THE DIGIT IN FILE sensing_image
            //digit = neural_network(sensing_image);
            std::string command = "python $PRACSYS_PATH/prx_core/sensing/sense_environment.py "+sensing_image;
            std::system(command.c_str());
            std::ifstream fin;
            char* w = std::getenv("PRACSYS_PATH");
            std::string filename(w);
            filename += ("/prx_output/images/predict.ion");
			fin.open(filename);
			fin >> digit;
            PRX_PRINT("\n##########################\n##########################\nSensed Digit "<<digit<<" in image \n"<<sensing_image<<"\n##########################\n##########################", PRX_TEXT_LIGHTGRAY);
            return digit_to_position[digit];
        }
    }
}
