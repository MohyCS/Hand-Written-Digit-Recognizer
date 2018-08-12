/**
 * @file application.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome,  Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_UTIL_DEMO_APPLICATION_HPP
#define	PRX_UTIL_DEMO_APPLICATION_HPP

#include "prx/utilities/applications/application.hpp"


#include <ros/ros.h>

namespace prx
{
    namespace util
    {


        /**
         * A demo application for prx utilities
         * 
         * @brief <b> An organizer class for our applications. </b>
         * 
         * @authors Rahul Shome 
         */
        class demo_application_t : public util_application_t
        {

          public:
            demo_application_t();
            virtual ~demo_application_t();

            //Sense the scene and return the array indices of the next goal
            std::pair<int, int> sense(std::string sensing_image);


        };


    }
}

#endif
