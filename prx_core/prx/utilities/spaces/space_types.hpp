/**
 * @file space_types.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2018, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Kimmel, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_SPACE_TYPES
#define PRX_SPACE_TYPES


#include "prx/utilities/definitions/hash.hpp"

namespace prx
{
	namespace util
	{
		extern hash_t<std::string,std::string> space_topologies;

		void init_space_types();
	}

}

#endif