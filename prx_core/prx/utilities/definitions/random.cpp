/**
 * @file random.cpp 
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


#include "prx/utilities/definitions/random.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <numeric>
//#include <boost/random/discrete_distribution.hpp>

namespace prx
{
    namespace util
    {

        unsigned rand_count;
        int start_seed;
        /**
         * Global random number generator.
         */
        boost::mt19937 gen;

        void init_random(int seed)
        {
            rand_count = 0;
            start_seed = seed;
            //    srand(55555);
            srand(seed);
        }

        double uniform_random()
        {
            rand_count++;
            double val = rand()*1.0 / RAND_MAX;
            //    PRX_WARN_S("Value: "<<val);
            return val;
        }

        double uniform_random(double min, double max)
        {
            rand_count++;
            double val = (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
            //    PRX_WARN_S("Min: "<<min<<" Max: "<<max<<" Value: "<<val);
            return val;
        }

        int uniform_int_random(int min, int max)
        {
            rand_count++;
            int val = (rand() % (max + 1 - min)) + min;
            // PRX_WARN_S("Min: " << min << " Max: " << max << " Value: " << val);
            return val;
        }

        int roll_weighted_die(std::vector<double> const& weights, bool use_boost_random)
        {
            int event_index = -1;

            double sum = 0;
            for( unsigned i = 0; i < weights.size(); i++ )
            {
                sum += weights[i];
            }
            double val = uniform_random(0, 1);
            double running_total = 0;
            for( unsigned i = 0; i < weights.size(); i++ )
            {
                running_total += weights[i] / sum;
                if( val <= running_total )
                {
                    event_index = i;
                    break;
                }
            }
            //   }
            PRX_ASSERT(event_index != -1);
            return event_index;
        }

        double gaussian_random()
        {
            static int flag = 0;
            static double t = 0.0;
            double v1,v2,r;
            if (flag == 0) 
            {
                do 
                {
                  v1 = 2.0 * uniform_random() - 1.0;
                  v2 = 2.0 * uniform_random() - 1.0;
                  r = v1 * v1 + v2 * v2;
                } 
                while (r == 0.0 || r > 1.0);
                r = sqrt((-2.0*log(r))/r);
                t = v2*r;
                flag = 1;
                return (v1*r);  
            }
            else 
            {
                flag = 0;
                return t;
            }
        }

    }
}
