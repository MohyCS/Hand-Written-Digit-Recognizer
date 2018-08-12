/**
 * @file space_types.cpp 
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

#include "prx/utilities/spaces/space_types.hpp"

namespace prx
{
	namespace util
	{

		hash_t<std::string,std::string> space_topologies;

		void init_space_types()
		{
			if(space_topologies.size()!=0)
				return;
			space_topologies["EMPTY"]= "";
			space_topologies["SE2"]="EER";
			space_topologies["SE2andZ"]="EEER";
			space_topologies["SE3"]="EEEQQQQ";
			space_topologies["X"]="E";
			space_topologies["XXXXXX"]="EEEEEE";
			space_topologies["Xd"]="E";
			space_topologies["Xdd"]="E";
			space_topologies["XY"]="EE";
			space_topologies["XdYd"]="EE";
			space_topologies["XddYdd"]="EE";
			space_topologies["XXd"]="EE";
			space_topologies["XYZ"]="EEE";
			space_topologies["TWO_D_BODY"]="EEEE";
			space_topologies["THREE_D_BODY"]="EEEEEE";
			space_topologies["BAXTER_EE_POSE"]="EEEEEEE";
			space_topologies["SE3_VEL_CONTROL"]="EEEEEEE";
			space_topologies["D"]="D";
			space_topologies["R"]="R";
			space_topologies["RR"]="RR";
			space_topologies["Rd"]="E";
			space_topologies["Rdd"]="E";
			space_topologies["XR"]="ER";
			space_topologies["XdRd"]="EE";
			space_topologies["XddRdd"]="EE";
			space_topologies["XXdRRd"]="EERE";
			space_topologies["SOCar"]="EEREE";
			space_topologies["Pushing"]="EERER";
			space_topologies["Airplane"]="EEEREEE";
			space_topologies["AirControl"]="EEE";
			space_topologies["FixedWing"]="EEEERRRRE";
			space_topologies["PushingManip"]="EEEEEEEE";
			space_topologies["PushingManipControl"]="EEEE";
			space_topologies["RallyCar"]="EEEEREEE";
			space_topologies["RallyCarTask"]="EEER";
			space_topologies["WheelControl"]="REE";
			space_topologies["Vector"]="ER";
			space_topologies["OneLink"]="RE";
			space_topologies["TwoLink"]="RREE";
			space_topologies["ThreeLink"]="RRREEE";
			space_topologies["Time"]="E";
			space_topologies["DynamicRigidBody"]="EEEQQQQEEEEEE";
			space_topologies["HeliControl"]="EERR";
			space_topologies["bomb_state"]="EEE";
			space_topologies["ODE_J"]="E";
			space_topologies["ODE_B"]="E";
			space_topologies["SimpleManipulator"]="EEERRRD";
			space_topologies["Baxter"]="EEEEEEEEEEEEEEDD";
			space_topologies["BaxterLeftFullArm"]="EEEEEEED";
			space_topologies["BaxterLeftArm"]="EEEEEEE";
			space_topologies["BaxterRightFullArm"]="EEEEEEED";
			space_topologies["BaxterRightArm"]="EEEEEEE";
			space_topologies["MobileBaxter"]="EEREEEEEEEEEEEEEEDD";
			space_topologies["MobileBaxterLeftFullArm"]="EEREEEEEEE";
			space_topologies["MobileBaxterLeftArm"]="EEEEEEE";
			space_topologies["MobileBaxterRightFullArm"]="EEREEEEEEE";
			space_topologies["MobileBaxterRightArm"]="EEEEEEE";
			space_topologies["MotomanTorso"]="E";
			space_topologies["MotomanNoUnigripper"]="EEEEEEEEEEEEEEEDD";
			space_topologies["MotomanNoUnigripperLeftFull"]="EEEEEEEED";
			space_topologies["MotomanNoUnigripperLeft"]="EEEEEEEE";
			space_topologies["MotomanNoUnigripperLeftFullArm"]="EEEEEEED";
			space_topologies["MotomanNoUnigripperLeftArm"]="EEEEEEE";
			space_topologies["MotomanNoUnigripperRightFull"]="EEEEEEEED";
			space_topologies["MotomanNoUnigripperRight"]="EEEEEEEE";
			space_topologies["MotomanNoUnigripperRightFullArm"]="EEEEEEED";
			space_topologies["MotomanNoUnigripperRightArm"]="EEEEEEE";
			space_topologies["MotomanDU"]="EEEEEEEEEEEEEEEEEDD";
			space_topologies["MotomanDULeftFull"]="EEEEEEEEED";
			space_topologies["MotomanDULeft"]="EEEEEEEEE";
			space_topologies["MotomanDULeftFullArm"]="EEEEEEEED";
			space_topologies["MotomanDULeftArm"]="EEEEEEEE";
			space_topologies["MotomanDURightFull"]="EEEEEEEEED";
			space_topologies["MotomanDURight"]="EEEEEEEEE";
			space_topologies["MotomanDURightFullArm"]="EEEEEEEED";
			space_topologies["MotomanDURightArm"]="EEEEEEEE";
			space_topologies["MotomanDUAllArms"]="EEEEEEEEEEEEEEEEE";
			space_topologies["Motoman"]="EEEEEEEEEEEEEEEEDD";
			space_topologies["MotomanLeftFull"]="EEEEEEEEED";
			space_topologies["MotomanLeft"]="EEEEEEEEE";
			space_topologies["MotomanLeftFullArm"]="EEEEEEEED";
			space_topologies["MotomanLeftArm"]="EEEEEEEE";
			space_topologies["MotomanRightFull"]="EEEEEEEED";
			space_topologies["MotomanRight"]="EEEEEEEE";
			space_topologies["MotomanRightFullArm"]="EEEEEEED";
			space_topologies["MotomanRightArm"]="EEEEEEE";
			space_topologies["Hand"]="D";
			space_topologies["Motoman_Parallel_Unigripper"]="EEEEEEEEEEEEEEEEDDDE";
			space_topologies["MotomanLeftFull_Parallel_Unigripper"]="EEEEEEEEED";
			space_topologies["MotomanLeft_Parallel_Unigripper"]="EEEEEEEEE";
			space_topologies["MotomanLeftFullArm_Parallel_Unigripper"]="EEEEEEEED";
			space_topologies["MotomanLeftArm_Parallel_Unigripper"]="EEEEEEEE";
			space_topologies["MotomanRightFull_Parallel_Unigripper"]="EEEEEEEEDDE";
			space_topologies["MotomanRight_Parallel_Unigripper"]="EEEEEEEE";
			space_topologies["MotomanRightFullArm_Parallel_Unigripper"]="EEEEEEEDDE";
			space_topologies["MotomanRightArm_Parallel_Unigripper"]="EEEEEEE";
			space_topologies["Parallel_Unigripper"]="DDE";
			space_topologies["Kuka"]="EEEEEEED";
			space_topologies["KukaArm"]="EEEEEEE";
			space_topologies["BaxterControl"]="EEEEEEEEEEEEEEE";
			space_topologies["ManipulationSampler"]="EEERR";
			space_topologies["ManipulationSamplerNear"]="ERR";
			space_topologies["PointRollPitchYaw"]="EEERRR";
			space_topologies["ConstructionDiskState"]="EEERRD";
			space_topologies["ConstructionDiskControl"]="ERDD";
			space_topologies["CPG"]="EEER";
			space_topologies["CPG3PO"]="EEEEE";
			space_topologies["CPG_freq"]="EEEEEEEER";
			space_topologies["15Puzzle"]="DDDDDDDDDDDDDDDD";
			space_topologies["24Puzzle"]="DDDDDDDDDDDDDDDDDDDDDDDDD";
			space_topologies["8Puzzle"]="DDDDDDDDD";
			space_topologies["NPuzzleControl"]= "D";
		}
	}
}