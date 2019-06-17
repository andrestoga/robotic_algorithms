/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-22 11:02:41
 */

#ifndef CELL_A_STAR_H
#define CELL_A_STAR_H

#include "ros/ros.h"
#include "cell.h"

namespace multi_agent_plan
{
	class CellAStar : public Cell
	{
	public:
		float g_;
		float f_;

		CellAStar( geometry_msgs::Pose2D pose, multi_agent_plan::CellAStar* parent = nullptr, float g = -1.0, float f = -1.0 )
		: Cell( pose, parent )
		, g_( g )
		, f_( f )
		{
			
		}

		CellAStar( float g = -1.0, float f = -1.0 )
		: g_( g )
		, f_( f )
		{}
	};
}

#endif