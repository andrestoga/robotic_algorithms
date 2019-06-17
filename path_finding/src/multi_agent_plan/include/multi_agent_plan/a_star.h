/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-05-30 19:25:47
 */

#ifndef A_STAR_H
#define A_STAR_H

#include "ros/ros.h"
#include "cell_a_star.h"
#include <algorithm>
#include <math.h>

namespace multi_agent_plan
{
  class AStar : public DetPlanner2D<CellAStar>
  {
  public:

    std::vector<CellAStar*> open_;
    std::vector<CellAStar*> closed_;

    AStar( int width, int height, bool is_edge_quartile = true );
    AStar( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile = true );
    AStar(bool is_edge_quartile = true){}
    float h_function( geometry_msgs::Pose2D cell, geometry_msgs::Pose2D goal );

    virtual std::vector<geometry_msgs::Pose2D> pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal ) override;

  };

  AStar::AStar( int width, int height, bool is_edge_quartile )
  : DetPlanner2D<CellAStar>( width, height, is_edge_quartile )
  {
  }

  AStar::AStar( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile )
  : DetPlanner2D<CellAStar>( ogm, is_edge_quartile )
  {
  }

  float AStar::h_function( geometry_msgs::Pose2D cell, geometry_msgs::Pose2D goal )
  {
    return sqrt( pow( goal.x - cell.x, 2 ) + pow( goal.y - cell.y, 2 ) );
  }

  // A-Star planning
  std::vector<geometry_msgs::Pose2D> AStar::pathPlanning( geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal )
  {
    multi_agent_plan::CellAStar* start_cell = map_->getCell( start.x, start.y );
    start_cell->g_ = 0.0;
    start_cell->f_ = start_cell->g_ + h_function( start_cell->pose_, goal );

    open_.push_back( start_cell );

    auto cmp = [](const CellAStar* a, const CellAStar* b)
    {
      return a->f_ > b->f_;
    };

    while( !open_.empty() )
    {
      multi_agent_plan::CellAStar* cell = open_.front();
      std::pop_heap( open_.begin(), open_.end(), cmp );
      open_.pop_back();

      closed_.push_back( cell );

      // Current cell
      // ROS_INFO( "Current cell: %f %f", cell->pose_.y, cell->pose_.x );

      if ( cell->pose_.x == goal.x && cell->pose_.y == goal.y )
      {
        return extractPath( cell );
      }

      std::vector<CellAStar*> neighbors = map_->expand( cell->pose_.x, cell->pose_.y );

      for (int i = 0; i < neighbors.size(); ++i)
      {
        CellAStar* tmp = neighbors[i];

        if ( -1 == tmp->g_ )
        {
          tmp->parent_ = cell;
          // TODO: Here the cost could be 1.4 for the diagonally
          tmp->g_ = cell->g_ + 1;
          tmp->f_ = tmp->g_ + h_function( tmp->pose_, goal );
          open_.push_back( tmp );
          std::push_heap( open_.begin(), open_.end(), cmp );
        }
        else if ( ( cell->g_ + 1 ) < tmp->g_ ) // TODO: Here the cost could be 1.4 for the diagonally
        {
          tmp->g_ = cell->g_ + 1;
          tmp->f_ = tmp->g_ + h_function( tmp->pose_, goal );
          tmp->parent_ = cell;
          std::make_heap( open_.begin(), open_.end(), cmp);
        }

      }

      // ROS_INFO( "Priority queue: " );

      // for( auto m : open_ )
      // {
      //   ROS_INFO( "%f %f : %f", m->pose_.y, m->pose_.x, m->f_ );
      // }

      // ROS_INFO( "1" );
    }

    std::vector<geometry_msgs::Pose2D> failure;

    return failure;
  }
}

#endif