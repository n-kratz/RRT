#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <random>
#include <algorithm>
#include <math.h>
#include <vector>
#include <iostream>


// utility function to test for a collision
bool CS436Context::state_collides( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
}

// utility function to interpolate between two configurations
CS436Context::vertex CS436Context::interpolate( const CS436Context::vertex& qA,
						const CS436Context::vertex& qB,
						double t ) const {
  CS436Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS436Context::~CS436Context(){}


// TODO
CS436Context::vertex CS436Context::random_sample( const CS436Context::vertex& q_goal ) const {  
  CS436Context::vertex q_rand;
  CS436Context::vertex q_temp;
  double bias_prob = 0.1;
  double randprob = (double)rand() / RAND_MAX;

  if(randprob<bias_prob){
      q_rand = q_goal;
  } else {
      for(int i=0; i<q_goal.size(); i++){
          double rangle = ((static_cast<double>(rand()) / RAND_MAX)*2*M_PI) - M_PI;
          q_rand.push_back(rangle);
      }
      bool collides = CS436Context::state_collides(q_rand);

      //if there's a collision keep generting random points until it doesn't collide
      while(collides){
          //std::cout << "collides: "; std::cout << q_rand[0]; std::cout << std::endl;
          q_rand.clear();
          for(int i=0; i<q_goal.size(); i++){
              double rangle = -M_PI + (static_cast<double>(rand()) / RAND_MAX)*2*M_PI;
              q_rand.push_back(rangle);
          }
          collides = CS436Context::state_collides(q_rand);
      }
  }

  return q_rand;
}

// TODO
double CS436Context::distance( const CS436Context::vertex& q1, const CS436Context::vertex& q2 ){
  double d=0;
  double a, b, c, g, e, f;
  //weighted distances, joints 1-3 are the most important

  a = pow(q1[0]-q2[0], 2);
  b = pow(q1[1]-q2[1], 2);
  c = pow(q1[2]-q2[2], 2);
  g = pow(q1[3]-q2[3], 2)*0.1;
  e = pow(q1[4]-q2[4], 2)*0.1;
  f = pow(q1[5]-q2[5], 2)*0.1;

  d = std::sqrt(a+b+c+g+e+f);
  return d;
}

// TODO
CS436Context::vertex CS436Context::nearest_configuration(CS436Context::vertex& q_rand, CS436Context::path& children){
  CS436Context::vertex q_near;
  double dist;

  double mind = 1000000;
  //search through all nodes in the children vector to find the closest node to add branch to
  for(int i=0; i< children.size(); i++){
      dist = CS436Context::distance(q_rand, children[i]);
      if(dist<mind){
          mind = dist;
          q_near = children[i];
      }
  }
  return q_near;
}

// TODO
bool CS436Context::is_subpath_collision_free( const CS436Context::vertex& q_near,
					      const CS436Context::vertex& q_new ){
    bool collision_free;
    bool collides;
    for( double t=0.0; t<=1.0; t+=0.05 ){
        CS436Context::vertex path = CS436Context::interpolate( q_near, q_new, t );
        collides = CS436Context::state_collides(path);

        if(collides){
            collision_free = false;
            break;
        } else {
            collision_free = true;
        }
    }

  return collision_free;
}


// TODO
CS436Context::path CS436Context::search_path( const CS436Context::vertex& q_init,
                                              const CS436Context::vertex& q_goal,
                                              CS436Context::path& children,
                                              CS436Context::path& parent){
  CS436Context::path P = {q_goal, children[children.size()-1]};
  CS436Context::vertex cur_parent = parent[parent.size() - 1];

  //reconstruct the path by finding the parent of each corresponding node starting from the goal
  while(cur_parent != q_init){
      CS436Context::path::iterator itr = std::find(children.begin(), children.end(), cur_parent);
      CS436Context::vertex next_parent = parent[std::distance(children.begin(), itr)];
      P.push_back(cur_parent);
      cur_parent = next_parent;
  }
  P.push_back(q_init);
  return P;
}

// TODO
CS436Context::path CS436Context::rrt( const CS436Context::vertex& q_init,
				      const CS436Context::vertex& q_goal ){
  CS436Context::path P;
  CS436Context::path child = {q_init};
  CS436Context::path parent = {q_init};
  CS436Context::vertex q_new, q_rand, q_near, q_step;

  bool solved = false;
  bool c;
  bool collision_free;

  //implement RRT based on slides pseudocode
  while(!solved){
      q_rand = CS436Context::random_sample(q_goal);
      q_near = CS436Context::nearest_configuration(q_rand, child);
      double step = 0.25;

      q_step = CS436Context::interpolate( q_near, q_rand, step );

      collision_free = CS436Context::is_subpath_collision_free(q_near, q_step);

      if(collision_free){
          q_new = q_step;
          parent.push_back(q_near);
          child.push_back(q_new);

          //stopping condition of collision free and small distance to avoid skipping collisions on check path
          double d = CS436Context::distance(q_new, q_goal);
          c = CS436Context::is_subpath_collision_free(q_new, q_goal);
          if(c && (d<1.6)){
              solved = true;
              std::cout << "solved"; std::cout << std::endl;
          }
      }
      else{
          continue;
      }
  }
  P = CS436Context::search_path(q_init, q_goal, child, parent);
  std::cout << "Path size: "; std::cout << P.size();
  std::cout << std::endl;
  return P;
}

// This is the method that is called each time a plan is requested
bool CS436Context::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  CS436Context::vertex q_init, q_goal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  CS436Context::path P = rrt( q_init, q_goal );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q_init );

  for( std::size_t i=P.size()-1; i>=1; i-- ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i], P[i-1], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }
  
  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}
