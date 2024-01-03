#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <random>
#include <algorithm>
#include <math.h>
#include <vector>
#include <iostream>

MOVEIT_CLASS_FORWARD( CS436Context );

class CS436Context : public planning_interface::PlanningContext {

public:
  
  typedef std::vector<double> vertex;
  typedef std::size_t index;
  typedef std::pair<index,index> edge;
  typedef std::vector<vertex> path;

  CS436Context( const robot_model::RobotModelConstPtr& model, 
		const std::string &name, 
		const std::string& group, 
		const ros::NodeHandle &nh = ros::NodeHandle("~") );
  virtual ~CS436Context();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Feel free to change this method if you can do better.

     Test if a state collides with the scene.
     Call this method if you need to find if the robot collides with the 
     environment in the given robot's state.
     \param[in] q The robot state
     \return      true if the robot state collides. false otherwise
  */
  bool state_collides( const vertex& q ) const;

  /**
     Feel free to change this method if you can do better.

     Linearly interpolate between qA and qB. t is between [0,1]. If t=0, 
     it returns qA. If t=1, it returns qB. 
     \param[in] qA   The start robot configuration
     \param[in] qB   The final robot configuration
     \param     t    The joint step used in between configurations
     \return         The interpolated configuration.
  */
  vertex interpolate( const vertex& qA, const vertex& qB, double t )const;

  /**
     TODO
     
     Create a collision-free random configuration (i.e. 6 joints) of the robot.
     \param q_goal The goal configuration (use this with a bias)
     \return  A collision-free random configuration
  */
  vertex random_sample( const vertex& q_goal ) const;

  /**
     TODO

     Calculate the distance between two configurations.
     \param q1 The first configuration
     \param q2 The second configuration
     \return   The distance between q1 and q2
  */
  double distance( const vertex& q1, const vertex& q2 );

  /**
     TODO

     Search the tree for the nearest configuration to q_rand and return it.
     \param q_rand The configuration to search for a nearest neighbor.
     \return    The nearest configuration to q_rand
  */
  vertex nearest_configuration( CS436Context::vertex& q_rand, CS436Context::path& children);

  /**
     TODO

     Check for collisions on a straight line path between q_near and q_new.
     \param q_near The nearest configuration in the tree
     \param q_new  The new configuration to add to the tree
     \return       True if the path is collision free. False otherwise.
  */
  bool is_subpath_collision_free( const vertex& q_near, const vertex& q_new );

  
  /**
     TODO
     
     Extract a path from the tree beteen q_init and q_goal
     \param     q_init     The start configuration
     \param     q_goal     The goal configuration
     \return               The path between q_init and q_goal
  */
  path search_path( const CS436Context::vertex& q_init,
                                              const CS436Context::vertex& q_goal,  CS436Context::path& children, CS436Context::path& parent );

  /**
     TODO

     Build a RRT tree from q_init until q_goal is reach (or give up)
     and return a path if one is found.
     \param     q_init     The start configuration
     \param     q_goal     The goal configuration
     \return               The path between q_init and q_goal
  */
  path rrt( const vertex& q_init, const vertex& q_goal );

  
protected:

  robot_model::RobotModelConstPtr robotmodel;

};
