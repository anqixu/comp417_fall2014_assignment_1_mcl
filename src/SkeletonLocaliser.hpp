#ifndef MMN_SKELETONLOKALISER_HPP_1403
#define MMN_SKELETONLOKALISER_HPP_1403

#include "MCLocaliser.hpp"

// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"

#include <iostream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

/**
 * This class implements the pure virtual methods of PFLocaliser. It
 * is an "empty" class, in that the methods don't actually update
 * anything. The assignment consists in implementing a class just like
 * this one, but with actual content in the methods.
 */
class SkeletonLocaliser: public MCLocaliser
{
public:
  const static double degree = 180.0/M_PI;
  const static double radian = M_PI/180.0;

  // TODO: these constants may need to be tuned
  const static double INIT_XY_STD_M = 15.0;
  const static double INIT_YAW_STD_DEG = 10000.0; // large value -> uniform random
  
  boost::mt19937 rng;
  boost::normal_distribution<> gausNorm;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > randg;
  
  
  SkeletonLocaliser( int particleCount = 100 ) : MCLocaliser(particleCount), gausNorm(0.0, 1.0), randg(rng, gausNorm)
  {
  }


  virtual void initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose )
  {
    // TODO: complete this function
    // Current code places all particles on a line along x=y. This should be
    // replaced with something more sensible, like drawing particles
    // from a Gaussian distribution with large variance centered on
    // the supplied initial pose, or just placing them in a regular
    // grid across the map.
    
    double init_x = initialpose.pose.pose.position.x;
    double init_y = initialpose.pose.pose.position.y;
    double init_yaw_rad = tf::getYaw(initialpose.pose.pose.orientation);
    ROS_INFO("Initializing PF at (x, y)=(%.2f, %.2f) m and yaw=%.2f deg", init_x, init_y, init_yaw_rad*degree);
    
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x = i + randg();
      particleCloud.poses[i].position.y = i + randg();
      geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(i*0.01 + randg());
      particleCloud.poses[i].orientation = odom_quat;
    }
  }
  
protected:

  /**
   * Your implementation of this should sample from a random
   * distribution instead of blindly adding the odometry increment. It
   * should also update the angle of the particles.
   */
  virtual void applyMotionModel( double deltaX, double deltaY, double deltaT )
  {
    // TODO: complete this function
    if (deltaX > 0 or deltaY > 0)
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x += deltaX;
      particleCloud.poses[i].position.y += deltaY;      
    }
  }

  /**
   * After the motion model moves the particles around, approximately
   * according to the odometry readings, the sensor model is used to
   * weight each particle according to how likely it is to get the
   * sensor reading that we have from that pose.
   */
  virtual void applySensorModel( const sensor_msgs::LaserScan& scan )
  {
    // TODO: complete this function
    /* This method is the beginning of an implementation of a beam
     * sensor model */  
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      geometry_msgs::Pose sensor_pose;      
      sensor_pose =  particleCloud.poses[i];
      /* If the laser and centre of the robot weren't at the same
       * position, we would first apply the tf from /base_footprint
       * to /base_laser here. */
      sensor_msgs::LaserScan::Ptr simulatedScan;
      try{
        simulatedScan
          = occupancy_grid_utils::simulateRangeScan
          ( this->map, sensor_pose, scan, true );
      }
      catch (occupancy_grid_utils::PointOutOfBoundsException)
      {
        continue;
      }

      /* Now we have the actual scan, and a simulated version ---
       * i.e., how a scan would look if the robot were at the pose
       * that particle i says it is in. So now we should evaluate how
       * likely this pose is; i.e., the actual sensor model. */
      
      //   if (i == 0)
      //   {
      //     for (unsigned int k = 0; k < simulatedScan->ranges.size(); ++k)
      //       std::cerr << simulatedScan->ranges[k] << "\n";
      //     std::cerr << "\n\n";
      //   }
    }
  }

  
  /**
   * This is where resampling should go, after applying the motion and
   * sensor models.
   */
  virtual geometry_msgs::PoseArray updateParticleCloud
  ( const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const geometry_msgs::PoseArray& particleCloud )
  {
    // TODO: complete this function
    return this->particleCloud;
  }

  /**
   * Update and return the most likely pose. 
   */
  virtual geometry_msgs::PoseWithCovariance updatePose()
  {
    // TODO: complete this function
    this->estimatedPose.pose.pose = particleCloud.poses[0]; // HINT: this is NOT the correct solution. We want the "most likely pose", not the "pose of the first particle"
    // HINT: remember to also update the covariance
    return this->estimatedPose.pose;
  }

  
};

#endif
