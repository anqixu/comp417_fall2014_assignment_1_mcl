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
  const static double TWO_PI = M_PI*2;
  
 
  const double angularDist(double aRad, double bRad)
  {
    double dRad = bRad - aRad + M_PI;
    if (dRad > 0) {
      dRad = dRad - floor(dRad/TWO_PI)*TWO_PI - M_PI;
    } else {
      dRad = dRad - (floor(dRad/TWO_PI) + 1)*TWO_PI + M_PI;
    }
    return abs(dRad);
  }


  // TODO: these constants may need to be tuned
  const static double INIT_XY_STD_M = 15.0;
  const static double INIT_YAW_STD_DEG = 10000.0; // large value -> uniform random

  // TODO: hint - might want to add more constants here relating to propagation and observation variance; also random particle re-injection settings
  
  // Hard-coded world constraints (okay for this assignment)
  const static double WORLD_X_MIN = 0;
  const static double WORLD_X_MAX = 50;
  const static double WORLD_Y_MIN = 0;
  const static double WORLD_Y_MAX = 50;

  boost::mt19937 rng;
  boost::normal_distribution<> gausNorm;
  boost::uniform_real<> uniformReal;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > randg;
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randn;
  
  
  SkeletonLocaliser( int particleCount = 200 ) : MCLocaliser(particleCount), gausNorm(0.0, 1.0), uniformReal(0.0, 1.0), randg(rng, gausNorm), randn(rng, uniformReal)
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
    
    double xInitM = initialpose.pose.pose.position.x;
    double yInitM = initialpose.pose.pose.position.y;
    double yawInitRad = tf::getYaw(initialpose.pose.pose.orientation);
    ROS_INFO("Initializing PF at (x, y)=(%.2f, %.2f) m and yaw=%.2f deg", xInitM, yInitM, yawInitRad*degree);
    
    const double numParticles = particleCloud.poses.size();
    const double initWeight = 1.0/numParticles;
    for (unsigned int i = 0; i < numParticles; ++i)
    {
      particleCloud.poses[i].position.x = i + randg();
      particleCloud.poses[i].position.y = i + randg();
      geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(i*0.01 + randg());
      particleCloud.poses[i].orientation = odom_quat;
      
      // Store the normalized particle weight as its z-position
      particleCloud.poses[i].position.z = initWeight;
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
    geometry_msgs::Pose sensor_pose;
    sensor_msgs::LaserScan::Ptr simulatedScan;
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      sensor_pose =  particleCloud.poses[i];
      /* If the laser and centre of the robot weren't at the same
       * position, we would first apply the tf from /base_footprint
       * to /base_laser here. */
      try{
        simulatedScan
          = occupancy_grid_utils::simulateRangeScan
          ( this->map, sensor_pose, scan, true );
      }
      catch (occupancy_grid_utils::PointOutOfBoundsException)
      {
        // TODO: hint -- how to recover gracefully?
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
