#include <ros/ros.h>

#include <Eigen/Dense>
#include <math.h>
#include <time.h>

#include "nearlab_utils/orbitPropagator.h"
#include "geometry_msgs/Vector3Stamped"
#include "geometry_msgs/AccelStamped"
#include "geometry_msgs/"
#include "quatMath.h"

Eigen::Vector3d r, v, w;
Eigen::Vector4d q;
Eigen::Vector3d u_linear, u_angular;
ros::Time tState, tControl;
double sc_mass, sc_thrust, mean_rate, grav_param, rOrb[3];
Eigen::Matrix3d J;
bool cwOnly;

void stateCallback(const geometry_msgs::PoseStamped msg){
  r(0) = msg.Pose.position.x;
  r(1) = msg.Pose.position.y;
  r(2) = msg.Pose.position.z;
  q(0) = msg.Pose.orientation.x;
  q(1) = msg.Pose.orientation.y;
  q(2) = msg.Pose.orientation.z;
  q(3) = msg.Pose.orientation.w;
  v = Eigen::VectorXd::Zero(3);
  w = Eigen::VectorXd::Zero(3);
  tState = msg.header.stamp;
}

void controlCallback(const nearlab_msgs::ControlStamped msg){
  u_linear(0) = msg.thrust.x;
  u_linear(1) = msg.thrust.y;
  u_linear(2) = msg.thrust.z;
  u_angular(0) = msg.torque.x;
  u_angular(1) = msg.torque.y;
  u_angular(2) = msg.torque.z;
  tControl = msg.header.stamp;
}

void setupSim(const NodeHandle& nh){
  double sc_inertia[3];
  nh.getParam("sc_mass",sc_mass);
  nh.getParam("sc_thrust",sc_thrust);
  nh.getParam("grav_param",grav_param);
  nh.getParam("orbital_radius_x",srv.request.rOrb[0]);
  nh.getParam("orbital_radius_x",srv.request.rOrb[1]);
  nh.getParam("orbital_radius_x",srv.request.rOrb[2]);
  nh.getParam("sc_inertia_xx",srv.request.sc_inertia[0]);
  nh.getParam("sc_inertia_yy",srv.request.sc_inertia[1]);
  nh.getParam("sc_inertia_zz",srv.request.sc_inertia[2]);
  nh.getParam("clohessy_wiltshire",cwOnly);
  mean_rate = sqrt(grav_param/pow(rOrb.norm(),3));
  J = Eigen::MatrixXd::Zero(3);
  J(0,0) = sc_inertia[0];
  J(1,1) = sc_inertia[1];
  J(2,2) = sc_inertia[2];
}

int main(int argc, char** argv){
  ros::init(argc,argv,"sc_controller");
  ros::NodeHandle nh;
  setupSim(nh);

  // Subscribers
  subState = nh.subscribe("/orbot/space/state/vicon",100,stateCallback);
  subControl = nh.subscribe("/orbot/space/control",100,controlCallback);

  // Publishers
  pubDynamics = nh.advertise<geometry_msgs::AccelStamped>("/orbot/space/dynamics/rel_accel"),100);
  pubState = nh.advertise<nearlab_msgs::StateStamped>("/orbot/space/state/truth"),100);
  
  // setup trajectory clients
  
  
  nearlab_msgs::energy_optimal_traj traj_srv = setupTrajRequest(nh);
  nearlab_msgs::attitude_traj att_srv = setupAttRequest(nh);
  Eigen::MatrixXd rStar, vStar, qStar, tStar;

  // Loop
  ROS_INFO("Controller Listening for Initial Estimate");
  ros::Rate loop_rate(100);
  bool initialized = false;
  tState = ros::Time(0);
  tControl = ros::Time(0);
  ros::Time tPrev;
  int sequence = 0;
  OrbitalParams params(sc_mass,sc_thrust,time_const,dist_const,rOrb,grav_param);

  while(ros::ok()){
    
    if(!initialized){
      if(tState.toSec() > 0){ // Don't need control input to initialize
        // Initialize
        initialized = true;
        if(tControl.toSec() == 0){
          u = Eigen::VectorXd::Zero(3);
        }
        tPrev = tState;
        // Deregister vicon updater
        subState.shutdown();

        ROS_INFO("Dynamics Simulator Initialized");
        continue;

      }
      ros::spinOnce();
      continue;
    }

    Eigen::Vector3d a,wd;//Acceleration, omega-dot
    // Calculate accel
    if(cwOnly){
      a = Eigen::VectorXd::Zero(3);
      a(0) = 3*mean_rate*mean_rate*r(0) + 2*mean_rate*v(1) + u_linear(0);
      a(1) = -2*mean_rate*v(0) + u_linear(1);
      a(2) = -mean_rate*mean_rate*r(2) + u_linear(2);
    }else{
      // TODO: two-body, J2, higher order, three/four body, etc
    }

    // Calculate gyro rate
    //omegaBd =  J\(T_rotor + T_lift + T_motor + T_inertia); %rate of attitude rate
    a = J.inverse()*u_angular;

    // Calculate time since last update to state
    double dt = (ros::Time::now() - tPrev).toSec();
    tPrev = ros::Time::now();

    // Propagate orbital dynamics
    Eigen::MatrixXd control = Eigen::MatrixXd::Zero(6,1);
    Eigen::MatrixXd stateHist = Eigen::MatrixXd::Zero(6,2);
    cwProp(stateHist,r,v,control,dt,2,params);
    
    // Propagate attitude dynamics










    // Publish rel accel and state
    geometry_msgs::AccelStamped accelMsg;
    accelMsg.header.seq = sequence++;
    accelMsg.header.stamp = ros::Time::now();
    accelMsg.header.

    pubControl.publish(controlMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }



  
}
