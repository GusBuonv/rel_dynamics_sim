#include <ros/ros.h>

#include <Eigen/Dense>
#include <math.h>
#include <time.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "nearlab_msgs/StateStamped.h"
#include "nearlab_msgs/ControlStamped.h"

// These 3 come from nearlab_utils
#include "orbitPropagator.h"
#include "attitudePropagator.h"
#include "quatMath.h"

// Declarations
Eigen::Vector3d r, v, w;
Eigen::Vector4d q;
Eigen::Vector3d u_linear, u_angular;
ros::Time tState, tControl;
double sc_mass, sc_thrust, mean_rate, grav_param, dist_const, time_const;
Eigen::Vector3d rOrb;
Eigen::Matrix3d J;
bool cwOnly;

void stateCallback(const geometry_msgs::PoseStamped msg){
  r(0) = msg.pose.position.x;
  r(1) = msg.pose.position.y;
  r(2) = msg.pose.position.z;
  q(0) = msg.pose.orientation.x;
  q(1) = msg.pose.orientation.y;
  q(2) = msg.pose.orientation.z;
  q(3) = msg.pose.orientation.w;
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

void setupSim(const ros::NodeHandle& nh){
  double sc_inertia[3];
  nh.getParam("sc_mass",sc_mass);
  nh.getParam("sc_thrust",sc_thrust);
  nh.getParam("dist_const",dist_const);
  nh.getParam("time_const",time_const);
  nh.getParam("grav_param",grav_param);
  nh.getParam("orbital_radius_x",rOrb(0));
  nh.getParam("orbital_radius_y",rOrb(1));
  nh.getParam("orbital_radius_z",rOrb(2));
  nh.getParam("sc_inertia_xx",sc_inertia[0]);
  nh.getParam("sc_inertia_yy",sc_inertia[1]);
  nh.getParam("sc_inertia_zz",sc_inertia[2]);
  nh.getParam("clohessy_wiltshire",cwOnly);
  mean_rate = sqrt(grav_param/pow(rOrb.norm(),3));
  J = Eigen::MatrixXd::Zero(3,3);
  J(0,0) = sc_inertia[0];
  J(1,1) = sc_inertia[1];
  J(2,2) = sc_inertia[2];
}

int main(int argc, char** argv){
  ros::init(argc,argv,"sc_controller");
  ros::NodeHandle nh;
  setupSim(nh);

  // Subscribers
  ros::Subscriber subState = nh.subscribe("/orbot/space/vicon",100,stateCallback);
  ros::Subscriber subControl = nh.subscribe("/orbot/space/control",100,controlCallback);

  // Publishers
  ros::Publisher pubAccel = nh.advertise<geometry_msgs::AccelStamped>("/orbot/space/dynamics/rel_accel",100);
  ros::Publisher pubState = nh.advertise<nearlab_msgs::StateStamped>("/orbot/space/state/truth",100);

  // Loop
  ROS_INFO("Dynamics Listening for Initial State from Vicon for Initial Estimate");
  ros::Rate loop_rate(100);
  bool initialized = false;
  tState = ros::Time(0);
  tControl = ros::Time(0);
  ros::Time tPrev;
  int sequence = 0;
 
  OrbitalParams orbParams(sc_mass,sc_thrust,time_const,dist_const,rOrb,grav_param);
  AttitudeParams attParams(J);

  while(ros::ok()){
    
    if(!initialized){
      if(tState.toSec() > 0){ // Don't need control input to initialize
        // Initialize
        initialized = true;
        tPrev = tState;
        // Deregister vicon updater
        subState.shutdown();
        if(tControl.toSec() == 0){
          u_linear = Eigen::VectorXd::Zero(3);
          u_angular = Eigen::VectorXd::Zero(3);
        }

        ROS_INFO("Dynamics Simulator Initialized");
        continue;

      }
      ros::spinOnce();
      continue;
    }


    // ************ Acceleration Calculation ***************
    Eigen::Vector3d a,wd;//Acceleration, omega-dot
    // Calculate accel
    a = Eigen::VectorXd::Zero(3);
    if(cwOnly){
      a(0) = 3*mean_rate*mean_rate*r(0) + 2*mean_rate*v(1) + u_linear(0);
      a(1) = -2*mean_rate*v(0) + u_linear(1);
      a(2) = -mean_rate*mean_rate*r(2) + u_linear(2);
    }else{
      // TODO: two-body, J2, higher order, three/four body, etc
    }

    // Calculate gyro rate
    wd = J.inverse()*u_angular;

    // Publish relative acceleration
    geometry_msgs::AccelStamped accelMsg;
    accelMsg.header.seq = sequence;
    accelMsg.header.stamp = tPrev;// Is this right?
    accelMsg.accel.linear.x = a(0);
    accelMsg.accel.linear.y = a(1);
    accelMsg.accel.linear.z = a(2);
    accelMsg.accel.angular.x = wd(0);
    accelMsg.accel.angular.y = wd(1);
    accelMsg.accel.angular.z = wd(2);
    pubAccel.publish(accelMsg);
    // *******************************************************


    // ************ Propagation of State ******************
    // Calculate time since last update to state
    double dt = (ros::Time::now() - tPrev).toSec();
    tPrev = ros::Time::now();

    // Propagate orbital dynamics
    Eigen::MatrixXd control_linear = Eigen::MatrixXd::Zero(6,1);
    control_linear.col(0).tail(3) = u_linear;
    Eigen::MatrixXd stateHistLin = Eigen::MatrixXd::Zero(6,2);
    cwProp(stateHistLin,r,v,control_linear,dt,2,orbParams);
    r = stateHistLin.col(1).head(3);
    v = stateHistLin.col(1).tail(3);
    
    // Propagate attitude dynamics
    Eigen::MatrixXd control_angular = Eigen::MatrixXd::Zero(7,1);
    control_angular.col(0).tail(3) = u_angular;
    Eigen::MatrixXd stateHistAtt = Eigen::MatrixXd::Zero(7,2);
    attProp(stateHistAtt,q,w,control_angular,dt,2,attParams);
    q = stateHistAtt.col(1).head(4);
    w = stateHistAtt.col(1).tail(3);

    // Publish State
    nearlab_msgs::StateStamped stateMsg;
    stateMsg.header.seq = sequence++;
    stateMsg.header.stamp = tPrev;

    stateMsg.r.x = r(0);
    stateMsg.r.y = r(1);
    stateMsg.r.z = r(2);
    stateMsg.v.x = v(0);
    stateMsg.v.y = v(1);
    stateMsg.v.z = v(2);
    stateMsg.w.x = w(0);
    stateMsg.w.y = w(1);
    stateMsg.w.z = w(2);
    stateMsg.q.x = q(0);
    stateMsg.q.y = q(1);
    stateMsg.q.z = q(2);
    stateMsg.q.z = q(3);

    //******************************************************


    ros::spinOnce();
    loop_rate.sleep();
  }



  
}
