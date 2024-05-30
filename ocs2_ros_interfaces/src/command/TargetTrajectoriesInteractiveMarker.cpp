/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h"

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesInteractiveMarker::TargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories)
    : server_("simple_marker"), gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)) {
  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));

  // create an interactive marker for our server
  menuHandler_.insert("Send target pose", boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));

  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(interactiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  menuHandler_.apply(server_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();



    BarrierHandler_.insert("Send barrier 1 pose", boost::bind(&TargetTrajectoriesInteractiveMarker::process1Feedback, this, _1));
  // create an interactive marker for our server
  auto BarrierinteractiveMarker = BarrierMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(BarrierinteractiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  BarrierHandler_.apply(server_, BarrierinteractiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();

  Barrier_2_Handler_.insert("Send barrier 2 pose", boost::bind(&TargetTrajectoriesInteractiveMarker::process2Feedback, this, _1));
  // create an interactive marker for our server
  auto Barrier_2_interactiveMarker = Barrier_2_Marker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(Barrier_2_interactiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  BarrierHandler_.apply(server_, Barrier_2_interactiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();


     obstacle_pub = nodeHandle.advertise<geometry_msgs::Point>("obstacle_position", 1);  
    obstacle_ballPublisher_=nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/obstacle_ball", 1);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::InteractiveMarker TargetTrajectoriesInteractiveMarker::createInteractiveMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.z = 1.0;

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}


visualization_msgs::InteractiveMarker TargetTrajectoriesInteractiveMarker::BarrierMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "barrier";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send barrier position";
  interactiveMarker.pose.position.z = 0.5;

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows

  return interactiveMarker;
}

visualization_msgs::InteractiveMarker TargetTrajectoriesInteractiveMarker::Barrier_2_Marker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "barrier 2";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send barrier 2 position";
  interactiveMarker.pose.position.z = 1.0;
  interactiveMarker.pose.position.x = 1.0;
  interactiveMarker.pose.position.y = 1.0;

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows

  return interactiveMarker;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                       feedback->pose.orientation.z);

  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  // get TargetTrajectories
  const auto targetTrajectories = gaolPoseToTargetTrajectories_(position, orientation, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

void TargetTrajectoriesInteractiveMarker::process1Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const ros::Time timeStamp = ros::Time::now();

  std::vector<geometry_msgs::Point> obs_ponits;

        barriermsg.x = feedback->pose.position.x; 

        barriermsg.y = feedback->pose.position.y;

        barriermsg.z = feedback->pose.position.z;
    obs_ponits.push_back(barriermsg);

        // 发布消息  s
  // publishObstaclePose(timeStamp,obs_ponits);
obstacle_pub.publish(barriermsg);
        // obstacle_pub.publish(barriermsg);  

  
}

void TargetTrajectoriesInteractiveMarker::process2Feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const ros::Time timeStamp = ros::Time::now();

  std::vector<geometry_msgs::Point> obs_ponits;

        barriermsg.x = feedback->pose.position.x; 

        barriermsg.y = feedback->pose.position.y;

        barriermsg.z = feedback->pose.position.z;
    obs_ponits.push_back(barriermsg);

        // 发布消息  s
  // publishObstaclePose(timeStamp,obs_ponits);

     obstacle_pub.publish(barriermsg);  

  
}

void TargetTrajectoriesInteractiveMarker::publishObstaclePose(const ros::Time& timeStamp,const std::vector<geometry_msgs::Point> obstacelsPose){
  int id=0;

  for(auto obs:obstacelsPose){
    std::cout<<"visualized obstacles"<<std::endl;
    visualization_msgs::Marker obs_points;
    obs_points.header.frame_id = "world";
    obs_points.header.stamp=timeStamp;
    obs_points.id=id;
    id++;
    obs_points.action=visualization_msgs::Marker::ADD;
    obs_points.type=visualization_msgs::Marker::SPHERE;
    obs_points.scale.x=0.4;
    obs_points.scale.y=0.4;
    obs_points.scale.z=0.4;
    //set color
    obs_points.color.a=0.8;
    obs_points.color.r=0.1;
    obs_points.color.g=0.4;
    //its pose and orientation
    obs_points.pose.orientation=ros_msg_helpers::getOrientationMsg({0., 0., 0., 1.});
    obs_points.pose.position=obs;
    obs_points_.markers.push_back(obs_points);
  }

  obstacle_ballPublisher_.publish(obs_points_);
  // obs_points_.markers.clear();
}



}  // namespace ocs2
