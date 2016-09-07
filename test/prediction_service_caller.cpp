/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Wed Jun 22 2016
 */

#define PREDICT_SERVICE_NAME "/human_pose_prediction/predict_human_poses"
#define PUBLISH_MARKERS_SRV_NAME                                               \
  "/human_pose_prediction/publish_prediction_markers"
#define PERSISTANT_CONNECTION true
#define PUBLISH_MARKERS true
#define LOOP_RATE 10.0
#define STRATEGY_CHANGE_TIME 10.0
#define PREDICT_TRAJ_SIZE 5
#define PREDICT_TRAJ_TIME 3.5

#include <ros/ros.h>
#include <hanp_prediction/HumanPosePredict.h>
#include <std_srvs/SetBool.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "prediction_service_caller");

  ros::NodeHandle nh("~");

  std::string predict_service_name, publish_markers_srv_name;
  bool persistant_connection, publish_markers;
  double loop_rate_time, predict_time;
  int traj_size;

  // get parameters from server
  nh.param<std::string>("predict_service_name", predict_service_name,
                        PREDICT_SERVICE_NAME);
  nh.param<std::string>("publish_markers_service_name",
                        publish_markers_srv_name, PUBLISH_MARKERS_SRV_NAME);
  nh.param<bool>("persistant_connection", persistant_connection,
                 PERSISTANT_CONNECTION);
  nh.param<bool>("publish_markers", publish_markers, PUBLISH_MARKERS);
  nh.param<double>("loop_rate", loop_rate_time, LOOP_RATE);
  nh.param<int>("predict_traj_size", traj_size, PREDICT_TRAJ_SIZE);
  nh.param<double>("predict_traj_time", predict_time, PREDICT_TRAJ_TIME);

  ros::Rate loop_rate(loop_rate_time);

  // set up service
  auto predict_humans_client =
      nh.serviceClient<hanp_prediction::HumanPosePredict>(
          predict_service_name, persistant_connection);
  auto publish_markers_client = nh.serviceClient<std_srvs::SetBool>(
      publish_markers_srv_name, persistant_connection);

  hanp_prediction::HumanPosePredict predict_srv;
  for (double i = 1.0; i <= traj_size; ++i) {
    predict_srv.request.predict_times.push_back(predict_time * (i / traj_size));
  }
  predict_srv.request.type =
      hanp_prediction::HumanPosePredictRequest::VELOCITY_SCALE;

  std_srvs::SetBool publish_markers_srv;
  publish_markers_srv.request.data = publish_markers;

  auto change_counter = STRATEGY_CHANGE_TIME * LOOP_RATE;
  while (ros::ok()) {
    ROS_DEBUG("calling prediction service");
    if (!publish_markers_client ||
        !publish_markers_client.call(publish_markers_srv)) {
      ROS_WARN_THROTTLE(
          4, "failed to call %s service, is prediction server running?",
          publish_markers_srv_name.c_str());
      publish_markers_client = nh.serviceClient<std_srvs::SetBool>(
          publish_markers_srv_name, persistant_connection);
    }
    if (!predict_humans_client || !predict_humans_client.call(predict_srv)) {
      ROS_WARN_THROTTLE(
          4, "failed to call %s service, is prediction server running?",
          predict_service_name.c_str());
      predict_humans_client =
          nh.serviceClient<hanp_prediction::HumanPosePredict>(
              predict_service_name, persistant_connection);
    }

    ROS_DEBUG("wating some time");
    loop_rate.sleep();

    ros::spinOnce();

    change_counter -= 1.0;
    if (change_counter < 0) {
      switch (predict_srv.request.type) {
      case hanp_prediction::HumanPosePredictRequest::VELOCITY_SCALE:
        predict_srv.request.type =
            hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;
        ROS_INFO("predictoin method switched to VELOCITY_OBSTACLE");
        break;
      case hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE:
        predict_srv.request.type =
            hanp_prediction::HumanPosePredictRequest::VELOCITY_SCALE;
        ROS_INFO("predictoin method switched to VELOCITY_SCALE");
        break;
      default:
        predict_srv.request.type =
            hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;
        ROS_INFO("predictoin method switched to VELOCITY_OBSTACLE");
      }

      change_counter = STRATEGY_CHANGE_TIME * LOOP_RATE;
    }
  }

  return 0;
}
