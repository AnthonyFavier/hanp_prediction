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
 *                                  Harmish Khambhaita on Sat Sep 12 2015
 */

// defining constants
#define NODE_NAME "human_pose_prediction"

#define HUMAN_SUB_TOPIC "humans"
#define PREDICT_SERVICE_NAME "predict_2d_human_poses"

#define DEFAULT_PREDICT_TIME 2.0 // seconds, time for predicting human position
#define LOWER_SCALE 0.8 // human slow-down velocity multiplier
#define HIGHER_SCALE 1.2 // human speed-up velocity multiplier
#define ANGLE 0.1 // deviation angle for human position predictions

#define MIN_VELOBS_RADIUS 0.25 // meters
#define MAX_VELOBS_RADIUS 1.25 // meters
#define MAX_VELOBS_RADIUS_TIME 4.0 // seconds

#include <signal.h>

#include <hanp_prediction/human_pose_prediction.h>

namespace hanp_prediction
{
    // empty constructor and destructor
    HumanPosePrediction::HumanPosePrediction() {}
    HumanPosePrediction::~HumanPosePrediction() {}

    void HumanPosePrediction::initialize()
    {
        // get private node handle
        ros::NodeHandle private_nh("~/");

        // get parameters
        private_nh.param("human_sub_topic", human_sub_topic_, std::string(HUMAN_SUB_TOPIC));
        private_nh.param("predict_service_name", predict_service_name_, std::string(PREDICT_SERVICE_NAME));

        // initialize subscribers and publishers
        humans_sub_ = private_nh.subscribe(HUMAN_SUB_TOPIC, 1, &HumanPosePrediction::trackedHumansCB, this);

        // set default parameters
        std::vector<double> predict_scales = {LOWER_SCALE, 1.0, HIGHER_SCALE};
        setParams(predict_scales, ANGLE);

        // set-up dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<HumanPosePredictionConfig>(private_nh);
        dynamic_reconfigure::Server<HumanPosePredictionConfig>::CallbackType cb = boost::bind(&HumanPosePrediction::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // initialize services
        predict_humans_server_ = private_nh.advertiseService(PREDICT_SERVICE_NAME, &HumanPosePrediction::predictHumans, this);

        ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
    }

    void HumanPosePrediction::setParams(std::vector<double> predict_scales, double predict_angle)
    {
        predict_scales_ = predict_scales;
        predict_angle_ = predict_angle;

        ROS_DEBUG_NAMED(NODE_NAME, "parameters set: "
        "predict_scales=[%f, %f, %f], predict_angle=%f",
        predict_scales_[0], predict_scales_[1], predict_scales_[2], predict_angle_);
    }

    void HumanPosePrediction::reconfigureCB(HumanPosePredictionConfig &config, uint32_t level)
    {
        setParams({config.scale_lower, config.scale_nominal, config.scale_higher},
            config.angle);
    }

    void HumanPosePrediction::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        humans_ = tracked_humans;
    }

    bool HumanPosePrediction::predictHumans(hanp_prediction::HumanPosePredict::Request& req,
        hanp_prediction::HumanPosePredict::Response& res)
    {
        boost::function<bool(hanp_prediction::HumanPosePredict::Request& req,
            hanp_prediction::HumanPosePredict::Response& res)> prediction_function;

        switch(req.type)
        {
            case hanp_prediction::HumanPosePredictRequest::VELOCITY_SCALE:
                prediction_function = boost::bind(&HumanPosePrediction::predictHumansVelScale, this, _1, _2);
                break;
            case hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE:
                prediction_function = boost::bind(&HumanPosePrediction::predictHumansVelObs, this, _1, _2);
                break;
            default:
                ROS_ERROR_NAMED(NODE_NAME, "%s: unkonwn prediction type %d", NODE_NAME, req.type);
        }

        if(!prediction_function.empty())
        {
            return prediction_function(req, res);
        }
        else
        {
            return false;
        }
    }

    bool HumanPosePrediction::predictHumansVelScale(hanp_prediction::HumanPosePredict::Request& req,
        hanp_prediction::HumanPosePredict::Response& res)
    {
        // validate prediction time
        if((req.predict_times.size() < 1) || (req.predict_times[0] < 0))
        {
            ROS_ERROR_NAMED(NODE_NAME, "prediction time cannot be negative (give %f)", req.predict_times[0]);
            return false;
        }

        res.header.stamp = humans_.header.stamp;
        res.header.frame_id = humans_.header.frame_id;

        // get local refrence of humans
        auto humans = humans_.tracks;

        for(auto human : humans)
        {
            // TODO: filter by res.ids

            // get linear velocity of the human
            tf::Vector3 linear_vel(human.twist.twist.linear.x, human.twist.twist.linear.y, human.twist.twist.linear.z);

            // calculate variations in velocity of human
            std::vector<tf::Vector3> vel_variations;
            for(auto vel_scale : predict_scales_)
            {
                vel_variations.push_back(linear_vel.rotate(tf::Vector3(0, 0, 1), predict_angle_) * vel_scale);
                vel_variations.push_back(linear_vel.rotate(tf::Vector3(0, 0, 1), -predict_angle_) * vel_scale);
            }

            // calculate future human poses based on velocity variations
            hanp_prediction::PredictedPoses predicted_poses;
            predicted_poses.track_id = human.track_id;
            for(auto vel : vel_variations)
            {
                hanp_prediction::PredictedPose predicted_pose;
                predicted_pose.pose2d.x = human.pose.pose.position.x + vel[0] * req.predict_times[0];
                predicted_pose.pose2d.y = human.pose.pose.position.y + vel[1] * req.predict_times[0];
                predicted_pose.pose2d.theta = tf::getYaw(human.pose.pose.orientation);
                predicted_pose.radius = 0.0;
                predicted_poses.poses.push_back(predicted_pose);

                ROS_DEBUG_NAMED(NODE_NAME, "predected human (%d)"
                    " pose: x=%f, y=%f, theta=%f with vel scale %f",
                    human.track_id, predicted_pose.pose2d.x, predicted_pose.pose2d.y,
                    predicted_pose.pose2d.theta, vel);
            }

            res.predicted_humans.push_back(predicted_poses);
        }

        return true;
    }

    bool HumanPosePrediction::predictHumansVelObs(hanp_prediction::HumanPosePredict::Request& req,
        hanp_prediction::HumanPosePredict::Response& res)
    {
        res.header.stamp = humans_.header.stamp;
        res.header.frame_id = humans_.header.frame_id;

        // get local refrence of humans
        auto humans = humans_.tracks;

        for(auto human : humans)
        {
            // TODO: filter by res.ids

            // get linear velocity of the human
            tf::Vector3 linear_vel(human.twist.twist.linear.x, human.twist.twist.linear.y, human.twist.twist.linear.z);

            // calculate future human poses based on current velocity
            hanp_prediction::PredictedPoses predicted_poses;
            predicted_poses.track_id = human.track_id;

            for(auto predict_time : req.predict_times)
            {
                // validate prediction time
                if(predict_time < 0)
                {
                    ROS_ERROR_NAMED(NODE_NAME, "%s: prediction time cannot be negative (give %f)",
                        NODE_NAME, predict_time);
                    return false;
                }
                hanp_prediction::PredictedPose predicted_pose;
                predicted_pose.pose2d.x = human.pose.pose.position.x * predict_time;
                predicted_pose.pose2d.y = human.pose.pose.position.y * predict_time;
                predicted_pose.pose2d.theta = tf::getYaw(human.pose.pose.orientation);
                predicted_pose.radius = MIN_VELOBS_RADIUS + (MAX_VELOBS_RADIUS - MIN_VELOBS_RADIUS) * MAX_VELOBS_RADIUS / predict_time;
                predicted_poses.poses.push_back(predicted_pose);

                ROS_DEBUG_NAMED(NODE_NAME, "%s: predected human (%d)"
                    " pose: x=%f, y=%f, theta=%f, predict-time=%f", NODE_NAME, human.track_id,
                    predicted_pose.pose2d.x, predicted_pose.pose2d.y, predicted_pose.pose2d.theta,
                    predict_time);
            }

            res.predicted_humans.push_back(predicted_poses);
        }

        return true;
    }
}

// handler for something to do before killing the node
void sigintHandler(int sig)
{
    ROS_DEBUG_NAMED(NODE_NAME, "node %s will now shutdown", NODE_NAME);

    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv)
{
    // starting the optotrack_person node
    ros::init(argc, argv, NODE_NAME);
    ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

    // initiazling HANPHeadBehavior class
    hanp_prediction::HumanPosePrediction HumanPosePrediction;
    HumanPosePrediction.initialize();

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
