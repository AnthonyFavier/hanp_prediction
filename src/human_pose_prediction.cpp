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
 #define PREDICT_SERVICE_NAME "predict_humans"

 #define DEFAULT_PREDICT_TIME 2.0 // seconds, time for predicting human position
 #define LOWER_SCALE 0.8 // human slow-down velocity multiplier
 #define HIGHER_SCALE 1.2 // human speed-up velocity multiplier
 #define ANGLE 0.1 // deviation angle for human position predictions

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
        predict_humans_srv_ = private_nh.advertiseService(PREDICT_SERVICE_NAME, &HumanPosePrediction::predictHumanPoses, this);
    }

    void HumanPosePrediction::setParams(std::vector<double> predict_scales, double predict_angle)
    {
        predict_scales_ = predict_scales;
        predict_angle_ = predict_angle;

        ROS_DEBUG_NAMED("human_pose_prediction", "parameters set: "
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

    }

    bool HumanPosePrediction::predictHumanPoses(hanp_prediction::PredictHumanPoses::Request& req,
        hanp_prediction::PredictHumanPoses::Response& res)
    {
        return true;
    }
}

// handler for something to do before killing the node
void sigintHandler(int sig)
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");

    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv)
{
    // starting the optotrack_person node
    ros::init(argc, argv, NODE_NAME);
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "started " << NODE_NAME << " node");

    // initiazling HANPHeadBehavior class
    hanp_prediction::HumanPosePrediction HumanPosePrediction();

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
