#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Point, PoseStamped
from hanp_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegmentType
from hanp_prediction.msg import PredictedGoal
from std_srvs.srv import SetBool, Trigger, TriggerResponse
import tf
EPS = 1e-12

class PredictGoal(object):
    def __init__(self, human_num=1):
        self.human_num = human_num

        # Map_LAAS
        self.goals_x = [1.0, 3.15, 10.2, 7.90, 7.8, 3.42, 4.72, 10.6, 1.0, 0.65]
        self.goals_y = [0.9, 3.2, -3.98, 5.1, 9.98, 9.48, 17.68, 15.8, 15.8, 8.50]
        self.goal_num = 10

        #Map_new
        # self.goals_x = [1.5,1.5,1.5,1.5,1.5,7.5,25,42,42,41.5,42,37,22,15.5,28.5,37,23.5,10.5,15.5,31.5,20,25.5,7]
        # self.goals_y = [45,15,30,60,87,87,81.5,81.5,66,41.5,22,3,3,12.5,12.5,20.5,21.5,28.5,39.5,47,53,59,59]

        self.predicted_goal = PoseStamped()
        self.last_idx = 0
        self.changed = False
        self.current_poses = [[] for i in range(self.human_num)]
        self.prev_poses = [[] for i in range(self.human_num)]
        self.mv_nd = multivariate_normal(mean=0,cov=0.1)
        self.theta_phi = [[0]*self.goal_num for i in range(self.human_num)]
        self.window_size = 10
        self.probability_goal = [np.array([1.0/self.goal_num]*self.goal_num) for i in range(self.human_num)]
        self.probability_goal_window = [np.array([[1.0/self.goal_num]*self.goal_num]*self.window_size) for i in range(self.human_num)]
        self.done = False
        self.itr = 0

        NODE_NAME = "human_goal_predict"
        rospy.init_node(NODE_NAME)
        self.humans_sub_ = rospy.Subscriber("/tracked_humans",TrackedHumans,self.tracked_humansCB)
        self.goal_pub_ = rospy.Publisher(NODE_NAME+"/predicted_goal",PredictedGoal, queue_size=2)
        self.goal_srv_ = rospy.Service("goal_changed", Trigger, self.goal_changed)
        rospy.spin()

    def tracked_humansCB(self,msg):
        self.prev_poses = self.current_poses
        self.current_poses = [[] for i in range(self.human_num)]

        for human in msg.humans:
            for segment in human.segments:
                if segment.type == TrackedSegmentType.TORSO:
                    # print((self.human_num))
                    self.current_poses[human.track_id-1].append(segment.pose.pose)
        if not self.done:
            self.prev_poses = self.current_poses

        for i in range(0,len(self.current_poses[0])):
            # print(self.current_poses[0][i])
            diff = np.linalg.norm([self.current_poses[0][i].position.x - self.prev_poses[0][i].position.x, self.current_poses[0][i].position.y - self.prev_poses[0][i].position.y])

            if diff > EPS or not self.done:
                dist = []
                for j in range(0,len(self.goals_x)):
                    # print(self.current_poses[i])
                    vec1 = np.array([self.goals_x[j],self.goals_y[j],0.0]) - np.array([self.current_poses[0][i].position.x,self.current_poses[0][i].position.y,0.0])  #Vector from current position to a goal
                    # print(self.current_poses[i][0].orientation)
                    rotation = (self.current_poses[0][i].orientation.x,self.current_poses[0][i].orientation.y,self.current_poses[0][i].orientation.z,self.current_poses[0][i].orientation.w)
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(rotation)
                    unit_vec = np.array([np.cos(yaw), np.sin(yaw),0.0])
                    self.theta_phi[i][j] = (np.arccos(np.dot(vec1,unit_vec)/np.linalg.norm(vec1)))
                    dist.append(np.linalg.norm([self.current_poses[0][i].position.x - self.goals_x[j],self.current_poses[0][i].position.y - self.goals_y[j]]))

                self.probability_goal_window[i][self.itr] = self.mv_nd.pdf(np.array(self.theta_phi[i]));

                self.probability_goal[i] = np.array([1.0]*self.goal_num)
                for k in range(0,len(self.probability_goal_window[i])):
                    gf = np.exp((k-self.window_size)/5)
                    self.probability_goal[i] =  np.power(self.probability_goal_window[i][k],gf)* np.array(self.probability_goal[i]) # Linear prediction of goal
                # print(self.probability_goal[i])

                for ln in range(0,len(self.goals_x)):
                    self.probability_goal[i][ln] = (1/dist[ln])*self.probability_goal[i][ln];

                self.probability_goal[i] = (self.probability_goal[i]-np.min(self.probability_goal[i]))/(np.max(self.probability_goal[i])-np.min(self.probability_goal[i]))

                # print(sum(self.probability_goal[i]))

                self.itr = self.itr + 1
                if self.itr == self.window_size:
                    self.itr = 0

                self.done = True

        self.predict_goal()


    def predict_goal(self):
        idx = 0
        max_prob = 0.0
        p_goal = PredictedGoal()

        for i in range(0,len(self.current_poses[0])):
            for j in range(0,len(self.goals_x)):
                if(max_prob<self.probability_goal[i][j]):
                    idx = j
                    max_prob = self.probability_goal[i][j]

            self.predicted_goal.header.stamp = rospy.Time.now()
            self.predicted_goal.header.frame_id = 'map'
            self.predicted_goal.pose.position.x = self.goals_x[idx]
            self.predicted_goal.pose.position.y = self.goals_y[idx]
            self.predicted_goal.pose.position.z = 0.0
            self.predicted_goal.pose.orientation = self.current_poses[0][i].orientation

            if self.last_idx != idx:
                p_goal.changed = True
                self.changed = True

        self.last_idx = idx
        p_goal.goal = self.predicted_goal
        self.goal_pub_.publish(p_goal)


    def goal_changed(self,req):
        if self.changed:
            self.changed = False
            return TriggerResponse(True,"Goal Changed")
        return TriggerResponse(False, "Goal not changed")


predict_srv = PredictGoal(3)

# void HumanPosePrediction::trackedHumansCB(
#     const hanp_msgs::TrackedHumans &tracked_humans) {
#   ROS_INFO_ONCE_NAMED(NODE_NAME, "hanp_prediction: received humans");
#   tracked_humans_ = tracked_humans;
# }

# map_size = [10,10] #10x10 m^2 grid
# cell_size = [0.1,0.1] # Each cell is 0.1x0.1 m^2
# s_x = cell_size[0]
# s_y = cell_size[1]
#
# grid = np.ones((int(map_size[0]/cell_size[0]),int(map_size[1]/cell_size[1]),3))
# # print(grid)
# print(grid.shape)
#
# for i in range(0,grid.shape[0]):
#     for j in range(0,grid.shape[1]):
#         grid[0,j] = 0.0;
#         grid[i,0] = 0.0;
#         grid[grid.shape[0]-1,j] = 0.0;
#         grid[i,grid.shape[1]-1] = 0.0;
#
#
# dm = np.array([[4,1,6,7,2],[3,6,8,3,9]])
# grid_dm = dm/s_x
#
# # for i in range(0/
# grid[grid_dm[0].astype(int),grid_dm[1].astype(int),0] = 1.0
# grid[grid_dm[0].astype(int),grid_dm[1].astype(int),1] = 0.0
# grid[grid_dm[0].astype(int),grid_dm[1].astype(int),2] = 0.0
#
#
# human_pose_grid = np.array([random.randint(1,99), random.randint(1,99)])
# human_pose = human_pose_grid*s_x;
#
# grid[human_pose_grid[0],human_pose_grid[1],2] = 1.0
# grid[human_pose_grid[0],human_pose_grid[1],0] = 0.0
# grid[human_pose_grid[0],human_pose_grid[1],1] = 0.0
#
# human_orient =  random.random()*2*np.pi
# human_dir_vec = np.array([np.cos(human_orient),np.sin(human_orient)])
#
# # grid[int(human_dir_vec[0]),int(human_dir_vec[1]),1] = 1.0
# # grid[int(human_dir_vec[0]),int(human_dir_vec[1]),0] = 0.0
# # grid[int(human_dir_vec[0]),int(human_dir_vec[1]),2] = 0.0
#
# # Move human pose
# human_move = np.array([human_pose[0]+np.cos(human_orient),human_pose[1]+np.sin(human_orient)])
#
# dm_vect_arr = np.array([dm[0]-human_pose[0],dm[1]-human_pose[1]])
# dm_vect_arr2 = np.array([dm[0]-human_move[0],dm[1]-human_move[1]])
#
#
# v = []
# v1_theta = []
# v2_theta = []
# for i in range(0,dm.shape[1]):
#     v1 = np.array([dm_vect_arr[0,i],dm_vect_arr[1,i]])
#     v2 = np.array([dm_vect_arr2[0,i],dm_vect_arr2[1,i]])
#     v1_theta.append(np.arccos(np.dot(human_dir_vec,v1)/np.linalg.norm(v1)))
#     v2_theta.append(np.arccos(np.dot(human_dir_vec,v2)/np.linalg.norm(v2)))
#
# # print('v1',v1)
# nd = multivariate_normal(mean=0,cov=0.1)
# grid[int(human_move[0]/s_x),int(human_move[1]/s_x),1] = 1.0
# grid[int(human_move[0]/s_x),int(human_move[1]/s_x),0] = 0.0
# grid[int(human_move[0]/s_x),int(human_move[1]/s_x),2] = 0.0
#
#
# print(nd.pdf(v1_theta)*nd.pdf(v2_theta))
# # print(nd.pdf(v1_theta[0]))
#
# # plt.imshow(grid)
# # plt.show()
