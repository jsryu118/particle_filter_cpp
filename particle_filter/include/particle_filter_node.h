#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include "utils.h"
#include "RangeLib.h"

class ParticleFilter {
public:
    ParticleFilter(ros::NodeHandle& nh);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void clickedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

private:
    void getOccupancyMap();
    void publish_tf(const std::vector<double>& x_y_yaw, const ros::Time& stamp);
    void visualize();
    void publishParticles(const std::vector<std::vector<double>>& particles);
    void publishScan(const std::vector<float>& angles, const std::vector<float>& ranges);
    void initializeParticlesPose(const geometry_msgs::Pose& pose);
    void initializeGlobal();
    void precomputeSensorModel();
    void motionModel(std::vector<std::vector<double>>& proposal_dist, const std::vector<double>& action);
    void sensorModel(std::vector<std::vector<double>>& proposal_dist, const std::vector<float>& obs, std::vector<double>& weights);
    void monteCarloLocalization(const std::vector<double>& action, const std::vector<float>& obs);
    std::vector<double> expectedPose();
    void update();
    int randomIndex(const std::vector<double>& weights);

    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particle_pub_;
    ros::Publisher fake_scan_pub_;
    ros::Publisher rect_pub_;
    ros::Publisher odom_pub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber click_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::mutex state_mutex_;

    std::default_random_engine generator_;
    Utils::CircularArray smoothing_;
    Utils::Timer timer_;
    ros::Time t_start_, t_propose_, t_motion_, t_sensor_, t_norm_;
    double t_total_;
    
    // topic parameters
    std::string map_topic_;
    std::string map_topic_frame_;
    std::string sub_scan_topic_;
    std::string sub_scan_topic_frame_;
    std::string sub_wheel_odom_topic_;
    std::string sub_imu_odom_topic_;

    // range method paramters
    std::unique_ptr<ranges::RangeMethod> range_method_;

    // ros parameters
    int odom_idx_;
    bool odom_fast_;
    bool no_initial_guess_;
    int angle_step_;
    int num_downsampled_ranges_;
    int max_particles_;
    int max_viz_particles_;
    float inv_squash_factor_;
    float max_range_meters_;
    int theta_discretization_;
    std::string which_rm_;
    int rangelib_var_;
    bool show_fine_timing_;
    bool publish_odom_;
    bool do_viz_;

    float z_short_;
    float z_max_;
    float z_rand_;
    float z_hit_;
    float sigma_hit_;
    float motion_dispersion_x_;
    float motion_dispersion_y_;
    float motion_dispersion_theta_;

    // Initialize random parameters
    float init_pose_sigma_x_;
    float init_pose_sigma_y_;
    float init_pose_sigma_yaw_;
    float init_point_sigma_x_;
    float init_point_sigma_y_;
    float init_point_sigma_yaw_;
    // float motion_dispersion_y_;
    // float motion_dispersion_theta_;

    int VAR_NO_EVAL_SENSOR_MODEL = 0;
    int VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1;
    int VAR_REPEAT_ANGLES_EVAL_SENSOR = 2;
    int VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3;
    int VAR_RADIAL_CDDT_OPTIMIZATIONS = 4;

    // 

    int max_range_px_;
    std::vector<double> odometry_data_;
    std::vector<float> laser_;
    int iters_;
    nav_msgs::MapMetaData map_info_;
    bool map_initialized_;
    bool lidar_initialized_;
    bool odom_initialized_;
    std::vector<double> last_pose_;
    std::vector<double> laser_angles_;
    std::vector<float> downsampled_angles_;
    std::vector<double> downsampled_ranges_;
    // ranges::OMap OMap;
    ranges::OMap* omap_ptr_;
    std::vector<std::vector<bool>> permissible_region_;

    ros::Time last_time_;
    ros::Time last_stamp_;
    bool first_sensor_update_;
    std::vector<std::vector<double>> local_deltas_;
    std::vector<std::vector<float>> queries_;
    std::vector<float> fake_ranges_;
    std::vector<float> tiled_angles_;
    std::vector<std::vector<double>> sensor_model_table_;

    std::vector<double> inferred_pose_;
    std::vector<int> particle_indices_;
    std::vector<std::vector<double>> particles_;
    std::vector<double> weights_;
};

#endif // PARTICLE_FILTER_H
