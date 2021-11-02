#ifndef SCAN_TO_POINT_CLOUD2_H
#define SCAN_TO_POINT_CLOUD2_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "laser_geometry/laser_geometry.h"

class ScanToPointCloud2
{
public:
    ScanToPointCloud2(ros::NodeHandle &n)
       {
          GetRosParam();
          InitRos(n);
       }
       ~ScanToPointCloud2()
       {

       }

    struct SequenceCheckBox
    {
      uint32_t seq = 0;
      uint32_t previous_seq = 0;
      uint32_t check_count = 0;
    };

    void GetRosParam();

    void InitRos(ros::NodeHandle &n);

    void ScanCallBack(const sensor_msgs::LaserScan &data);

    bool CheckCallbackSequence(SequenceCheckBox *check_box,uint32_t *seq);

    sensor_msgs::PointCloud2 _ScanToPointCloud2(sensor_msgs::LaserScan &data);

    void UpdatePointCloud2();

    void Update();

    void Publisher();

    void Spin();

private:

    struct CallBackDatas
    {
      sensor_msgs::LaserScan scan;
    };

    struct InitDatas
    {
      sensor_msgs::LaserScan scan;
    };

    struct TopicNames
    {
      std::string scan;
      std::string scan_to_point_cloud2;
    };

    CallBackDatas callback_datas_;
    InitDatas init_datas_;
    SequenceCheckBox sequence_check_box_scan_;
    TopicNames topic_names_;

    ros::Subscriber subscriber_scan_;

    ros::Publisher publisher_scan_to_point_cloud2_;

    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection laser_projector_;
    sensor_msgs::PointCloud2 point_cloud2_;

};

#endif // SCAN_TO_POINT_CLOUD2_H
