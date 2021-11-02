#include "scan_to_point_cloud2.h"

void ScanToPointCloud2::ScanCallBack(const sensor_msgs::LaserScan &data)
{
  callback_datas_.scan = data;
}

bool ScanToPointCloud2::CheckCallbackSequence(SequenceCheckBox *check_box, uint32_t *seq)
{
    check_box->seq = *seq;

    if(check_box->seq == check_box->previous_seq)
    {
        check_box->check_count++;

        if(check_box->check_count > 99)
        {
            check_box->check_count = 100;
            return false;
        }
    }
    else
    {
        check_box->check_count= 0;
    }

    check_box->previous_seq = check_box->seq;

    return true;
}

sensor_msgs::PointCloud2 ScanToPointCloud2::_ScanToPointCloud2(sensor_msgs::LaserScan &data)
{
  sensor_msgs::PointCloud2 point_cloud2;

  ros::Time now_time = ros::Time::now();
  point_cloud2.header.stamp = now_time;
  data.header.stamp = now_time;

  if(!tf_listener_.waitForTransform(data.header.frame_id,data.header.frame_id,data.header.stamp,ros::Duration(1.0)))
  {
    return point_cloud2;
  }

  laser_projector_.transformLaserScanToPointCloud(data.header.frame_id,data,
          point_cloud2,tf_listener_);

  return point_cloud2;
}

void ScanToPointCloud2::GetRosParam()
{
    ros::NodeHandle n("~");
    n.param<std::string>("TopicNameScan", topic_names_.scan, "scan");
    n.param<std::string>("TopicNameScanToPointCloud2", topic_names_.scan_to_point_cloud2, "scan_to_point_cloud2");
}

void ScanToPointCloud2::InitRos(ros::NodeHandle &n)
{
    subscriber_scan_ = n.subscribe(topic_names_.scan, 1, &ScanToPointCloud2::ScanCallBack, this);
    publisher_scan_to_point_cloud2_ = n.advertise<sensor_msgs::PointCloud2>(topic_names_.scan_to_point_cloud2,1000);
}

void ScanToPointCloud2::UpdatePointCloud2()
{
  point_cloud2_ = _ScanToPointCloud2(callback_datas_.scan);
}

void ScanToPointCloud2::Update()
{
  UpdatePointCloud2();
}

void ScanToPointCloud2::Publisher()
{
  publisher_scan_to_point_cloud2_.publish(point_cloud2_);
}

void ScanToPointCloud2::Spin()
{
  Update();
  if(CheckCallbackSequence(&sequence_check_box_scan_, &callback_datas_.scan.header.seq) == false)
  {
      callback_datas_.scan = init_datas_.scan;
  }
  Publisher();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_point_cloud2_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(60);

  ScanToPointCloud2 scan_to_point_cloud2(n);

  ROS_INFO("scan_to_point_cloud2_node Open");

  while (ros::ok())
  {
    scan_to_point_cloud2.Spin();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("scan_to_point_cloud2_node Close");

  return 0;
}
