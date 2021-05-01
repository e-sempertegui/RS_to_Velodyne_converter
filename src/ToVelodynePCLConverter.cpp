#include <smb_common/ToVelodynePCLConverter.h>

namespace smb_calibration
{
    
RSLidartoVelodyneConverter::RSLidartoVelodyneConverter(const ros::NodeHandle &nh,
                                                       const ros::NodeHandle &private_nh):
                                                       nh_(nh),
                                                       private_nh_(private_nh){

    pcl_sub_ = nh_.subscribe("pcl_in", 10, &RSLidartoVelodyneConverter::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_out", 10);
    ROS_INFO_ONCE("Initialized PCL Converter...");
}

RSLidartoVelodyneConverter::~RSLidartoVelodyneConverter(){
}

void RSLidartoVelodyneConverter::pclCallback(const sensor_msgs::PointCloud2Ptr &msg){

    pcl::PointCloud<Velodyne::Point> pcl;
    pcl::fromROSMsg(*msg, pcl);

    int point_counter = 0;
    int pcl_width = msg->width;

    // Ring assignment is based on the assumption that ordering of PCL points follows the order of the received ROS msg
    // and that these are ordered pcls starting from the lowest ring (0) to the highest ring (15)
    for (Velodyne::Point& point : pcl.points){
        point.ring = point_counter/pcl_width;
        point_counter++;
    }

    pcl_pub_.publish(pcl);

}

} // namespace smb_calibration
