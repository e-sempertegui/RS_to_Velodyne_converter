#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

/// Velodyne point type containing ring information like in original guindal code and in velodyne driver
namespace Velodyne {
	struct Point {
		PCL_ADD_POINT4D; ///< quad-word XYZ
		float intensity; ///< laser intensity reading
        uint16_t ring; ///< laser ring number
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< ensure proper alignment
	} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
	Velodyne::Point, (float, x, x) (float, y, y) (float, z, z)
		(float, intensity, intensity) (uint16_t, ring, ring));


namespace smb_calibration
{

class RSLidartoVelodyneConverter{
    public:
        RSLidartoVelodyneConverter(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
        ~RSLidartoVelodyneConverter();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher pcl_pub_;
        ros::Subscriber pcl_sub_;

        void pclCallback (const sensor_msgs::PointCloud2Ptr &msg);
};

} // namespace smb_calibration
