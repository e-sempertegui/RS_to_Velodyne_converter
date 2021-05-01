#include <smb_common/ToVelodynePCLConverter.h>

int main (int argc, char **argv){
    ros::init(argc, argv, "RSLidarToVelodynePCLConverter");

    ros::NodeHandle nh, private_nh("~");

    smb_calibration::RSLidartoVelodyneConverter converter(nh, private_nh);

    ros::spin();
}