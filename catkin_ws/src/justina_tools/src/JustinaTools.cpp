#include "justina_tools/JustinaTools.h"

bool JustinaTools::is_node_set = false;
tf::TransformListener* JustinaTools::tf_listener;
std::string JustinaTools::pathDeviceScript;

void JustinaTools::initialize()
{
    std::cout << "JustinaTools.->Setting ros node..." << std::endl;
    tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "laser_link", ros::Time(0), ros::Duration(10.0));
    pathDeviceScript = ros::package::getPath("justina_tools");
}

void JustinaTools::laserScanToPclWrtRobot(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    tf::StampedTransform transformTf;
    tf_listener->lookupTransform("base_link","laser_link", ros::Time(0), transformTf);
    Eigen::Affine3d transformEigen;
    tf::transformTFToEigen(transformTf, transformEigen);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLaser(new pcl::PointCloud<pcl::PointXYZ>);
    cloudLaser->width = readings->ranges.size();
    cloudLaser->height = 1;
    cloudLaser->points.resize(cloudLaser->width * cloudLaser->height);
    for (size_t i = 0; i < cloudLaser->points.size(); ++i)
    {
        cloudLaser->points[i].x = readings->ranges[i] * cos(readings->angle_min + i * readings->angle_increment);
        cloudLaser->points[i].y = readings->ranges[i] * sin(readings->angle_min + i * readings->angle_increment);
    }
    pcl::transformPointCloud(*cloudLaser, *cloud, transformEigen);
}

void JustinaTools::PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
    int offset_x = 0;
    int offset_y = 4;
    int offset_z = 8;
    int offset_bgr  = 12;
    for(int i=0; i < pc_msg.fields.size(); i++)
        if (pc_msg.fields[i].name == "x")
            offset_x = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "y")
            offset_y = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "z")
            offset_z = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgb")
            offset_bgr = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgba")
            offset_bgr = pc_msg.fields[i].offset;

    bgr_dest = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_x];
            float* y = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_y];
            float* z = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_z];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 1];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 2];
        }
}

void JustinaTools::PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
    int offset_x = 0;
    int offset_y = 4;
    int offset_z = 8;
    int offset_bgr  = 12;
    for(int i=0; i < pc_msg->fields.size(); i++)
        if (pc_msg->fields[i].name == "x")
            offset_x = pc_msg->fields[i].offset;
        else if (pc_msg->fields[i].name == "y")
            offset_y = pc_msg->fields[i].offset;
        else if (pc_msg->fields[i].name == "z")
            offset_z = pc_msg->fields[i].offset;
        else if (pc_msg->fields[i].name == "rgb")
            offset_bgr = pc_msg->fields[i].offset;
        else if (pc_msg->fields[i].name == "rgba")
            offset_bgr = pc_msg->fields[i].offset;
    
    bgr_dest = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_x];
            float* y = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_y];
            float* z = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_z];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_bgr    ];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_bgr + 1];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + offset_bgr + 2];
        }
}

bool JustinaTools::transformPoint(std::string src_frame, float inX, float inY, float inZ, std::string dest_frame, float& outX, float& outY, float& outZ)
{
    tf::StampedTransform transformTf;
    tf_listener->lookupTransform(dest_frame,src_frame, ros::Time(0), transformTf);
    tf::Vector3 v(inX, inY, inZ);
    v = transformTf * v;
    outX = v.x();
    outY = v.y();
    outZ = v.z();
    return true;
}

bool JustinaTools::transformPose(std::string src_frame, float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw,
                                 std::string dest_frame,float& outX,float& outY,float& outZ, float& outRoll, float& outPitch,
                                 float& outYaw)
{
    std::cout << "JustinaTools.->Trans: "<<inX<<" "<<inY<<" "<<inZ<<" " << inRoll << " " << inPitch << " " << inYaw << std::endl;
    tf::StampedTransform ht;
    tf_listener->lookupTransform(dest_frame,src_frame, ros::Time(0), ht);
    tf::Quaternion q;
    q.setRPY(inRoll, inPitch, inYaw);
    tf::Vector3 p(inX, inY, inZ);
    p = ht * p;
    q = ht * q;
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);
    outX = p.x();
    outY = p.y();
    outZ = p.z();
    outRoll = (float)dRoll;
    outPitch = (float)dPitch;
    outYaw = (float)dYaw;
    
    std::cout << "JustinaTools.->Trans: "<<outX<<" "<<outY<<" "<<outZ<<" "<<outRoll<<" "<<outPitch<< " " << outYaw << std::endl;
    return true;
}

bool JustinaTools::transformPose(std::string src_frame, std::vector<float>& xyz_rpy_in, std::string dest_frame, std::vector<float>& xyz_rpy_out)
{
    if(xyz_rpy_in.size() < 6)
    {
        std::cout << "JustinaTools.->Cannot transform pose. vector<float> must have 6 values: xyz and roll pitch yaw" << std::endl;
        return false;
    }
    if(xyz_rpy_out.size() < 6)
    {
        xyz_rpy_out.clear();
        for(int i=0; i< 6; i++)
            xyz_rpy_out.push_back(0);
    }
    return transformPose(src_frame, xyz_rpy_in[0] ,xyz_rpy_in[1] , xyz_rpy_in[2] , xyz_rpy_in[3] , xyz_rpy_in[4] , xyz_rpy_in[5],
                         dest_frame,xyz_rpy_out[0],xyz_rpy_out[1], xyz_rpy_out[2], xyz_rpy_out[3], xyz_rpy_out[4], xyz_rpy_out[5]);

}


std::string JustinaTools::startRecordSpeach(std::string competition, std::string test)
{
    std::cout << "JustinaTools.->initRecordSpeech init record speech" << std::endl;
    std::stringstream ss;
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    ss << homedir;
    std::cout << "JustinaTools.->initRecordSpeech homedir: " << ss.str() << std::endl;
    ss << "/" << competition;
    boost::filesystem::path dir(ss.str().c_str());
    if(boost::filesystem::create_directory(dir))
        std::cout << "The directory already have created" << ss.str() << std::endl;
    ss << "/" << test;
    dir = boost::filesystem::path(ss.str().c_str());
    if(boost::filesystem::create_directory(dir))
        std::cout << "The directory already have created" << ss.str() << std::endl;
    ss << "/rosbags";
    dir = boost::filesystem::path(ss.str().c_str());
    if(boost::filesystem::create_directory(dir))
        std::cout << "The directory already have created" << ss.str() << std::endl;
    ss << "/" << "recoAudio";
    std::string filename = ss.str();
    ss.str("");
    ss << pathDeviceScript << "/src/startRecordSpeech.sh " << filename;
    std::cout << system(ss.str().c_str()) << std::endl;
    return filename;
}

void JustinaTools::stopRecordSpeach()
{
    std::stringstream ss;
    ss << pathDeviceScript << "/src/tools/justina_tools/src/stopRecordSpeech.sh ";
    std::cout << system(ss.str().c_str()) << std::endl;
}

void JustinaTools::removeAudioRecord(std::string path)
{
    std::stringstream ss;
    ss << "rm " << path;
    std::cout << system(ss.str().c_str()) << std::endl;
}
