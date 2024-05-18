#ifndef POINTCLOUDMAPPING_H_
#define POINTCLOUDMAPPING_H_

#include "System.h"
#include "Settings.h"


#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>


#include <opencv2/core.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "Atlas.h"
#include "KeyFrame.h"
#include <list>
#include <Eigen/Dense>


namespace ORB_SLAM3{


class PointCloudMapping
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>
    //  ORB3_dense 构造函数
    PointCloudMapping(double resolution_,double meank_=10,double stdthresh_=1,double unit_=1000);
    ~PointCloudMapping();


    // 插入keyframe，并且更新地图
    // rgbd
    void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);


    // rgbd稠密建图
    PointCloud::Ptr GetPointCloud(KeyFrame *kf, cv::Mat &left, cv::Mat &depth);

    // void insertKeyFrame(ORB_SLAM3::KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth, std::vector<ORB_SLAM3::KeyFrame*> vpKFs);
    // void updatecloud(ORB_SLAM3::Map& curMap);
    // void generatePointCloud(ORB_SLAM3::KeyFrame *kf);


    // 关闭本线程并调用save保存点云地图
    void shutdown();

    // 展示
    void viewer();

    // 保存点云图为.pcd文件
    void save();

     // add the Coordinate which stand for the pose into the pcl viewer
    /**
     * @brief
     * @param viewer the pcl viewer
     * @param pose the pose of the camera
     * @param prefix the Coordinate name
     */


    void addCoordinateSystem(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const Eigen::Matrix4f &pose, const std::string &prefix);


private:
    // global map
    PointCloud::Ptr globalMap;

    // 单一线程 , 用 std::move
    std::unique_ptr<thread> viewerThread;

    // 关机标志
    bool shutDownFlag = false;
    std::mutex shutDownMutex;


    // 线程  更新关键帧条件变量
    std::condition_variable keyFrameUpdated;
    std::mutex keyFrameUpdateMutex;

    // 生成点云的数据
    std::vector<KeyFrame *> keyframes;//关键帧
    std::vector<cv::Mat> colorImgs;// rgb图像
    std::vector<cv::Mat> depthImgs;// 深度

    // 线程锁
    std::mutex keyframeMutex;
    uint16_t lastKeyframeSize = 0;

   // 降采样
    double resolution = 0.04;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;

    // 离群点去除
    double meank = 10;
    double stdthresh = 1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // 单位
    double unit = 1000; //defalut mm


    // // 数据成员和成员函数声明
    // double mResolution;
    // int mCx, mCy, mFx, mFy;
    // bool mbShutdown, mbFinish;
    // pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> statistical_filter;
    // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> mPointCloud;
    // std::thread viewerThread;
    // std::mutex mKeyFrameMtx, shutDownMutex, updateMutex, keyframeMutex, mMutexGlobalMap;
    // std::condition_variable mKeyFrameUpdatedCond;
    // std::list<ORB_SLAM3::KeyFrame*> mvKeyFrames;
    // std::vector<cv::Mat> mvColorImgs, mvDepthImgs;
    // std::vector<ORB_SLAM3::KeyFrame*> keyframes;
    // std::vector<ORB_SLAM3::KeyFrame*> currentvpKFs;
    // bool bStop, mabIsUpdating;

    // // ROS相关
    // ros::Publisher pclPoint_pub, pclPoint_local_pub, octomap_pub;
    // sensor_msgs::PointCloud2 pcl_point, pcl_local_point;
    // pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_local_kf, pcl_cloud_kf;
};



}
#endif // POINTCLOUDMAPPING_H_

