#include "PointCloudMapping.h"
#include <pcl/common/projection_matrix.h>

#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"

namespace ORB_SLAM3{

pcl::PointXYZ pcl_transform(const Eigen::Matrix4f &pose, const Eigen::Vector3f &p)
{
    pcl::PointXYZ pcl_p;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.matrix() = pose;
    Eigen::Vector3f e_p = transform * p;
    return pcl::PointXYZ(e_p(0), e_p(1), e_p(2));
}

PointCloudMapping(double resolution_,double meank_,double stdthresh_):resolution(resolution_),meank(meank_),stdthresh(stdthresh_),unit(1000)
{
    std::cout<<"initializa with disp images !"<<std::endl;
    // 体素采样
    std::cout << "the resolution of Point cloud Voxel filter : " << resolution << std::endl;
    // this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);
    // 离群滤波
    std::cout << "the Point cloud Outlier filter params :   \n"
              << "meank : " << meank << std::endl
              << "stdthresh :" << stdthresh << std::endl;
    // this->meank = meank_;
    // this->stdthresh = stdthresh_;
    sor.setMeanK(meank);
    sor.setStddevMulThresh(stdthresh);
    // 单位
    std::cout << "the Unit of Point cloud : " << unit << std::endl;
    // this->unit = unit_;
    // 全局点云
    globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 线程
    viewerThread = make_unique<thread>(bind(&PointCloudMapping::viewer, this));

}

void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth,int idk,vector<KeyFrame*> vpKFs )
{
    cout << "receive a keyframe, id = "<<idk <<"   "<< kf->mnId << endl;
    unique_lock<mutex> lck(keyframeMutex);
    // 数据
    currentvpKFs = vpKFs;
    keyframes.push_back(kf);
    // colorImgs.push_back(color.clone());
    // depthImgs.push_back(depth.clone());


    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );
    pointcloude.pcE = GetPointCloud(kf,color,depth);
    pointcloud.push_back(pointcloude);

    // 线程阻塞
    keyFrameUpdated.notify_one();
}
// rgbd稠密建图
PointCloud::Ptr GetPointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    PointCloud::Ptr tmp(new PointCloud());
    // depth convert
    if (depth.type() != CV_32F)
        depth.convertTo(depth, CV_32F);
    // point cloud convert
    for (int m = 0; m < depth.rows; m += 1)
    {
        for (int n = 0; n < depth.cols; n += 1)
        {
            float d = depth.ptr<float>(m)[n];
            if (d / unit < 0.01 || d / unit> 10.0)
                continue;
            pcl::PointXYZRGB p;
            p.z = d / unit;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n * 3 + 2];
            p.g = color.ptr<uchar>(m)[n * 3 + 1];
            p.r = color.ptr<uchar>(m)[n * 3 ];

            tmp->points.push_back(p);
            }
    }
    // get the transform point cloud
    // PointCloud::Ptr cloud(new PointCloud());
    // pcl::transformPointCloud(*tmp, *cloud, ORBSLAM3::Converter::toMatrix4d(kf->GetPoseInverse()));
    // cloud->is_dense = false;
    // // print
    // // cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
    // return cloud;
    return tmp;
}
// 关闭本线程并调用save保存点云地图
void shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
    save();
}
void viewer()
{
    // std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer("Point cloud Viewer"));
    std::shared_ptr<pcl::visualization::CloudViewer> cloud_viewer(new pcl::visualization::CloudViewer("Point Cloud"));
    while (1)
    {
        {
            // shut down lock
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            // 这里等待 inseart key frame 函数的条件变量
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }
        // keyframe is updated
        size_t N = 0;
        {
            // keyframes lock
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
            continue;
        }
        if(lastKeyframeSize == N)
            cloudbusy = false;
        //cout<<"待处理点云个数 = "<<N<<endl;
        cloudbusy = true;


        // PointCloud::Ptr all_p(new PointCloud());

        // check the sensor so that decide the Point cloud generation method
        // rgbd

        for (size_t i = lastKeyframeSize; i < N; i++)
        {
            PointCloud::Ptr p;
            // p = GetPointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            // addCoordinateSystem(pcl_viewer, keyframes[i]->GetPoseInverse().matrix(), std::to_string(i));
            (*globalMap) += *p;
        }


        PointCloud::Ptr tmp(new PointCloud());

        // outiler filter
        sor.setInputCloud(globalMap);
        sor.filter(*tmp);

        // voxel filter
        voxel.setInputCloud(globalMap);
        voxel.filter(*globalMap);



        // print
        std::cout << "the num of points : " << globalMap->points.size() << std::endl;
        // add point cloud
        // pcl_viewer->removePointCloud();
        // pcl_viewer->addPointCloud(globalMap);
        // pcl_viewer->spinOnce(1000);


        cloud_viewer->showCloud(globalMap);
        lastKeyframeSize = N;
        cloudbusy = false;
    }
}
void save()
{
    if (this->globalMap != nullptr)
        pcl::io::savePCDFile("./PointCloudmapping.pcd", *globalMap);
    std::cout<<"点云地图已保存"<<std::endl;
}
void addCoordinateSystem(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const Eigen::Matrix4f &pose, const std::string &prefix)
{
    Eigen::Vector3f o(0, 0, 0);
    Eigen::Vector3f x_axis(this->unit * 0.25, 0, 0);
    Eigen::Vector3f y_axis(0, this->unit * 0.25, 0);
    Eigen::Vector3f z_axis(0, 0, this->unit * 0.25);

    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, x_axis), 1.0, 0.0, 0.0, prefix + "_x");
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, y_axis), 0.0, 1.0, 0.0, prefix + "_y");
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl_transform(pose, o), pcl_transform(pose, z_axis), 0.0, 0.0, 1.0, prefix + "_z");
}
void updatecloud()
{
    if(!cloudbusy)
	{
		loopbusy = true;
		cout<<"startloopmappoint"<<endl;
        PointCloud::Ptr tmp1(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId)
				{

					Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;

					//cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
					continue;
				}
			}
		}
        cout<<"finishloopmap"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );
        //viewer.showCloud( globalMap );
        loopbusy = false;
        //cloudbusy = true;
        loopcount++;

        //*globalMap = *tmp1;
	}
}








/*             ------------------------------------------    */



//     /** @brief 从追踪线程插入关键帧
//      * @param kf 关键帧
//      * @param color RGB图像
//      * @param depth 深度图像
//      * @param vpKFs 所有地图的所有的关键帧
//      */

//     void PointCloudMapping::insertKeyFrame(ORB_SLAM3::KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth,vector<ORB_SLAM3::KeyFrame*> vpKFs)
//     {
//         unique_lock<mutex> locker(mKeyFrameMtx);
//         cout << GREEN <<"receive a keyframe, id = "<<kf->mnId<<" 第"<<kf->mnId<<"个"<<endl;
//         mvKeyFrames.push(kf);
//         keyframes.push_back( kf );
//         currentvpKFs = vpKFs;
//         cv::Mat colorImg_ ,depth_;
//         mvColorImgs.push( color.clone() );
//         mvDepthImgs.push( depth.clone() );
//         // std::condition_variable mKeyFrameUpdatedCond;
//         // 这行代码是在使用条件变量（std::condition_variable）中的 notify_one() 方法。
//         // 这个方法的作用是通知一个等待在这个条件变量上的线程，告诉它们某些条件已经发生变化，可能是在多线程环境下用于线程间的同步。
//         // 当条件变量被通知时，等待在这个条件变量上的一个线程会被唤醒，继续执行。
//         mKeyFrameUpdatedCond.notify_one();
//     }

// /**
//  * @brief 更新当前点云
//  * @param curMap 当前地图
//  */
// void PointCloudMapping::updatecloud(ORB_SLAM3::Map &curMap)
// {
//     std::unique_lock<std::mutex> lck(updateMutex);

//     mabIsUpdating = true;
//     //  1.获取当前地图的所有关键帧
//     currentvpKFs = curMap.GetAllKeyFrames();
//     // loopbusy = true;
//     cout << "开始点云更新" << endl;
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMap(new pcl::PointCloud<pcl::PointXYZRGBA>);
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr curPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpGlobalMapFilter(new pcl::PointCloud<pcl::PointXYZRGBA>());
//     //  2.遍历每一个关键帧
//     /// 这段代码的目的似乎是将当前关键帧的点云数据转换到世界坐标系下，然后将其合并到全局地图中，并对全局地图进行体素滤波处理，以最终更新或维护一个更精确的全局地图。
//     for (int i = 0; i < currentvpKFs.size(); i++)
//     {
//         /// 2.1在闭环检测时如果地图有本质图优化则说明地图有较大改变,停止更新地图
//         if (!mabIsUpdating)
//         {
//             std::cout << "中断点云更新" <<std::endl;
//             return;
//         }
//         /// 2.2 如果这个关键帧没有被优化掉 且 这个关键帧生成了点云
//         // 在PointCloudMapping::generatePointCloud中关键帧生成点云
//         if (!currentvpKFs[i]->isBad() && currentvpKFs[i]->mptrPointCloud)
//         {
//             // 生成的点云是在相机坐标系下的,要转化成世界坐标系下
//             pcl::transformPointCloud(
//                 *(currentvpKFs[i]->mptrPointCloud), *(curPointCloud),
//                 Converter::toMatrix4d(currentvpKFs[i]->GetPoseInverse()));
//             *tmpGlobalMap += *curPointCloud;

//             voxel->setInputCloud(tmpGlobalMap);
//             voxel->filter(*tmpGlobalMapFilter);
//             // 将 tmpGlobalMap 指向的点云数据和 tmpGlobalMapFilter 指向的点云数据进行交换。
//             tmpGlobalMap->swap(*tmpGlobalMapFilter);
//         }
//     }
//     cout << "点云更新完成" << endl;
//     {
//         std::unique_lock<std::mutex> lck(mMutexGlobalMap);
//         globalMap = tmpGlobalMap;
//     }
//     mabIsUpdating = false;
// }

// void PointCloudMapping::generatePointCloud(ORB_SLAM3::KeyFrame *kf) //,Eigen::Isometry3d T
// {
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//     // point cloud is null ptr
//     for (int m = 0; m < kf->imDepth.rows; m += 4)
//     {
//         for (int n = 0; n < kf->imDepth.cols; n += 4)
//         {
//             float d = kf->imDepth.ptr<float>(m)[n];
//             if (d < 0.05 || d > 6)
//                 continue;
//             pcl::PointXYZRGBA p;
//             p.z = d;
//             p.x = (n - kf->cx) * p.z / kf->fx;
//             p.y = (m - kf->cy) * p.z / kf->fy;

//             p.b = kf->imLeftRgb.ptr<uchar>(m)[n * 3];
//             p.g = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 1];
//             p.r = kf->imLeftRgb.ptr<uchar>(m)[n * 3 + 2];

//             pPointCloud->points.push_back(p);
//         }
//     }
//     pPointCloud->height = 1;
//     pPointCloud->width = pPointCloud->points.size();
//     pPointCloud->is_dense = true;
//     kf->mptrPointCloud = pPointCloud;
// }

// /**
//  * @brief 显示线程
//  */
// void PointCloudMapping::viewer()
// {
//     pcl::visualization::CloudViewer viewer("viewer");
//     while (1)
//     {
//         {
//             unique_lock<mutex> lck_shutdown(shutDownMutex);
//             if (shutDownFlag)
//             {
//                 break;
//             }
//         }
//         if (bStop || mabIsUpdating)
//         {
//             continue;
//         }
//         int N;
//         std::list<KeyFrame *> lNewKeyFrames;
//         {
//             unique_lock<mutex> lck(keyframeMutex);
//             N = mlNewKeyFrames.size();
//             lNewKeyFrames = mlNewKeyFrames;
//             if(N == 0)
//                 continue;
//             else
//             {
//                 mlNewKeyFrames.clear();
//             }

//         }

//         double generatePointCloudTime = 0, transformPointCloudTime = 0;
//         for (auto pKF : lNewKeyFrames)
//         {
//             if (pKF->isBad())
//                 continue;
//             generatePointCloud(pKF);

//             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>);
//             pcl::transformPointCloud(*(pKF->mptrPointCloud), *(p), Converter::toMatrix4d(pKF->GetPoseInverse()));

//             {
//                 std::unique_lock<std::mutex> lck(mMutexGlobalMap);
//                 *globalMap += *p;
//             }

//         }
//         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

//         // 去除孤立点这个比较耗时，用处也不是很大，可以去掉
//         // statistical_filter->setInputCloud(globalMap);
//         // statistical_filter->filter(*tmp);

//         voxel->setInputCloud(globalMap);
//         voxel->filter(*globalMap);
//         viewer.showCloud(globalMap);  // 这个比较费时，建议不需要实时显示的可以屏蔽或改成几次显示一次
//     }
// }

/*                   -----------------                 */

}

