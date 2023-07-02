#ifndef _A_LOAM_LASER_ODOMETRY_H
#define _A_LOAM_LASER_ODOMETRY_H


#include "common.h"

#include <queue>
#include <mutex>
#include <string>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>

#include <pcl/kdtree/kdtree_flann.h>

#include "LidarFactor.hpp"


namespace ALOAM{

class LaserOdometry{

public:
    LaserOdometry()
        :q_last_curr(para_q),
        t_last_curr(para_t),
        cornerPointsSharp(nullptr),
        cornerPointsLessSharp(nullptr),
        surfPointsFlat(nullptr),
        surfPointsLessFlat(nullptr),
        corner_correspondence(0),
        plane_correspondence(0),
        kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
        kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
        laserCloudCornerLast(nullptr),
        laserCloudSurfLast(nullptr),
        laserCloudFullRes(nullptr),
        isFirstFrame(true),
        q_w_curr(1, 0, 0, 0),
        t_w_curr(0, 0, 0)
    {

    }


    /**
     * 插值补偿运动损失
    */
    void TransFormToStart(PointType const *const pi, PointType *const po){
        double s = 1.0;
        if(DISTORTION){
            s = (pi -> intensity - int(pi -> intensity)) / SCAN_PERIOD;
        }
        
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(pi -> x, pi -> y, pi -> z);
        Eigen::Vector3d un_point = q_point_last * point + t_point_last;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }

    /**
     * 将当前点云转换到下一帧的起始位置
    */
    void TransFormToEnd(PointType const *const pi, PointType *const po){
        PointType un_point_tmp;
        TransFormToStart(pi, &un_point_tmp);

        Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

        po -> x = point_end.x();
        po -> y = point_end.y();
        po -> z = point_end.z();

        po -> intensity = int(pi -> intensity);
    }

    /**
     * 将点云数据压入队列
    */
    void queueHandler(CloudTypePtr cloud_in, std::string name){
        mBuf.lock();
        
        if(name == cornerPointsSharpName) cornerSharpBuf.push(cloud_in);
        else if(name == cornerPointsLessSharpName) cornerLessSharpBuf.push(cloud_in);
        else if(name == surfPointsFlatName) surfFlatBuf.push(cloud_in);
        else if(name == surfPointsLessFlatName) surfLessFlatBuf.push(cloud_in);
        else if(name == laserCloudFullResName) fullPointsBuf.push(cloud_in);

        mBuf.unlock();
    }

    /**
     * 判断当前队列是否为空
    */
    bool isEmpty(){
        return cornerSharpBuf.empty() || cornerLessSharpBuf.empty() || 
                surfFlatBuf.empty() || surfLessFlatBuf.empty() || fullPointsBuf.empty();
    }

    /**
     * 初始化
    */
    bool init(){
        if(!isEmpty()){
            mBuf.lock();
        
            cornerPointsSharp = cornerSharpBuf.front();
            cornerSharpBuf.pop();

            cornerPointsLessSharp = cornerLessSharpBuf.front();
            cornerLessSharpBuf.pop();

            surfPointsFlat = surfFlatBuf.front();
            surfFlatBuf.pop();

            surfPointsLessFlat = surfLessFlatBuf.front();
            surfLessFlatBuf.pop();

            laserCloudFullRes = fullPointsBuf.front();
            fullPointsBuf.pop();

            mBuf.unlock();
            return true;
        }
        return false;     
    }

    /**
     * 里程计
    */
    void odometry(){
        // 初始化
        if(!init()){
            return;
        }

        if(isFirstFrame){
            isFirstFrame = false;
        }else{
            int cornerPointsSharpNum = cornerPointsSharp -> points.size();
            int surfPointsFlatNum = surfPointsFlat -> points.size();

            // 两次优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                corner_correspondence = 0;
                plane_correspondence = 0;

                ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
                ceres::Manifold *q_manifold = 
                    new ceres::EigenQuaternionManifold();
                ceres::Problem::Options problem_options;
                
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_manifold);
                problem.AddParameterBlock(para_t, 3);

                PointType pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // corner
                for(int i = 0; i < cornerPointsSharpNum; ++i){
                    TransFormToStart(&(cornerPointsSharp -> points[i]), &pointSel);
                    kdtreeCornerLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                    
                    // 最近点索引
                    int closestPointInd = -1, minPointInd2 = -1;
                    if(pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(laserCloudCornerLast -> points[closestPointInd].intensity);

                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                        // 增大方向寻找
                        for(int j = closestPointInd + 1; j < int(laserCloudCornerLast -> size()); ++j){
                            // 是同一条激光线
                            if(int(laserCloudCornerLast -> points[j].intensity) <= closestPointScanID)
                                continue;
                            // 不是附近的激光线
                            if(int(laserCloudCornerLast -> points[j].intensity) > closestPointScanID + NEARBY_SCAN)
                                break;

                            // 计算欧式距离
                            double pointSqDis = (laserCloudCornerLast -> points[j].x - pointSel.x) *
                                                (laserCloudCornerLast -> points[j].x - pointSel.x) +
                                                (laserCloudCornerLast -> points[j].y - pointSel.y) *
                                                (laserCloudCornerLast -> points[j].y - pointSel.y) +
                                                (laserCloudCornerLast -> points[j].z - pointSel.z) *
                                                (laserCloudCornerLast -> points[j].z - pointSel.z);
                            if(pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                        // 减小方向寻找
                        for(int j = closestPointInd - 1; j >= 0; --j){
                            // 是同一条激光线
                            if(int(laserCloudCornerLast -> points[j].intensity) >= closestPointScanID)
                                continue;
                            // 不是附近的激光线
                            if(int(laserCloudCornerLast -> points[j].intensity) < closestPointScanID - NEARBY_SCAN)
                                break;

                            // 计算欧式距离
                            double pointSqDis = (laserCloudCornerLast -> points[j].x - pointSel.x) *
                                                (laserCloudCornerLast -> points[j].x - pointSel.x) +
                                                (laserCloudCornerLast -> points[j].y - pointSel.y) *
                                                (laserCloudCornerLast -> points[j].y - pointSel.y) +
                                                (laserCloudCornerLast -> points[j].z - pointSel.z) *
                                                (laserCloudCornerLast -> points[j].z - pointSel.z);
                            if(pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        if(minPointInd2 >= 0){
                            Eigen::Vector3d curr_point(cornerPointsSharp -> points[i].x,
                                                       cornerPointsSharp -> points[i].y,
                                                       cornerPointsSharp -> points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast -> points[closestPointInd].x,
                                                         laserCloudCornerLast -> points[closestPointInd].y,
                                                         laserCloudCornerLast -> points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast -> points[minPointInd2].x,
                                                         laserCloudCornerLast -> points[minPointInd2].y,
                                                         laserCloudCornerLast -> points[minPointInd2].z);

                            double s = 1.0;
                            if(DISTORTION){
                                s = (cornerPointsSharp -> points[i].intensity - int(cornerPointsSharp -> points[i].intensity)) / SCAN_PERIOD;
                            }
                            ceres::CostFunction *cost_function = 
                                    LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            ++corner_correspondence;
                        }
                    }



                }

                // plane
                for(int i = 0; i < surfPointsFlatNum; ++i){
                    TransFormToStart(&(surfPointsFlat -> points[i]), &pointSel);
                    kdtreeSurfLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if(pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){
                        closestPointInd = pointSearchInd[0];

                        int closestPointScanID = int(laserCloudSurfLast -> points[closestPointInd].intensity);
                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                        // 增大方向
                        for(int j = closestPointInd + 1; j < int(laserCloudSurfLast -> size()); ++j){
                            if(int(laserCloudSurfLast -> points[j].intensity) > closestPointScanID + NEARBY_SCAN)
                                break;
                            
                            double pointSqDis = (laserCloudSurfLast -> points[j].x - pointSel.x) *
                                                (laserCloudSurfLast -> points[j].x - pointSel.x) +
                                                (laserCloudSurfLast -> points[j].y - pointSel.y) *
                                                (laserCloudSurfLast -> points[j].y - pointSel.y) +
                                                (laserCloudSurfLast -> points[j].z - pointSel.z) *
                                                (laserCloudSurfLast -> points[j].z - pointSel.z);
                            
                            if(int(laserCloudSurfLast -> points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if(int(laserCloudSurfLast -> points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        // 减小方向
                        for(int j = closestPointInd - 1; j >= 0; --j){
                            if(int(laserCloudSurfLast -> points[j].intensity) > closestPointScanID - NEARBY_SCAN)
                                break;
                            
                            double pointSqDis = (laserCloudSurfLast -> points[j].x - pointSel.x) *
                                                (laserCloudSurfLast -> points[j].x - pointSel.x) +
                                                (laserCloudSurfLast -> points[j].y - pointSel.y) *
                                                (laserCloudSurfLast -> points[j].y - pointSel.y) +
                                                (laserCloudSurfLast -> points[j].z - pointSel.z) *
                                                (laserCloudSurfLast -> points[j].z - pointSel.z);
                            
                            if(int(laserCloudSurfLast -> points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if(int(laserCloudSurfLast -> points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if(minPointInd2 >= 0 && minPointInd3 >= 0){
                            Eigen::Vector3d curr_point(surfPointsFlat -> points[i].x,
                                                       surfPointsFlat -> points[i].y,
                                                       surfPointsFlat -> points[i].z);
                            Eigen::Vector3d last_point_a(surfPointsFlat -> points[closestPointInd].x,
                                                         surfPointsFlat -> points[closestPointInd].y,
                                                         surfPointsFlat -> points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(surfPointsFlat -> points[minPointInd2].x,
                                                         surfPointsFlat -> points[minPointInd2].y,
                                                         surfPointsFlat -> points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(surfPointsFlat -> points[minPointInd3].x,
                                                         surfPointsFlat -> points[minPointInd3].y,
                                                         surfPointsFlat -> points[minPointInd3].z);
                            
                            double s = 1.0;
                            if(DISTORTION){
                                s = (surfPointsFlat -> points[i].intensity - int(surfPointsFlat -> points[i].intensity)) / SCAN_PERIOD;
                            }
                            ceres::CostFunction* cost_function = 
                                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            ++plane_correspondence;
                        }

                    }
                }
                

                // 求解
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }

            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
        }

        // // 变换矩阵
        // trans_curr = Eigen::Translation3d(t_w_curr) * q_w_curr.toRotationMatrix();

        // 将边缘特征和平面特征转换到帧末
        if(0){
            int cornerPointsLessSharpNum = cornerPointsLessSharp -> size();
            for(int i = 0; i < cornerPointsLessSharpNum; ++i){
                TransFormToEnd(&cornerPointsLessSharp -> points[i], &cornerPointsLessSharp -> points[i]);
            }

            int surfPointsLessFlatNum = surfPointsLessFlat -> points.size();
            for(int i = 0; i < surfPointsLessFlatNum; ++i){
                TransFormToEnd(&surfPointsLessFlat -> points[i], &surfPointsLessFlat -> points[i]);
            }

            int laserCloudFullResNum = laserCloudFullRes -> points.size();
            for(int i = 0; i < laserCloudFullResNum; ++i){
                TransFormToEnd(&laserCloudFullRes -> points[i], &laserCloudFullRes -> points[i]);
            }
        }

        // 更新前一帧点云
        CloudTypePtr laserCloudTmp = cornerPointsLessSharp;

        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTmp;

        laserCloudTmp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTmp;

        kdtreeCornerLast -> setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast -> setInputCloud(laserCloudSurfLast);
    }

    /**
     * 获取四元数
    */
    Eigen::Quaterniond getCurrQuateriniond() const {
        return q_w_curr;
    } 

    /**
     * 获取平移矩阵
    */
   Eigen::Vector3d getCurrVector() const {
        return t_w_curr;
   }


private:

    Eigen::Map<Eigen::Quaterniond> q_last_curr;
    Eigen::Map<Eigen::Vector3d> t_last_curr;

    // 队列，存储未配准的点云
    std::queue<CloudTypePtr> cornerSharpBuf;
    std::queue<CloudTypePtr> cornerLessSharpBuf;
    std::queue<CloudTypePtr> surfFlatBuf;
    std::queue<CloudTypePtr> surfLessFlatBuf;
    std::queue<CloudTypePtr> fullPointsBuf;
    std::mutex mBuf;

    // scan registration传递的五种点云
    CloudTypePtr cornerPointsSharp;
    CloudTypePtr cornerPointsLessSharp;
    CloudTypePtr surfPointsFlat;
    CloudTypePtr surfPointsLessFlat;

    int corner_correspondence;
    int plane_correspondence;

    // 上一帧点云信息与kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;
    CloudTypePtr laserCloudCornerLast;
    CloudTypePtr laserCloudSurfLast;
    CloudTypePtr laserCloudFullRes;


    bool isFirstFrame;

    // 变换结果
    Eigen::Quaterniond q_w_curr;
    Eigen::Vector3d t_w_curr;

};

}

#endif