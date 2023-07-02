#ifndef _A_LOAM_LASER_MAPPING_HPP
#define _A_LOAM_LASER_MAPPING_HPP

#include <queue>
#include <mutex>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include "common.h"
#include "LidarFactor.hpp"


namespace ALOAM{

class LaserMapping{
public:
    LaserMapping(int lineRes, int planeRes)
        : laserCloudNum(laserCloudWidth * laserCloudHeight * laserCloudDepth),
          frameCount(0),
          q_wodom_curr(1, 0, 0, 0),
          t_wodom_curr(0, 0, 0),
          q_wmap_wodom(1, 0, 0, 0),
          t_wmap_wodom(0, 0, 0),
          q_w_curr(parameters),
          t_w_curr(parameters + 4),
          laserCloudCornerLast(new CloudType()),
          laserCloudSurfLast(new CloudType()),
          laserCloudFullRes(new CloudType()),
          laserCloudCornerArray(laserCloudNum),
          laserCloudValidInd(125),
          laserCloudSurroundInd(125),
          kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()),
          kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>()),
          laserCloudCornerFromMap(new CloudType()),
          laserCloudSurfFromMap(new CloudType())
    {
        downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
        downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);
    }

    // 初始化世界坐标系下位姿
    void transformAssociateToMap(){
        q_w_curr = q_wmap_wodom * q_wodom_curr;
        t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }
    // 更新里程计坐标系到世界坐标系的变换矩阵
    void transformUpdate(){
        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
    }
    // 将点转换到世界坐标系下
    void pointAssociateToMap(PointType const *const pi, PointType *const po){
        Eigen::Vector3d point_curr(pi -> x, pi -> y, pi -> z);
        Eigen::Vector3d point_w = point_curr * q_w_curr + t_w_curr;
        po -> x = point_w.x();
        po -> y = point_w.y();
        po -> z = point_w.z();
        po -> intensity = pi -> intensity;
    }

    void pointAssociateTobeMapped(PointType const *const pi, PointType *const po){
        Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        po->intensity = pi->intensity;
    }

    void laserCloudHandler(CloudTypePtr cloud_in, string name){
        mBuf.lock();
        if(name == cornerLast) cornerLastBuf.push(cloud_in);
        else if(name == surfLast) surfLastBuf.push(cloud_in);
        else if(name == fullRes) fullResBuf.push(cloud_in);
        mBuf.unlock();
    }

    bool isEmpty(){
        return cornerLastBuf.empty() || surfLastBuf.empty() || fullResBuf.empty();
    }

    bool init(){
        if(isEmpty()){
            return false;
        }
        mBuf.lock();

        laserCloudCornerLast = cornerLastBuf.front();
        cornerLastBuf.pop();

        laserCloudSurfLast = surfLastBuf.front();
        surfLastBuf.pop();

        laserCloudFullRes = fullResBuf.front();
        fullResBuf.pop();

        mBuf.unlock();
        return true;
    }

    void process(){
        if(!init()){
            return;
        }

        int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
        int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

        if(t_w_curr.x() + 25.0 < 0)
            centerCubeI--;
        if(t_w_curr.y() + 25.0 < 0)
            centerCubeJ--;
        if(t_w_curr.z() + 25.0 < 0)
            centerCubeK--;

        // 将当前时刻的cube移动到地图相对中心位置
        while(centerCubeI < 3){
            for(int j = 0; j < laserCloudHeight; ++j){
                for(int k = 0; k < laserCloudDepth; ++k){
                    int i = laserCloudWidth - 1;
                    CloudTypePtr laserCloudCubeCornerPointer = 
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    CloudTypePtr laserCloudCubeSurfPointer = 
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for(; i >= 1; --i){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer -> clear();
                    laserCloudCubeSurfPointer -> clear();
                }
            }
            ++centerCubeI;
            ++laserCloudCenWidth;   
        }
        while (centerCubeI >= laserCloudWidth - 3){ 
            for (int j = 0; j < laserCloudHeight; ++j){
                for (int k = 0; k < laserCloudDepth; ++k){
                    int i = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; i < laserCloudWidth - 1; ++i){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --centerCubeI;
            --laserCloudCenWidth;
        }
        while (centerCubeJ < 3){
            for (int i = 0; i < laserCloudWidth; ++i){
                for (int k = 0; k < laserCloudDepth; ++k){
                    int j = laserCloudHeight - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j >= 1; --j){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            ++centerCubeJ;
            ++laserCloudCenHeight;
        }
        while (centerCubeJ >= laserCloudHeight - 3){
            for (int i = 0; i < laserCloudWidth; ++i){
                for (int k = 0; k < laserCloudDepth; ++k){
                    int j = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j < laserCloudHeight - 1; ++j){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --centerCubeJ;
            --laserCloudCenHeight;
        }
        while (centerCubeK < 3){
            for (int i = 0; i < laserCloudWidth; ++i){
                for (int j = 0; j < laserCloudHeight; ++j){
                    int k = laserCloudDepth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k >= 1; --k){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            ++centerCubeK;
            ++laserCloudCenDepth;
        }
        while (centerCubeK >= laserCloudDepth - 3){
            for (int i = 0; i < laserCloudWidth; ++i){
                for (int j = 0; j < laserCloudHeight; ++j)
                {
                    int k = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k < laserCloudDepth - 1; ++k){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            --centerCubeK;
            --laserCloudCenDepth;
        }

        // 从地图中分割出5 * 5 * 3 的cube
        int laserCloudValidNum = 0;
        int laserCloudSurroundNum = 0;

        for(int i = centerCubeI - 2; i <= centerCubeI + 2; ++i){
            for(int j = centerCubeJ - 2; j <= centerCubeJ + 2; ++j){
                for(int k = centerCubeK - 1; k <= centerCubeK + 1; ++k){
                    if(i >= 0 && i < laserCloudWidth && 
                       j >= 0 && j < laserCloudHeight &&
                       k >= 0 && k < laserCloudDepth){
                        laserCloudValidInd[laserCloudValidNum++] = 
                            i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudSurroundInd[laserCloudSurroundNum++] = 
                            i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    }
                }
            }
        }

        laserCloudCornerFromMap -> clear();
        laserCloudSurfFromMap -> clear();
        for(int i = 0; i < laserCloudValidNum; ++i){
            *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
            *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }

        int laserCloudCornerFromMapNum = laserCloudCornerFromMap -> size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap -> size();

        CloudTypePtr laserCloudCornerStack(new CloudType());
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack -> size();

        CloudTypePtr laserCloudSurfStack(new CloudType());
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack -> size();

        if(laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50){
            kdtreeCornerFromMap -> setInputCloud(laserCloudCornerFromMap);
            kdtreeSurfFromMap -> setInputCloud(laserCloudSurfFromMap);

            for(int iterCount = 0; iterCount < 2; ++iterCount){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Manifold *q_manifold = new ceres::EigenQuaternionManifold();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameters, 4, q_manifold);
                problem.AddParameterBlock(parameters + 4, 3);

                int corner_num = 0;
                for(int i = 0; i < laserCloudCornerStackNum; ++i){
                    pointOri = laserCloudCornerStack -> points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    kdtreeCornerFromMap -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
                    
                    // 求最近邻的协方差
                    if(pointSearchSqDis[4] < 1.0){
                        std::vector<Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for(int j = 0; j < 5; ++j){
                            Eigen::Vector3d tmp(laserCloudCornerFromMap -> points[pointSearchInd[j]].x,
                                                laserCloudCornerFromMap -> points[pointSearchInd[j]].y,
                                                laserCloudCornerFromMap -> points[pointSearchInd[j]].z);
                            center = center + tmp;
                            nearCorners.push_back(tmp);
                        }
                        center = center / 5.0;

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

                        for(int j = 0; j < 5; ++j){
                            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]){
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d point_a, point_b;
                            point_a = 0.1 * unit_direction + point_on_line;
                            point_b = -0.1 * unit_direction + point_on_line;

                            ceres::CostFunction * cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            ++corner_num;
                        }
                    }
                } 

                int surf_num = 0;
                for(int i = 0; i < laserCloudSurfStackNum; ++i){
                    pointOri = laserCloudSurfStack -> points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    kdtreeSurfFromMap -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    Eigen::Matrix<double, 5, 3> matA0;
                    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    if(pointSearchSqDis[4] < 1.0){
                        for(int j = 0; j < 5; ++j){
                            matA0[j, 0] = laserCloudSurfFromMap -> points[pointSearchInd[j]].x;
                            matA0[j, 1] = laserCloudSurfFromMap -> points[pointSearchInd[j]].y;
                            matA0[j, 2] = laserCloudSurfFromMap -> points[pointSearchInd[j]].z;
                        }

                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        bool planeValid = true;
                        for(int j = 0; j < 5; ++j){
                            if(fabs(norm(0) * laserCloudSurfFromMap -> points[pointSearchInd[j]].x + 
                                    norm(1) * laserCloudSurfFromMap -> points[pointSearchInd[j]].y +
                                    norm(2) * laserCloudSurfFromMap -> points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2){
                                planeValid = false;
                                break;
                            }
                        }
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if(planeValid){
                            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            ++surf_num;
                        }
                    }
                }

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }

        }else{
            std::cout << "corner and surf num are not enough" << std::endl;
        }

        transformUpdate();

        // 更新地图
        for(int i = 0; i < laserCloudCornerStackNum; ++i){
            pointAssociateToMap(&laserCloudCornerStack -> points[i], &pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if(pointSel.x + 25.0 < 0) --cubeI;
            if(pointSel.y + 25.0 < 0) --cubeJ;
            if(pointSel.z + 25.0 < 0) --cubeK;

            if(cubeI >= 0 && cubeI < laserCloudWidth &&
               cubeJ >= 0 && cubeJ < laserCloudHeight &&
               cubeK >= 0 && cubeK < laserCloudDepth){
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudCornerArray[cubeInd] -> push_back(pointSel);
            }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++){
            pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if (pointSel.x + 25.0 < 0) --cubeI;
            if (pointSel.y + 25.0 < 0) --cubeJ;
            if (pointSel.z + 25.0 < 0) --cubeK;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudSurfArray[cubeInd] -> push_back(pointSel);
            }
        }

        // 滤波
        for(int i = 0; i < laserCloudValidNum; ++i){
            int ind = laserCloudValidInd[i];

            CloudTypePtr tmpCorner(new CloudType());
            downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
            downSizeFilterCorner.filter(*tmpCorner);
            laserCloudCornerArray[ind] = tmpCorner;

            CloudTypePtr tmpSurf(new CloudType());
            downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
            downSizeFilterSurf.filter(*tmpCorner);
            laserCloudSurfArray[ind] = tmpSurf;
        }

    }



private:
    int laserCloudNum;
    int frameCount;
    // 激光里程计坐标系下位姿
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    
    // 里程计坐标到世界坐标的转换
    Eigen::Quaterniond q_wmap_wodom;
    Eigen::Vector3d t_wmap_wodom;

    // 世界坐标系下位姿
    Eigen::Map<Eigen::Quaterniond> q_w_curr;
    Eigen::Map<Eigen::Vector3d> t_w_curr;

    // 点云队列
    std::queue<CloudTypePtr> cornerLastBuf;
    std::queue<CloudTypePtr> surfLastBuf;
    std::queue<CloudTypePtr> fullResBuf;
    std::mutex mBuf;

    CloudTypePtr laserCloudCornerLast;
    CloudTypePtr laserCloudSurfLast;
    CloudTypePtr laserCloudFullRes;

    // cube数组
    std::vector<CloudTypePtr> laserCloudCornerArray;
    std::vector<CloudTypePtr> laserCloudSurfArray;

    // 
    std::vector<int> laserCloudValidInd;
    std::vector<int> laserCloudSurroundInd;

    // kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    // 地图中分割出的点云
    CloudTypePtr laserCloudCornerFromMap;
    CloudTypePtr laserCloudSurfFromMap;

    // 滤波器
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel;
};


}


#endif