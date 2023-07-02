#ifndef _A_LOAM_SCAN_REGISTRATION_H
#define _A_LOAM_SCAN_REGISTRATION_H

#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "common.h"



using std::atan2;
using std::cos;
using std::sin;
using std::cout;
using std::endl;

namespace ALOAM{

template <typename PointT>
class ScanRegistration{

using CLOUD = pcl::PointCloud<PointT>;
using CLOUDPTR = std::shared_ptr<pcl::PointCloud<PointT>>;




public:
    /**
     * 初始化参数
    */
    ScanRegistration()   
    {
        laserCloud.reset(new CloudType());
        cornerPointsSharp.reset(new CloudType());
        cornerPointsLessSharp.reset(new CloudType());
        surfPointsFlat.reset(new CloudType());
        surfPointsLessFlat.reset(new CloudType());
    }
    /**
     * 去除距离车近的点
    */
    void removeClosedPointCloud(CLOUDPTR cloud_in, CLOUDPTR cloud_out, float thres){
        if(cloud_in != cloud_out){
            cloud_out -> header = cloud_in -> header;
            cloud_out -> points.resize(cloud_in -> points.size());
        }

        size_t j = 0;
        float thres_2 = thres * thres;

        for(size_t i = 0; i < cloud_in -> size(); ++i){
            PointT point = cloud_in -> points[i];
            if(point.x * point.x + point.y * point.y + point.z * point.z > thres_2){
                cloud_out -> points[j] = point;
                ++j;
            }
        }

        cloud_out -> points.resize(j);
        cloud_out -> height = 1;
        cloud_out -> width = static_cast<uint32_t>(j);
        cloud_out -> is_dense = true;
    }
    
    /**
     * 计算曲率
     * 提取平面点与边缘点
    */
    void extractFeatureFromPointCloud(CLOUDPTR cloud_in){
        std::vector<int> scanStrInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);

        int cloudSize = cloud_in -> size();
        float strOri = -atan2(cloud_in -> points[0].y, cloud_in -> points[0].x);
        float endOri = -atan2(cloud_in -> points[cloudSize - 1].y, 
                              cloud_in -> points[cloudSize - 1].x) + 2 * M_PI;

        /**
         * 保证endOri - strOri 属于 [pi, 3pi] why?
        */
        if(endOri - strOri > 3 * M_PI){
            endOri -= 2 * M_PI;
        }else if(endOri - strOri < M_PI){
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        int count = cloudSize;
        PointType point;
        std::vector<CloudType> laserCloudScans(N_SCANS);
        for(int i = 0; i < cloudSize; ++i){
            point.x = cloud_in -> points[i].x;
            point.y = cloud_in -> points[i].y;
            point.z = cloud_in -> points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;
            switch(N_SCANS){
                case 16:
                    scanID = int((angle + 15) / 2 + 0.5);
                    break;
                case 32:
                    scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                    break;                  
                default:
                    cout << "wrong scan number" << endl;
            }
            if(scanID > (N_SCANS - 1) || scanID < 0){
                --count;
                continue;
            }

            float curOri = -atan2(point.y, point.x);
            if(!halfPassed){
                if(curOri < strOri - M_PI / 2) 
                    curOri += 2 * M_PI;
                else if(curOri > strOri + M_PI * 3 / 2) 
                    curOri -= 2 * M_PI;
                if(curOri - strOri > M_PI) 
                    halfPassed = true;
            }else{
                curOri += 2 * M_PI;
                if(curOri < endOri - M_PI * 3 / 2)
                    curOri += 2 * M_PI;
                else if(curOri > endOri + M_PI / 2)
                    curOri -= 2 * M_PI;
            }

            float relTime = (curOri - strOri) / (endOri - strOri);
            point.intensity = scanID + SCAN_PERIOD * relTime;
            laserCloudScans[scanID].push_back(point);
        }

        cloudSize = count;

        /**
         * 计算曲率
        */
        for(int i = 0; i < N_SCANS; ++i){
            scanStrInd[i] = laserCloud -> size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud -> size() - 6;
        }

        std::vector<float> cloudCurvature(40000);
        std::vector<int> cloudSortInd(40000);
        std::vector<int> cloudNeighborPicked(40000);
        std::vector<int> cloudLabel(40000);
        auto comp = [&](const int i, const int j){
            return cloudCurvature[i] < cloudCurvature[j];
        };

        for(int i = 5; i < cloudSize - 5; ++i){
            float diffX = laserCloud -> points[i - 5].x +
                          laserCloud -> points[i - 4].x +
                          laserCloud -> points[i - 3].x +
                          laserCloud -> points[i - 2].x +
                          laserCloud -> points[i - 1].x +
                          laserCloud -> points[i].x     +
                          laserCloud -> points[i + 1].x +
                          laserCloud -> points[i + 2].x +
                          laserCloud -> points[i + 3].x +
                          laserCloud -> points[i + 4].x +
                          laserCloud -> points[i + 5].x ;

            float diffY = laserCloud -> points[i - 5].y +
                          laserCloud -> points[i - 4].y +
                          laserCloud -> points[i - 3].y +
                          laserCloud -> points[i - 2].y +
                          laserCloud -> points[i - 1].y +
                          laserCloud -> points[i].y     +
                          laserCloud -> points[i + 1].y +
                          laserCloud -> points[i + 2].y +
                          laserCloud -> points[i + 3].y +
                          laserCloud -> points[i + 4].y +
                          laserCloud -> points[i + 5].y ;

            float diffZ = laserCloud -> points[i - 5].z +
                          laserCloud -> points[i - 4].z +
                          laserCloud -> points[i - 3].z +
                          laserCloud -> points[i - 2].z +
                          laserCloud -> points[i - 1].z +
                          laserCloud -> points[i].z     +
                          laserCloud -> points[i + 1].z +
                          laserCloud -> points[i + 2].z +
                          laserCloud -> points[i + 3].z +
                          laserCloud -> points[i + 4].z +
                          laserCloud -> points[i + 5].z ;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }


        for(int i = 0; i < N_SCANS; ++i){
            if(scanEndInd[i] - scanStrInd[i] < 6)
                continue;   
            CloudType::Ptr surfPointsLessFlatScan(new CloudType());
            for(int j = 0; j < 6; ++j){
                int sp = scanStrInd[i] + (scanEndInd[i] - scanStrInd[i]) * j / 6;
                int ep = scanStrInd[i] + (scanEndInd[i] - scanStrInd[i]) * (j + 1) / 6 - 1;

                std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1, comp);

                /**
                 * 边缘点提取
                */
                int largestPickedNum = 0;
                for(int k = ep; k >= sp; --k){
                    int curInd = cloudSortInd[k];

                    if(cloudNeighborPicked[curInd] == 0 && cloudCurvature[curInd] > 0.1){
                        ++largestPickedNum;
                        if(largestPickedNum <= 2){
                            cloudLabel[curInd] = 2;
                            cornerPointsSharp -> push_back(laserCloud -> points[curInd]);
                            cornerPointsLessSharp -> push_back(laserCloud -> points[curInd]);
                        }else if(largestPickedNum <= 20){
                            cloudLabel[curInd] = 1;
                            cornerPointsLessSharp -> push_back(laserCloud -> points[curInd]);
                        }else{
                            break;
                        }

                        cloudNeighborPicked[curInd] = 1;
                        for(int l = 1; l <= 5; ++l){
                            float diffX = laserCloud -> points[curInd + l].x -
                                          laserCloud -> points[curInd + l - 1].x;
                            float diffY = laserCloud -> points[curInd + l].y -
                                          laserCloud -> points[curInd + l - 1].y;
                            float diffZ = laserCloud -> points[curInd + l].z -
                                          laserCloud -> points[curInd + l - 1].z;
                            if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;
                            cloudNeighborPicked[curInd + l] = 1;
                        }
                        for(int l = -1; l >= -5; --l){
                            float diffX = laserCloud -> points[curInd + l].x -
                                          laserCloud -> points[curInd + l + 1].x;
                            float diffY = laserCloud -> points[curInd + l].y -
                                          laserCloud -> points[curInd + l + 1].y;
                            float diffZ = laserCloud -> points[curInd + l].z -
                                          laserCloud -> points[curInd + l + 1].z;
                            if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;
                            cloudNeighborPicked[curInd + l] = 1;
                        }
                    }
                }

                /**
                 * 平面点提取
                */
                int smallestPickedNum = 0;
                for(int k = sp; k <= ep; ++k){
                    int curInd = cloudSortInd[k];
                    if(cloudNeighborPicked[curInd] == 0 && cloudCurvature[curInd] < 0.1){
                        cloudLabel[curInd] = -1;
                        surfPointsFlat -> push_back(laserCloud -> points[curInd]);

                        ++smallestPickedNum;
                        if(smallestPickedNum >= 4){
                            break;
                        }

                        cloudNeighborPicked[curInd] = 1;
                        for(int l = 1; l <= 5; ++l){
                            float diffX = laserCloud -> points[curInd + l].x -
                                          laserCloud -> points[curInd + l - 1].x;
                            float diffY = laserCloud -> points[curInd + l].y -
                                          laserCloud -> points[curInd + l - 1].y;
                            float diffZ = laserCloud -> points[curInd + l].z -
                                          laserCloud -> points[curInd + l - 1].z;
                            if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;
                            cloudNeighborPicked[curInd + l] = 1;
                        }
                        for(int l = -1; l >= -5; --l){
                            float diffX = laserCloud -> points[curInd + l].x -
                                          laserCloud -> points[curInd + l + 1].x;
                            float diffY = laserCloud -> points[curInd + l].y -
                                          laserCloud -> points[curInd + l + 1].y;
                            float diffZ = laserCloud -> points[curInd + l].z -
                                          laserCloud -> points[curInd + l + 1].z;
                            if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                                break;
                            cloudNeighborPicked[curInd + l] = 1;
                        }
                    }
                }
                
                for(int k = sp; k <= ep; ++k){
                    if(cloudLabel[k] <= 0)
                        surfPointsLessFlatScan -> push_back(laserCloud -> points[k]);
                }
            }

            CloudType surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            *surfPointsLessFlat += surfPointsLessFlatScanDS;
        }
        

    }

    CloudType getLaserCloud() const{
        return *laserCloud;
    }
    CloudType getCornerPointsSharp() const{
        return *cornerPointsSharp;
    }
    CloudType getCornerPointsLessSharp() const{
        return *cornerPointsLessSharp;
    }
    CloudType getSurfPointsFlat() const{
        return *surfPointsFlat;
    }
    CloudType getSurfPointsLessFlat() const{
        return *surfPointsLessFlat;
    }

private:

    CloudTypePtr cornerPointsSharp;
    CloudTypePtr cornerPointsLessSharp;
    CloudTypePtr surfPointsFlat;
    CloudTypePtr surfPointsLessFlat;

    CloudTypePtr laserCloud;

};

}

#endif