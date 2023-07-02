#ifndef _RADAR_FEATURE_H
#define _RADAR_FEATURE_H

#include "RadarConfig.h"

namespace radar{

class RadarFeatures{
public:
    void Cen2018Features(cv::Mat fft_data){
    
        std::vector<float> sigma_q(fft_data.rows, 0);
        // Estimate the bias and subtract it
        Mat q = fft_data.clone();
        for(int i = 0; i < fft_data.rows; ++i){
            float mean = 0;
            for(int j = 0; j < fft_data.cols; ++j){
                mean += fft_data.at<float>(i, j);
            }

            mean /= fft_data.cols;
            for(int j = 0; j < fft_data.cols; ++j){
                q.at<float>(i, j) = fft_data.at<float>(i, j) - mean;
            }
        }

        // create 1D gaussian filter
        int fsize = sigma_gauss * 3;
        int mu = fsize / 2;
        float sig_sqr = sigma_gauss * sigma_gauss;
        Mat filter = Mat::zeros(1, fsize, CV_32F);
        float s = 0;
        for(int i = 0; i < fsize; ++i){
            filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
            s += filter.at<float>(0, i);
        }
        filter /= s;
        Mat p;
        cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

        // estimate variance of noise at each azimuth
        for(int i = 0; i < fft_data.rows; ++i){
            int nonzero = 0;
            for(int j = 0; i < fft_data.cols; ++j){
                float n = q.at<float>(i, j);
                if(n < 0){
                    sigma_q[i] += 2 * (n * n);
                    nonzero++;
                }
            }
            if(nonzero)
                sigma_q[i] = sqrt(sigma_q[i] / nonzero);
            else
                sigma_q[i] = 0.034;
        }

        // estract peak centers from each azimuth
        std::vector<std::vector<cv::Point2f>> t(fft_data.rows);
        for (int i = 0; i < fft_data.rows; ++i) {
            std::vector<int> peak_points;
            float thres = zq * sigma_q[i];
            for (int j = min_range; j < fft_data.cols; ++j) {
                float nqp = exp(-0.5 * pow((q.at<float>(i, j) - p.at<float>(i, j)) / sigma_q[i], 2));
                float npp = exp(-0.5 * pow(p.at<float>(i, j) / sigma_q[i], 2));
                float b = nqp - npp;
                float y = q.at<float>(i, j) * (1 - nqp) + p.at<float>(i, j) * b;
                if (y > thres) {
                    peak_points.push_back(j);
                } else if (peak_points.size() > 0) {
                    t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
                    peak_points.clear();
                }
            }
            if (peak_points.size() > 0)
                t[i].push_back(cv::Point(i, peak_points[peak_points.size() / 2]));
        }

        int size = 0;
        for (uint i = 0; i < t.size(); ++i) {
            size += t[i].size();
        }
        targets = Eigen::MatrixXd::Ones(3, size);
        int k = 0;
        for (uint i = 0; i < t.size(); ++i) {
            for (uint j = 0; j < t[i].size(); ++j) {
                targets(0, k) = t[i][j].x;
                targets(1, k) = t[i][j].y;
                k++;
            }
        }

    }

    void Cen2019Features(cv::Mat fft_data){
        const int min_range = 58;
        const int max_points = 10000;

        // 求梯度
        cv::Mat prewitt = cv::Mat::zeros(1, 3, CV_32F);
        prewitt.at<float>(0, 0) = -1;
        prewitt.at<float>(0, 2) = 1;
        cv::Mat g;
        cv::filter2D(fft_data, g, -1, prewitt, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);
        g = cv::abs(g);
        // 梯度归一化
        double maxg = 1, ming = 1;
        cv::minMaxIdx(g, &ming, &maxg);
        g /= maxg;

        // fft数据去偏置，并通过梯度大小放缩
        float mean = cv::mean(fft_data)[0];
        cv::Mat s = fft_data - mean;
        cv::Mat h = s.mul(1 - g);
        float mean_h = cv::mean(h)[0];

        // 获得放缩后大小幅值大于均值的点坐标
        auto cmp = [](const Point& p1, const Point& p2){
            return p1.i > p2.i;
        };
        std::vector<Point> vec;
        for(int i = 0; i < fft_data.rows; ++i)
            for(int j = 0; j < fft_data.cols; ++j)
                if(h.at<float>(i, j) > mean_h)
                    vec.push_back(Point(h.at<float>(i, j), i, j));
        std::sort(vec.begin(), vec.end(), cmp);

        // 
        int falseCount = fft_data.rows * fft_data.cols;
        uint j = 0;
        int l = 0;
        cv::Mat R = cv::Mat::zeros(fft_data.rows, fft_data.cols, CV_32F);
        while(l < max_points && j < vec.size() && falseCount > 0){
            if(!R.at<float>(vec[j].r, vec[j].c)){
                int clow = vec[j].c;
                int chigh = vec[j].c;
                // 寻找边界
                if(clow > 0){
                    for(int i = clow - 1; i >= 0; --i){
                        if(s.at<float>(vec[j].r, i) >= 0)
                            break;
                        clow = i;
                    }
                }
                if(chigh < s.cols - 1){
                    for(int i = chigh + 1; i < s.cols; ++i){
                        if(s.at<float>(vec[j].r, i) >= 0)
                            break;
                        chigh = i;
                    }
                }

                //
                bool already_marked = false;
                for(int i = clow; i <= chigh; ++i){
                    if(R.at<float>(vec[j].r, i)){
                        already_marked = true;
                        continue;
                    }
                    R.at<float>(vec[j].r, i) = 1;
                    falseCount--;
                }
                if(!already_marked){
                    ++l;
                }
            }
            ++j;
        }
        std::vector<std::vector<cv::Point2f>> t(fft_data.rows);

        // 在每个方位上寻找连续被标记的区域
        for(int i = 0; i < fft_data.rows; ++i){
            int str = 0;
            int end = 0;
            bool counting = false;
            for(int j = min_range; j < fft_data.cols; ++j){
                if(R.at<float>(i, j)){
                    if(!counting){
                        str = j;
                        end = j;
                        counting = true;
                    }else{
                        end = j;
                    }
                }else if(counting){
                    if(checkAdjacentMarked(R, i, str, end)){
                        int max_r = str;
                        getMaxInRegion(h, i, str, end, max_r);
                        t[i].push_back(cv::Point(i, max_r));
                    }
                    counting = false;
                }
            }
        }

        for(uint i = 0; i < t.size(); ++i){
            for(uint j = 0; j < t[i].size(); ++j){
                keyPointsCoor.push_back(std::make_pair(i, j));
            }
        }
    }

    std::vector<std::pair<int, int>> getKeyPointsCoor() const{
        return keyPointsCoor;
    }

private:
    // 查询邻近的方位上是否有被标记的区域
    static bool checkAdjacentMarked(Mat &R, int r, int str, int end){
        int below = r - 1;
        int above = r + 1;
        if(below < 0){
            below = R.rows - 1;
        }
        if(above >= R.rows){
            above = 0;
        }
        for(int i = str; i <= end; ++i){
            if(R.at<float>(below, i) || R.at<float>(above, i)) return true;
        }
        return false;
    }

    // 获得区域内最大的值
    static void getMaxInRegion(Mat &h, int r, int str, int end, int &max_r){
        int max_value = -1000;
        for(int i = str; i <= end; ++i){
            if(h.at<float>(r, i) > max_value){
                max_value = h.at<float>(r, i);
                max_r = r;
            }
        }
    }

private:
    Eigen::MatrixXd targets;
    std::vector<std::pair<int, int>> keyPointsCoor;

};



}











#endif