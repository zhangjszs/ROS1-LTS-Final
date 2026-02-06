/**
 * @file test_ground_segmentation_perf.cpp
 * @brief 地面分割算法性能单元测试
 *
 * 使用GTest框架测试FGS、RANSAC、Patchwork++的性能
 * 运行: catkin run_tests perception_core
 */

#include <gtest/gtest.h>

#include <perception_core/fast_ground_segmentation.hpp>
#include <perception_core/patchworkpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <random>
#include <vector>
#include <numeric>
#include <cmath>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using Clock = std::chrono::high_resolution_clock;

namespace {

// 生成合成点云
PointCloud::Ptr generateTestCloud(size_t num_points, double ground_ratio = 0.6) {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->reserve(num_points);

    std::mt19937 gen(42);  // 固定种子保证可重复性
    std::uniform_real_distribution<> dist_x(-40.0, 40.0);
    std::uniform_real_distribution<> dist_y(-40.0, 40.0);
    std::uniform_real_distribution<> dist_ground_z(-0.12, 0.03);
    std::uniform_real_distribution<> dist_obstacle_z(0.1, 1.5);
    std::uniform_real_distribution<> dist_intensity(10.0, 200.0);

    size_t ground_points = static_cast<size_t>(num_points * ground_ratio);

    for (size_t i = 0; i < num_points; ++i) {
        PointType pt;
        pt.x = static_cast<float>(dist_x(gen));
        pt.y = static_cast<float>(dist_y(gen));
        pt.intensity = static_cast<float>(dist_intensity(gen));

        if (i < ground_points) {
            double slope = 0.015;
            pt.z = static_cast<float>(slope * std::sqrt(pt.x*pt.x + pt.y*pt.y) + dist_ground_z(gen) - 0.135);
        } else {
            pt.z = static_cast<float>(dist_obstacle_z(gen));
        }

        cloud->push_back(pt);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

// 计算统计量
struct TimeStats {
    double mean;
    double stddev;
    double max_val;
    double min_val;

    static TimeStats compute(const std::vector<double>& times) {
        TimeStats stats;
        if (times.empty()) {
            stats.mean = stats.stddev = stats.max_val = stats.min_val = 0;
            return stats;
        }

        stats.mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        stats.max_val = *std::max_element(times.begin(), times.end());
        stats.min_val = *std::min_element(times.begin(), times.end());

        if (times.size() > 1) {
            double sq_sum = 0;
            for (double t : times) {
                sq_sum += (t - stats.mean) * (t - stats.mean);
            }
            stats.stddev = std::sqrt(sq_sum / (times.size() - 1));
        } else {
            stats.stddev = 0;
        }

        return stats;
    }
};

}  // namespace

// ============== FGS 性能测试 ==============

class FGSPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.num_sectors = 32;
        config_.num_bins = 80;
        config_.min_range = 0.1;
        config_.max_range = 80.0;
        config_.sensor_height = 0.135;
        config_.th_ground = 0.08;
        config_.near_distance = 2.0;
        config_.th_ground_near = 0.12;
        config_.use_neighbor_model = true;

        fgs_.configure(config_);
    }

    perception::FGSConfig config_;
    perception::FastGroundSegmentation fgs_;
};

// 测试小规模点云性能
TEST_F(FGSPerformanceTest, SmallCloud_10k) {
    const size_t num_points = 10000;
    const int iterations = 50;
    const double max_allowed_ms = 5.0;  // 性能阈值

    auto cloud = generateTestCloud(num_points);
    std::vector<double> times;
    times.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        PointCloud::Ptr ground(new PointCloud);
        PointCloud::Ptr non_ground(new PointCloud);

        auto start = Clock::now();
        fgs_.segment(cloud, ground, non_ground);
        auto end = Clock::now();

        double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
        times.push_back(elapsed);

        // 验证输出有效性
        EXPECT_GT(ground->size(), 0u);
        EXPECT_GT(non_ground->size(), 0u);
        EXPECT_EQ(ground->size() + non_ground->size(), cloud->size());
    }

    auto stats = TimeStats::compute(times);

    std::cout << "[FGS 10k] Mean: " << stats.mean << "ms, "
              << "Max: " << stats.max_val << "ms, "
              << "Stddev: " << stats.stddev << "ms\n";

    EXPECT_LT(stats.mean, max_allowed_ms) << "FGS平均耗时超过阈值";
}

// 测试中等规模点云性能
TEST_F(FGSPerformanceTest, MediumCloud_50k) {
    const size_t num_points = 50000;
    const int iterations = 30;
    const double max_allowed_ms = 10.0;

    auto cloud = generateTestCloud(num_points);
    std::vector<double> times;
    times.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        PointCloud::Ptr ground(new PointCloud);
        PointCloud::Ptr non_ground(new PointCloud);

        auto start = Clock::now();
        fgs_.segment(cloud, ground, non_ground);
        auto end = Clock::now();

        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    auto stats = TimeStats::compute(times);

    std::cout << "[FGS 50k] Mean: " << stats.mean << "ms, "
              << "Max: " << stats.max_val << "ms, "
              << "Stddev: " << stats.stddev << "ms\n";

    EXPECT_LT(stats.mean, max_allowed_ms) << "FGS平均耗时超过阈值";
}

// 测试大规模点云性能
TEST_F(FGSPerformanceTest, LargeCloud_100k) {
    const size_t num_points = 100000;
    const int iterations = 20;
    const double max_allowed_ms = 20.0;

    auto cloud = generateTestCloud(num_points);
    std::vector<double> times;
    times.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        PointCloud::Ptr ground(new PointCloud);
        PointCloud::Ptr non_ground(new PointCloud);

        auto start = Clock::now();
        fgs_.segment(cloud, ground, non_ground);
        auto end = Clock::now();

        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    auto stats = TimeStats::compute(times);

    std::cout << "[FGS 100k] Mean: " << stats.mean << "ms, "
              << "Max: " << stats.max_val << "ms, "
              << "Stddev: " << stats.stddev << "ms\n";

    EXPECT_LT(stats.mean, max_allowed_ms) << "FGS平均耗时超过阈值";
}

// 测试地面点比例合理性
TEST_F(FGSPerformanceTest, GroundRatioReasonable) {
    auto cloud = generateTestCloud(30000, 0.6);  // 60%地面点

    PointCloud::Ptr ground(new PointCloud);
    PointCloud::Ptr non_ground(new PointCloud);

    fgs_.segment(cloud, ground, non_ground);

    double ground_ratio = static_cast<double>(ground->size()) / cloud->size();

    std::cout << "[FGS] Ground ratio: " << (ground_ratio * 100) << "%\n";

    // 地面点比例应该在合理范围内
    // 注意：合成点云的地面点分布与真实场景不同，阈值放宽
    EXPECT_GT(ground_ratio, 0.1) << "地面点比例过低";
    EXPECT_LT(ground_ratio, 0.95) << "地面点比例过高";
}

// ============== Patchwork++ 性能测试 ==============

class PatchworkppPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        params_.sensor_height = 0.135;
        params_.num_iter = 4;
        params_.num_lpr = 20;
        params_.num_min_pts = 15;
        params_.th_seeds = 0.05;
        params_.th_dist = 0.05;
        params_.max_range = 80.0;
        params_.min_range = 0.1;

        patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params_);
    }

    patchwork::Params params_;
    std::unique_ptr<patchwork::PatchWorkpp> patchworkpp_;
};

// 测试小规模点云性能
TEST_F(PatchworkppPerformanceTest, SmallCloud_10k) {
    const size_t num_points = 10000;
    const int iterations = 30;
    const double max_allowed_ms = 100.0;  // Patchwork++较慢，放宽阈值

    auto cloud = generateTestCloud(num_points);

    // 转换为Eigen矩阵
    Eigen::MatrixXf cloud_matrix(cloud->size(), 4);
    for (size_t i = 0; i < cloud->size(); ++i) {
        cloud_matrix(i, 0) = cloud->points[i].x;
        cloud_matrix(i, 1) = cloud->points[i].y;
        cloud_matrix(i, 2) = cloud->points[i].z;
        cloud_matrix(i, 3) = cloud->points[i].intensity;
    }

    std::vector<double> times;
    times.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        auto start = Clock::now();
        patchworkpp_->estimateGround(cloud_matrix);
        auto end = Clock::now();

        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    auto stats = TimeStats::compute(times);

    std::cout << "[Patchwork++ 10k] Mean: " << stats.mean << "ms, "
              << "Max: " << stats.max_val << "ms, "
              << "Stddev: " << stats.stddev << "ms\n";

    EXPECT_LT(stats.mean, max_allowed_ms) << "Patchwork++平均耗时超过阈值";
}

// 测试中等规模点云性能
TEST_F(PatchworkppPerformanceTest, MediumCloud_50k) {
    const size_t num_points = 50000;
    const int iterations = 20;
    const double max_allowed_ms = 250.0;  // Patchwork++较慢，放宽阈值

    auto cloud = generateTestCloud(num_points);

    Eigen::MatrixXf cloud_matrix(cloud->size(), 4);
    for (size_t i = 0; i < cloud->size(); ++i) {
        cloud_matrix(i, 0) = cloud->points[i].x;
        cloud_matrix(i, 1) = cloud->points[i].y;
        cloud_matrix(i, 2) = cloud->points[i].z;
        cloud_matrix(i, 3) = cloud->points[i].intensity;
    }

    std::vector<double> times;
    times.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        auto start = Clock::now();
        patchworkpp_->estimateGround(cloud_matrix);
        auto end = Clock::now();

        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    auto stats = TimeStats::compute(times);

    std::cout << "[Patchwork++ 50k] Mean: " << stats.mean << "ms, "
              << "Max: " << stats.max_val << "ms, "
              << "Stddev: " << stats.stddev << "ms\n";

    EXPECT_LT(stats.mean, max_allowed_ms) << "Patchwork++平均耗时超过阈值";
}

// ============== 算法对比测试 ==============

TEST(AlgorithmComparison, FGS_vs_Patchworkpp) {
    const size_t num_points = 30000;
    const int iterations = 20;

    auto cloud = generateTestCloud(num_points);

    // FGS
    perception::FGSConfig fgs_config;
    fgs_config.num_sectors = 32;
    fgs_config.num_bins = 80;
    fgs_config.min_range = 0.1;
    fgs_config.max_range = 80.0;
    fgs_config.sensor_height = 0.135;

    perception::FastGroundSegmentation fgs;
    fgs.configure(fgs_config);

    std::vector<double> fgs_times;
    for (int i = 0; i < iterations; ++i) {
        PointCloud::Ptr ground(new PointCloud);
        PointCloud::Ptr non_ground(new PointCloud);

        auto start = Clock::now();
        fgs.segment(cloud, ground, non_ground);
        auto end = Clock::now();

        fgs_times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    // Patchwork++
    patchwork::Params pp_params;
    pp_params.sensor_height = 0.135;
    patchwork::PatchWorkpp patchworkpp(pp_params);

    Eigen::MatrixXf cloud_matrix(cloud->size(), 4);
    for (size_t i = 0; i < cloud->size(); ++i) {
        cloud_matrix(i, 0) = cloud->points[i].x;
        cloud_matrix(i, 1) = cloud->points[i].y;
        cloud_matrix(i, 2) = cloud->points[i].z;
        cloud_matrix(i, 3) = cloud->points[i].intensity;
    }

    std::vector<double> pp_times;
    for (int i = 0; i < iterations; ++i) {
        auto start = Clock::now();
        patchworkpp.estimateGround(cloud_matrix);
        auto end = Clock::now();

        pp_times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    auto fgs_stats = TimeStats::compute(fgs_times);
    auto pp_stats = TimeStats::compute(pp_times);

    std::cout << "\n========== 算法对比 (30k points) ==========\n";
    std::cout << "FGS:         " << fgs_stats.mean << "ms (stddev: " << fgs_stats.stddev << "ms)\n";
    std::cout << "Patchwork++: " << pp_stats.mean << "ms (stddev: " << pp_stats.stddev << "ms)\n";
    std::cout << "FGS加速比:   " << (pp_stats.mean / fgs_stats.mean) << "x\n";
    std::cout << "=============================================\n";

    // FGS应该比Patchwork++快
    EXPECT_LT(fgs_stats.mean, pp_stats.mean) << "FGS应该比Patchwork++更快";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
