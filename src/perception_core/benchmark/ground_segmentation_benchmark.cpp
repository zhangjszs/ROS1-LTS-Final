/**
 * @file ground_segmentation_benchmark.cpp
 * @brief 地面分割算法性能基准测试
 *
 * 支持三种地面分割算法的性能对比：
 * - FGS (Fast Ground Segmentation)
 * - RANSAC
 * - Patchwork++
 *
 * 用法:
 *   ./ground_segmentation_benchmark <pcd_file_or_directory> [iterations]
 *   ./ground_segmentation_benchmark --generate [num_points] [iterations]
 *
 * 输出:
 *   - 控制台统计报告
 *   - CSV文件 (benchmark_results.csv)
 */

#include <perception_core/fast_ground_segmentation.hpp>
#include <perception_core/patchworkpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <filesystem>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using Clock = std::chrono::high_resolution_clock;

namespace fs = std::filesystem;

// 性能统计结构
struct PerfStats {
    std::string algorithm_name;
    std::vector<double> times_ms;
    std::vector<double> ground_ratios;
    size_t total_points = 0;

    double mean_time() const {
        if (times_ms.empty()) return 0;
        return std::accumulate(times_ms.begin(), times_ms.end(), 0.0) / times_ms.size();
    }

    double max_time() const {
        if (times_ms.empty()) return 0;
        return *std::max_element(times_ms.begin(), times_ms.end());
    }

    double min_time() const {
        if (times_ms.empty()) return 0;
        return *std::min_element(times_ms.begin(), times_ms.end());
    }

    double stddev_time() const {
        if (times_ms.size() < 2) return 0;
        double mean = mean_time();
        double sq_sum = 0;
        for (double t : times_ms) {
            sq_sum += (t - mean) * (t - mean);
        }
        return std::sqrt(sq_sum / (times_ms.size() - 1));
    }

    double mean_ground_ratio() const {
        if (ground_ratios.empty()) return 0;
        return std::accumulate(ground_ratios.begin(), ground_ratios.end(), 0.0) / ground_ratios.size();
    }

    double throughput_fps() const {
        double mean_t = mean_time();
        return mean_t > 0 ? 1000.0 / mean_t : 0;
    }
};

// 生成合成点云（平面地面 + 随机障碍物）
PointCloud::Ptr generateSyntheticCloud(size_t num_points, double ground_ratio = 0.6) {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->reserve(num_points);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(-50.0, 50.0);
    std::uniform_real_distribution<> dist_y(-50.0, 50.0);
    std::uniform_real_distribution<> dist_ground_z(-0.15, 0.05);  // 地面噪声
    std::uniform_real_distribution<> dist_obstacle_z(0.1, 2.0);   // 障碍物高度
    std::uniform_real_distribution<> dist_intensity(10.0, 255.0);
    std::uniform_real_distribution<> dist_01(0.0, 1.0);

    size_t ground_points = static_cast<size_t>(num_points * ground_ratio);

    for (size_t i = 0; i < num_points; ++i) {
        PointType pt;
        pt.x = static_cast<float>(dist_x(gen));
        pt.y = static_cast<float>(dist_y(gen));
        pt.intensity = static_cast<float>(dist_intensity(gen));

        if (i < ground_points) {
            // 地面点（带坡度和噪声）
            double slope = 0.02;  // 2%坡度
            pt.z = static_cast<float>(slope * std::sqrt(pt.x*pt.x + pt.y*pt.y) + dist_ground_z(gen) - 0.135);
        } else {
            // 障碍物点
            pt.z = static_cast<float>(dist_obstacle_z(gen));
        }

        cloud->push_back(pt);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

// 加载PCD文件
std::vector<PointCloud::Ptr> loadPCDFiles(const std::string& path) {
    std::vector<PointCloud::Ptr> clouds;

    if (fs::is_directory(path)) {
        for (const auto& entry : fs::directory_iterator(path)) {
            if (entry.path().extension() == ".pcd") {
                PointCloud::Ptr cloud(new PointCloud);
                if (pcl::io::loadPCDFile(entry.path().string(), *cloud) == 0) {
                    clouds.push_back(cloud);
                    std::cout << "Loaded: " << entry.path().filename() << " (" << cloud->size() << " points)\n";
                }
            }
        }
    } else if (fs::exists(path) && fs::path(path).extension() == ".pcd") {
        PointCloud::Ptr cloud(new PointCloud);
        if (pcl::io::loadPCDFile(path, *cloud) == 0) {
            clouds.push_back(cloud);
            std::cout << "Loaded: " << path << " (" << cloud->size() << " points)\n";
        }
    }

    return clouds;
}

// FGS基准测试
PerfStats benchmarkFGS(const std::vector<PointCloud::Ptr>& clouds, int iterations) {
    PerfStats stats;
    stats.algorithm_name = "FGS";

    perception::FGSConfig config;
    config.num_sectors = 32;
    config.num_bins = 80;
    config.min_range = 0.1;
    config.max_range = 80.0;
    config.sensor_height = 0.135;
    config.th_ground = 0.08;
    config.near_distance = 2.0;
    config.th_ground_near = 0.12;
    config.use_neighbor_model = true;

    perception::FastGroundSegmentation fgs;
    fgs.configure(config);

    for (int iter = 0; iter < iterations; ++iter) {
        for (const auto& cloud : clouds) {
            PointCloud::Ptr ground(new PointCloud);
            PointCloud::Ptr non_ground(new PointCloud);

            auto start = Clock::now();
            fgs.segment(cloud, ground, non_ground);
            auto end = Clock::now();

            double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            stats.times_ms.push_back(elapsed_ms);
            stats.total_points += cloud->size();

            double ground_ratio = static_cast<double>(ground->size()) / cloud->size();
            stats.ground_ratios.push_back(ground_ratio);
        }
    }

    return stats;
}

// Patchwork++基准测试
PerfStats benchmarkPatchworkpp(const std::vector<PointCloud::Ptr>& clouds, int iterations) {
    PerfStats stats;
    stats.algorithm_name = "Patchwork++";

    patchwork::Params params;
    params.sensor_height = 0.135;
    params.num_iter = 4;
    params.num_lpr = 20;
    params.num_min_pts = 15;
    params.th_seeds = 0.05;
    params.th_dist = 0.05;
    params.max_range = 80.0;
    params.min_range = 0.1;

    patchwork::PatchWorkpp patchworkpp(params);

    for (int iter = 0; iter < iterations; ++iter) {
        for (const auto& cloud : clouds) {
            Eigen::MatrixXf cloud_matrix(cloud->size(), 4);
            for (size_t i = 0; i < cloud->size(); ++i) {
                cloud_matrix(i, 0) = cloud->points[i].x;
                cloud_matrix(i, 1) = cloud->points[i].y;
                cloud_matrix(i, 2) = cloud->points[i].z;
                cloud_matrix(i, 3) = cloud->points[i].intensity;
            }

            auto start = Clock::now();
            patchworkpp.estimateGround(cloud_matrix);
            auto end = Clock::now();

            double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            stats.times_ms.push_back(elapsed_ms);
            stats.total_points += cloud->size();

            Eigen::MatrixXf ground = patchworkpp.getGround();
            double ground_ratio = static_cast<double>(ground.rows()) / cloud->size();
            stats.ground_ratios.push_back(ground_ratio);
        }
    }

    return stats;
}

// 打印统计报告
void printReport(const std::vector<PerfStats>& all_stats, size_t cloud_size) {
    std::cout << "\n";
    std::cout << "================================================================================\n";
    std::cout << "                    地面分割算法性能基准测试报告\n";
    std::cout << "================================================================================\n";
    std::cout << "点云规模: " << cloud_size << " points\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(15) << "算法"
              << std::right << std::setw(12) << "平均(ms)"
              << std::setw(12) << "最小(ms)"
              << std::setw(12) << "最大(ms)"
              << std::setw(12) << "标准差(ms)"
              << std::setw(10) << "FPS"
              << std::setw(12) << "地面比例"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    for (const auto& stats : all_stats) {
        std::cout << std::left << std::setw(15) << stats.algorithm_name
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(12) << stats.mean_time()
                  << std::setw(12) << stats.min_time()
                  << std::setw(12) << stats.max_time()
                  << std::setw(12) << stats.stddev_time()
                  << std::setw(10) << stats.throughput_fps()
                  << std::setw(11) << (stats.mean_ground_ratio() * 100) << "%"
                  << "\n";
    }

    std::cout << "================================================================================\n";
}

// 保存CSV报告
void saveCSV(const std::vector<PerfStats>& all_stats, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    file << "algorithm,mean_ms,min_ms,max_ms,stddev_ms,fps,ground_ratio\n";
    for (const auto& stats : all_stats) {
        file << stats.algorithm_name << ","
             << std::fixed << std::setprecision(3)
             << stats.mean_time() << ","
             << stats.min_time() << ","
             << stats.max_time() << ","
             << stats.stddev_time() << ","
             << stats.throughput_fps() << ","
             << stats.mean_ground_ratio() << "\n";
    }

    file.close();
    std::cout << "Results saved to: " << filename << "\n";
}

void printUsage(const char* prog) {
    std::cout << "用法:\n";
    std::cout << "  " << prog << " <pcd_file_or_directory> [iterations]\n";
    std::cout << "  " << prog << " --generate [num_points] [iterations]\n";
    std::cout << "\n";
    std::cout << "示例:\n";
    std::cout << "  " << prog << " /path/to/cloud.pcd 100\n";
    std::cout << "  " << prog << " /path/to/pcd_folder/ 50\n";
    std::cout << "  " << prog << " --generate 50000 100\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::vector<PointCloud::Ptr> clouds;
    int iterations = 100;
    size_t avg_cloud_size = 0;

    if (std::string(argv[1]) == "--generate") {
        size_t num_points = (argc > 2) ? std::stoul(argv[2]) : 50000;
        iterations = (argc > 3) ? std::stoi(argv[3]) : 100;

        std::cout << "生成合成点云: " << num_points << " points\n";
        clouds.push_back(generateSyntheticCloud(num_points));
        avg_cloud_size = num_points;
    } else {
        clouds = loadPCDFiles(argv[1]);
        iterations = (argc > 2) ? std::stoi(argv[2]) : 100;

        if (clouds.empty()) {
            std::cerr << "未找到有效的PCD文件: " << argv[1] << "\n";
            return 1;
        }

        for (const auto& c : clouds) {
            avg_cloud_size += c->size();
        }
        avg_cloud_size /= clouds.size();
    }

    std::cout << "\n开始基准测试 (迭代次数: " << iterations << ")\n";
    std::cout << "----------------------------------------\n";

    std::vector<PerfStats> all_stats;

    // FGS测试
    std::cout << "测试 FGS...\n";
    all_stats.push_back(benchmarkFGS(clouds, iterations));

    // Patchwork++测试
    std::cout << "测试 Patchwork++...\n";
    all_stats.push_back(benchmarkPatchworkpp(clouds, iterations));

    // 打印报告
    printReport(all_stats, avg_cloud_size);

    // 保存CSV
    saveCSV(all_stats, "benchmark_results.csv");

    return 0;
}
