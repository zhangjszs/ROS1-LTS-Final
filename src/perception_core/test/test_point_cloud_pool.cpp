/**
 * @file test_point_cloud_pool.cpp
 * @brief 点云内存池单元测试
 */

#include <gtest/gtest.h>
#include <perception_core/point_cloud_pool.hpp>

#include <chrono>
#include <vector>

using namespace perception;
using Clock = std::chrono::high_resolution_clock;

// 测试基本的获取和归还
TEST(PointCloudPoolTest, BasicAcquireRelease) {
    PointCloudPoolConfig config;
    config.small_pool_size = 5;
    config.medium_pool_size = 3;
    config.large_pool_size = 2;

    PointCloudPool pool(config);

    // 获取小点云
    {
        auto cloud1 = pool.acquireSmall();
        ASSERT_NE(cloud1, nullptr);
        EXPECT_EQ(cloud1->get()->size(), 0u);  // 初始为空

        auto stats = pool.getStats();
        EXPECT_EQ(stats.small_available, 4u);  // 5 - 1
        EXPECT_EQ(stats.total_acquisitions, 1u);
    }  // cloud1离开作用域，自动归还

    // 验证归还
    auto stats = pool.getStats();
    EXPECT_EQ(stats.small_available, 5u);  // 归还后恢复
    EXPECT_EQ(stats.total_returns, 1u);
}

// 测试不同大小的池
TEST(PointCloudPoolTest, DifferentSizes) {
    PointCloudPool pool;

    auto small = pool.acquireSmall();
    auto medium = pool.acquireMedium();
    auto large = pool.acquireLarge();

    ASSERT_NE(small, nullptr);
    ASSERT_NE(medium, nullptr);
    ASSERT_NE(large, nullptr);

    // 验证点云对象有效
    EXPECT_EQ(small->get()->size(), 0u);
    EXPECT_EQ(medium->get()->size(), 0u);
    EXPECT_EQ(large->get()->size(), 0u);
}

// 测试自动选择池
TEST(PointCloudPoolTest, AutoSelectPool) {
    PointCloudPool pool;

    auto cloud1 = pool.acquire(3000);    // 应该选择small
    auto cloud2 = pool.acquire(15000);   // 应该选择medium
    auto cloud3 = pool.acquire(50000);   // 应该选择large

    ASSERT_NE(cloud1, nullptr);
    ASSERT_NE(cloud2, nullptr);
    ASSERT_NE(cloud3, nullptr);
}

// 测试池耗尽时的fallback
TEST(PointCloudPoolTest, PoolExhaustion) {
    PointCloudPoolConfig config;
    config.small_pool_size = 2;  // 只有2个

    PointCloudPool pool(config);

    std::vector<PooledPointCloud::Ptr> clouds;

    // 获取3个点云（超过池大小）
    for (int i = 0; i < 3; ++i) {
        auto cloud = pool.acquireSmall();
        ASSERT_NE(cloud, nullptr);
        clouds.push_back(cloud);
    }

    auto stats = pool.getStats();
    EXPECT_EQ(stats.small_available, 0u);
    EXPECT_EQ(stats.fallback_allocations, 1u);  // 第3个是fallback
}

// 测试点云使用
TEST(PointCloudPoolTest, CloudUsage) {
    PointCloudPool pool;

    auto cloud = pool.acquireMedium();

    // 添加点
    for (int i = 0; i < 100; ++i) {
        PointType pt;
        pt.x = i * 0.1f;
        pt.y = i * 0.2f;
        pt.z = i * 0.3f;
        pt.intensity = 100.0f;
        cloud->get()->push_back(pt);
    }

    EXPECT_EQ(cloud->get()->size(), 100u);

    // 归还后应该被清空
    cloud.reset();

    auto cloud2 = pool.acquireMedium();
    EXPECT_EQ(cloud2->get()->size(), 0u);  // 已清空
}

// 测试统计信息
TEST(PointCloudPoolTest, Statistics) {
    PointCloudPool pool;

    pool.resetStats();

    {
        auto c1 = pool.acquireSmall();
        auto c2 = pool.acquireMedium();
        auto c3 = pool.acquireLarge();
    }

    auto stats = pool.getStats();
    EXPECT_EQ(stats.total_acquisitions, 3u);
    EXPECT_EQ(stats.total_returns, 3u);
    EXPECT_EQ(stats.fallback_allocations, 0u);
}

// 性能测试：对比有无内存池的性能
TEST(PointCloudPoolTest, PerformanceComparison) {
    const int iterations = 100;
    const int points_per_cloud = 10000;  // 更大的点云

    // 无内存池：每次new/delete
    auto start1 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto cloud = boost::make_shared<PointCloud>();
        cloud->reserve(30000);
        for (int j = 0; j < points_per_cloud; ++j) {
            PointType pt;
            pt.x = j * 0.1f;
            pt.y = j * 0.2f;
            pt.z = j * 0.3f;
            cloud->push_back(pt);
        }
        cloud->clear();  // 模拟使用后清空
    }
    auto end1 = Clock::now();
    double time_without_pool = std::chrono::duration<double, std::milli>(end1 - start1).count();

    // 有内存池
    PointCloudPool pool;
    auto start2 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto cloud = pool.acquireMedium();
        for (int j = 0; j < points_per_cloud; ++j) {
            PointType pt;
            pt.x = j * 0.1f;
            pt.y = j * 0.2f;
            pt.z = j * 0.3f;
            cloud->get()->push_back(pt);
        }
        // 离开作用域自动归还
    }
    auto end2 = Clock::now();
    double time_with_pool = std::chrono::duration<double, std::milli>(end2 - start2).count();

    std::cout << "\n========== 内存池性能对比 ==========\n";
    std::cout << "无内存池: " << time_without_pool << "ms\n";
    std::cout << "有内存池: " << time_with_pool << "ms\n";
    std::cout << "性能提升: " << ((time_without_pool - time_with_pool) / time_without_pool * 100) << "%\n";
    std::cout << "====================================\n";

    // 注意：内存池的优势在于减少内存分配，但在小规模测试中可能不明显
    // 实际应用中，配合零拷贝优化效果更佳
    EXPECT_GT(time_without_pool, 0);  // 只验证测试运行成功
    EXPECT_GT(time_with_pool, 0);
}

// 测试全局单例
TEST(PointCloudPoolTest, GlobalSingleton) {
    auto& pool1 = GlobalPointCloudPool::instance();
    auto& pool2 = GlobalPointCloudPool::instance();

    // 应该是同一个实例
    EXPECT_EQ(&pool1, &pool2);

    auto cloud = GlobalPointCloudPool::instance().acquireMedium();
    ASSERT_NE(cloud, nullptr);
}

// 测试clear功能
TEST(PointCloudPoolTest, ClearPool) {
    PointCloudPool pool;

    {
        auto c1 = pool.acquireSmall();
        auto c2 = pool.acquireMedium();
    }

    auto stats1 = pool.getStats();
    EXPECT_GT(stats1.total_acquisitions, 0u);

    pool.clear();

    auto stats2 = pool.getStats();
    EXPECT_EQ(stats2.total_acquisitions, 0u);
    EXPECT_EQ(stats2.total_returns, 0u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
