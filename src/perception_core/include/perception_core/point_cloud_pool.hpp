/**
 * @file point_cloud_pool.hpp
 * @brief 点云内存池，减少频繁的内存分配/释放
 *
 * 设计思路：
 * - 预分配固定数量的点云对象
 * - 使用对象池模式管理点云生命周期
 * - 支持不同大小的点云（小/中/大）
 * - 线程安全（可选）
 *
 * 使用场景：
 * - 地面分割输出（ground/non_ground）
 * - 聚类结果
 * - ROI裁剪
 * - 临时点云缓冲区
 */

#ifndef PERCEPTION_CORE_POINT_CLOUD_POOL_HPP
#define PERCEPTION_CORE_POINT_CLOUD_POOL_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <functional>

namespace perception {

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
// PCL使用boost::shared_ptr
using PointCloudPtr = typename PointCloud::Ptr;

/**
 * @brief 点云内存池配置
 */
struct PointCloudPoolConfig {
    // 小点云池（< 5k点）
    size_t small_pool_size = 10;        // 池大小
    size_t small_capacity = 5000;       // 预分配容量

    // 中等点云池（5k-30k点）
    size_t medium_pool_size = 8;
    size_t medium_capacity = 30000;

    // 大点云池（> 30k点）
    size_t large_pool_size = 4;
    size_t large_capacity = 100000;

    bool thread_safe = false;           // 是否线程安全
};

/**
 * @brief 点云智能指针包装器（RAII）
 *
 * 自动归还点云到内存池
 */
class PooledPointCloud {
public:
    using Ptr = std::shared_ptr<PooledPointCloud>;

    PooledPointCloud(PointCloudPtr cloud, std::function<void(PointCloudPtr)> deleter)
        : cloud_(cloud), deleter_(deleter) {}

    ~PooledPointCloud() {
        if (cloud_ && deleter_) {
            deleter_(cloud_);
        }
    }

    // 禁止拷贝
    PooledPointCloud(const PooledPointCloud&) = delete;
    PooledPointCloud& operator=(const PooledPointCloud&) = delete;

    // 允许移动
    PooledPointCloud(PooledPointCloud&& other) noexcept
        : cloud_(std::move(other.cloud_)), deleter_(std::move(other.deleter_)) {
        other.cloud_ = nullptr;
        other.deleter_ = nullptr;
    }

    PointCloudPtr get() { return cloud_; }
    const PointCloudPtr get() const { return cloud_; }

    PointCloud& operator*() { return *cloud_; }
    const PointCloud& operator*() const { return *cloud_; }

    PointCloud* operator->() { return cloud_.get(); }
    const PointCloud* operator->() const { return cloud_.get(); }

private:
    PointCloudPtr cloud_;
    std::function<void(PointCloudPtr)> deleter_;
};

/**
 * @brief 点云内存池
 */
class PointCloudPool {
public:
    explicit PointCloudPool(const PointCloudPoolConfig& config = PointCloudPoolConfig());
    ~PointCloudPool() = default;

    // 禁止拷贝和移动
    PointCloudPool(const PointCloudPool&) = delete;
    PointCloudPool& operator=(const PointCloudPool&) = delete;

    /**
     * @brief 获取点云（自动选择合适大小的池）
     * @param estimated_size 预估点数（用于选择池）
     * @return RAII包装的点云指针
     */
    PooledPointCloud::Ptr acquire(size_t estimated_size = 0);

    /**
     * @brief 获取小点云（< 5k点）
     */
    PooledPointCloud::Ptr acquireSmall();

    /**
     * @brief 获取中等点云（5k-30k点）
     */
    PooledPointCloud::Ptr acquireMedium();

    /**
     * @brief 获取大点云（> 30k点）
     */
    PooledPointCloud::Ptr acquireLarge();

    /**
     * @brief 获取统计信息
     */
    struct Stats {
        size_t small_available;
        size_t small_total;
        size_t medium_available;
        size_t medium_total;
        size_t large_available;
        size_t large_total;
        size_t total_acquisitions;
        size_t total_returns;
        size_t fallback_allocations;  // 池耗尽时的临时分配次数
    };

    Stats getStats() const;

    /**
     * @brief 重置统计信息
     */
    void resetStats();

    /**
     * @brief 清空所有池（释放内存）
     */
    void clear();

private:
    /**
     * @brief 单个池的实现
     */
    struct Pool {
        std::vector<PointCloudPtr> available;
        size_t capacity;
        size_t total_size;
        mutable std::mutex mutex;  // mutable for const methods

        Pool(size_t size, size_t cap) : capacity(cap), total_size(size) {
            available.reserve(size);
            for (size_t i = 0; i < size; ++i) {
                auto cloud = boost::make_shared<PointCloud>();
                cloud->reserve(capacity);
                available.push_back(cloud);
            }
        }

        PointCloudPtr acquire(bool thread_safe) {
            if (thread_safe) {
                std::lock_guard<std::mutex> lock(mutex);
                return acquireInternal();
            } else {
                return acquireInternal();
            }
        }

        void release(PointCloudPtr cloud, bool thread_safe) {
            // 清空点云但保留容量
            cloud->clear();

            if (thread_safe) {
                std::lock_guard<std::mutex> lock(mutex);
                available.push_back(cloud);
            } else {
                available.push_back(cloud);
            }
        }

        size_t availableCount(bool thread_safe) const {
            if (thread_safe) {
                std::lock_guard<std::mutex> lock(mutex);
                return available.size();
            } else {
                return available.size();
            }
        }

    private:
        PointCloudPtr acquireInternal() {
            if (!available.empty()) {
                auto cloud = available.back();
                available.pop_back();
                return cloud;
            }
            return nullptr;
        }
    };

    PointCloudPoolConfig config_;

    std::unique_ptr<Pool> small_pool_;
    std::unique_ptr<Pool> medium_pool_;
    std::unique_ptr<Pool> large_pool_;

    // 统计信息
    std::atomic<size_t> total_acquisitions_{0};
    std::atomic<size_t> total_returns_{0};
    std::atomic<size_t> fallback_allocations_{0};

    /**
     * @brief 归还点云到池
     */
    void releaseSmall(PointCloudPtr cloud);
    void releaseMedium(PointCloudPtr cloud);
    void releaseLarge(PointCloudPtr cloud);

    /**
     * @brief 创建临时点云（池耗尽时的fallback）
     */
    PooledPointCloud::Ptr createFallback(size_t capacity);
};

/**
 * @brief 全局点云内存池单例
 *
 * 使用示例：
 * @code
 * auto cloud = GlobalPointCloudPool::instance().acquireMedium();
 * // 使用 cloud->points, cloud->size() 等
 * // 离开作用域自动归还
 * @endcode
 */
class GlobalPointCloudPool {
public:
    static PointCloudPool& instance() {
        static PointCloudPool pool;
        return pool;
    }

private:
    GlobalPointCloudPool() = default;
};

}  // namespace perception

#endif  // PERCEPTION_CORE_POINT_CLOUD_POOL_HPP
