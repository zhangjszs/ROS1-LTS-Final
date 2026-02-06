#include <perception_core/point_cloud_pool.hpp>

namespace perception {

PointCloudPool::PointCloudPool(const PointCloudPoolConfig& config)
    : config_(config) {
    // 初始化三个池
    small_pool_ = std::make_unique<Pool>(config_.small_pool_size, config_.small_capacity);
    medium_pool_ = std::make_unique<Pool>(config_.medium_pool_size, config_.medium_capacity);
    large_pool_ = std::make_unique<Pool>(config_.large_pool_size, config_.large_capacity);
}

PooledPointCloud::Ptr PointCloudPool::acquire(size_t estimated_size) {
    if (estimated_size == 0 || estimated_size < 5000) {
        return acquireSmall();
    } else if (estimated_size < 30000) {
        return acquireMedium();
    } else {
        return acquireLarge();
    }
}

PooledPointCloud::Ptr PointCloudPool::acquireSmall() {
    total_acquisitions_++;

    auto cloud = small_pool_->acquire(config_.thread_safe);
    if (cloud) {
        auto deleter = [this](PointCloudPtr c) { this->releaseSmall(c); };
        return std::make_shared<PooledPointCloud>(cloud, deleter);
    }

    // 池耗尽，创建临时点云
    return createFallback(config_.small_capacity);
}

PooledPointCloud::Ptr PointCloudPool::acquireMedium() {
    total_acquisitions_++;

    auto cloud = medium_pool_->acquire(config_.thread_safe);
    if (cloud) {
        auto deleter = [this](PointCloudPtr c) { this->releaseMedium(c); };
        return std::make_shared<PooledPointCloud>(cloud, deleter);
    }

    return createFallback(config_.medium_capacity);
}

PooledPointCloud::Ptr PointCloudPool::acquireLarge() {
    total_acquisitions_++;

    auto cloud = large_pool_->acquire(config_.thread_safe);
    if (cloud) {
        auto deleter = [this](PointCloudPtr c) { this->releaseLarge(c); };
        return std::make_shared<PooledPointCloud>(cloud, deleter);
    }

    return createFallback(config_.large_capacity);
}

void PointCloudPool::releaseSmall(PointCloudPtr cloud) {
    total_returns_++;
    small_pool_->release(cloud, config_.thread_safe);
}

void PointCloudPool::releaseMedium(PointCloudPtr cloud) {
    total_returns_++;
    medium_pool_->release(cloud, config_.thread_safe);
}

void PointCloudPool::releaseLarge(PointCloudPtr cloud) {
    total_returns_++;
    large_pool_->release(cloud, config_.thread_safe);
}

PooledPointCloud::Ptr PointCloudPool::createFallback(size_t capacity) {
    fallback_allocations_++;

    auto cloud = boost::make_shared<PointCloud>();
    cloud->reserve(capacity);

    // Fallback点云不归还到池，直接释放
    auto deleter = [this](PointCloudPtr) {
        this->total_returns_++;
    };

    return std::make_shared<PooledPointCloud>(cloud, deleter);
}

PointCloudPool::Stats PointCloudPool::getStats() const {
    Stats stats;
    stats.small_available = small_pool_->availableCount(config_.thread_safe);
    stats.small_total = config_.small_pool_size;
    stats.medium_available = medium_pool_->availableCount(config_.thread_safe);
    stats.medium_total = config_.medium_pool_size;
    stats.large_available = large_pool_->availableCount(config_.thread_safe);
    stats.large_total = config_.large_pool_size;
    stats.total_acquisitions = total_acquisitions_.load();
    stats.total_returns = total_returns_.load();
    stats.fallback_allocations = fallback_allocations_.load();
    return stats;
}

void PointCloudPool::resetStats() {
    total_acquisitions_ = 0;
    total_returns_ = 0;
    fallback_allocations_ = 0;
}

void PointCloudPool::clear() {
    small_pool_ = std::make_unique<Pool>(config_.small_pool_size, config_.small_capacity);
    medium_pool_ = std::make_unique<Pool>(config_.medium_pool_size, config_.medium_capacity);
    large_pool_ = std::make_unique<Pool>(config_.large_pool_size, config_.large_capacity);
    resetStats();
}

}  // namespace perception
