#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>

#include <vector>
#include <numeric>

namespace perception {

/// Fast Euclidean Clustering (FEC)
/// Uses dual-radius BFS + Union-Find for faster clustering.
/// Inner radius r_inner = (1-quality)*d consumes points (marks removed),
/// outer ring continues BFS expansion. Different BFS seeds are merged
/// via Union-Find. Guarantees: only over-segmentation, never under-segmentation.
/// API compatible with pcl::EuclideanClusterExtraction.
template <typename PointT>
class FastEuclideanClustering {
public:
    FastEuclideanClustering() = default;

    void setClusterTolerance(double d) { tolerance_ = d; }
    void setMinClusterSize(int min_size) { min_size_ = min_size; }
    void setMaxClusterSize(int max_size) { max_size_ = max_size; }
    void setQuality(double q) { quality_ = std::max(0.0, std::min(0.9, q)); }
    void setSearchMethod(const typename pcl::search::KdTree<PointT>::Ptr& tree) { tree_ = tree; }
    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud) { cloud_ = cloud; }

    void segment(std::vector<pcl::PointIndices>& clusters) {
        clusters.clear();
        if (!cloud_ || cloud_->points.empty()) return;

        const size_t n = cloud_->points.size();
        const double d = tolerance_;
        const double r_inner = (1.0 - quality_) * d;
        const double r_inner_sq = r_inner * r_inner;

        // Union-Find data structure
        std::vector<int> parent(n);
        std::vector<int> rank(n, 0);
        std::iota(parent.begin(), parent.end(), 0);

        auto find = [&](int x) -> int {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];  // path compression
                x = parent[x];
            }
            return x;
        };

        auto unite = [&](int a, int b) {
            a = find(a);
            b = find(b);
            if (a == b) return;
            if (rank[a] < rank[b]) std::swap(a, b);
            parent[b] = a;
            if (rank[a] == rank[b]) rank[a]++;
        };

        // Build KdTree if not provided
        if (!tree_) {
            tree_.reset(new pcl::search::KdTree<PointT>);
        }
        tree_->setInputCloud(cloud_);

        std::vector<bool> removed(n, false);  // consumed by inner radius
        std::vector<bool> visited(n, false);

        std::vector<int> neighbors;
        std::vector<float> distances;
        std::vector<int> bfs_queue;

        for (size_t i = 0; i < n; ++i) {
            if (removed[i]) continue;

            // Start BFS from this seed
            bfs_queue.clear();
            bfs_queue.push_back(static_cast<int>(i));
            visited[i] = true;

            size_t head = 0;
            while (head < bfs_queue.size()) {
                int curr = bfs_queue[head++];

                neighbors.clear();
                distances.clear();
                tree_->radiusSearch(cloud_->points[curr], d, neighbors, distances);

                for (size_t k = 0; k < neighbors.size(); ++k) {
                    int nb = neighbors[k];
                    if (nb == curr) continue;

                    // Union all neighbors within outer radius
                    unite(static_cast<int>(i), nb);

                    if (distances[k] <= r_inner_sq) {
                        // Inner radius: consume point
                        removed[nb] = true;
                    }

                    if (!visited[nb] && !removed[nb]) {
                        // Outer ring: continue BFS
                        visited[nb] = true;
                        bfs_queue.push_back(nb);
                    }
                }
            }
        }

        // Collect clusters by root
        std::unordered_map<int, std::vector<int>> cluster_map;
        cluster_map.reserve(n / 4);
        for (size_t i = 0; i < n; ++i) {
            cluster_map[find(static_cast<int>(i))].push_back(static_cast<int>(i));
        }

        clusters.reserve(cluster_map.size());
        for (auto& kv : cluster_map) {
            int sz = static_cast<int>(kv.second.size());
            if (sz >= min_size_ && sz <= max_size_) {
                pcl::PointIndices pi;
                pi.indices = std::move(kv.second);
                clusters.push_back(std::move(pi));
            }
        }
    }

private:
    double tolerance_ = 0.3;
    int min_size_ = 1;
    int max_size_ = 1000;
    double quality_ = 0.3;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    typename pcl::PointCloud<PointT>::Ptr cloud_;
};

}  // namespace perception
