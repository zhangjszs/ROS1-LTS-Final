#include <perception_core/fast_ground_segmentation.hpp>
#include <numeric>

namespace perception {

FastGroundSegmentation::FastGroundSegmentation() {
    // 使用默认配置初始化
    configure(FGSConfig());
}

void FastGroundSegmentation::configure(const FGSConfig& config) {
    config_ = config;

    // 预计算常量
    sector_angle_ = 2.0 * M_PI / config_.num_sectors;
    bin_size_ = (config_.max_range - config_.min_range) / config_.num_bins;
    inv_sector_angle_ = 1.0 / sector_angle_;
    inv_bin_size_ = 1.0 / bin_size_;

    // 初始化栅格
    initPolarGrid();
    initialized_ = true;
}

void FastGroundSegmentation::initPolarGrid() {
    // 预分配极坐标栅格
    polar_grid_.resize(config_.num_sectors);
    for (int s = 0; s < config_.num_sectors; ++s) {
        polar_grid_[s].resize(config_.num_bins);
    }

    // 预分配扇区模型
    sector_models_.resize(config_.num_sectors);
}

void FastGroundSegmentation::resetPolarGrid() {
    for (int s = 0; s < config_.num_sectors; ++s) {
        for (int b = 0; b < config_.num_bins; ++b) {
            polar_grid_[s][b].reset();
        }
        sector_models_[s].valid = false;
        sector_models_[s].segments.clear();
    }
}

void FastGroundSegmentation::fillPolarGrid(const pcl::PointCloud<PointType>::Ptr& input) {
    for (const auto& pt : input->points) {
        // 跳过无效点
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        int sector_idx, bin_idx;
        double distance;
        getGridIndex(pt, sector_idx, bin_idx, distance);

        // 跳过超出范围的点
        if (distance < config_.min_range || distance > config_.max_range) {
            continue;
        }

        // 添加到对应bin
        polar_grid_[sector_idx][bin_idx].addPoint(pt.z);
    }
}

void FastGroundSegmentation::fitGroundModels() {
    // 首先拟合每个扇区
    for (int s = 0; s < config_.num_sectors; ++s) {
        fitSectorModel(s);
    }

    // 然后对无效扇区进行插值
    if (config_.use_neighbor_model) {
        interpolateInvalidModels();
    }

    // 扇区间平滑 (Himmelsbach 2010)
    if (config_.enable_sector_smoothing) {
        smoothSectorModels();
    }

    // 帧间时序平滑 (抑制闪烁)
    if (config_.enable_temporal_smoothing) {
        temporalSmooth();
    }
}

void FastGroundSegmentation::fitSectorModel(int sector_idx) {
    auto& model = sector_models_[sector_idx];
    const auto& bins = polar_grid_[sector_idx];

    model.segments.clear();

    // 在线最小二乘累加器
    struct OnlineFitter {
        double sum_r = 0.0, sum_z = 0.0, sum_rr = 0.0, sum_rz = 0.0;
        int n = 0;
        int bin_start = 0;
        int bin_end = 0;

        void reset(int start) {
            sum_r = sum_z = sum_rr = sum_rz = 0.0;
            n = 0;
            bin_start = start;
            bin_end = start;
        }

        void add(double r, double z, int bin) {
            sum_r += r; sum_z += z; sum_rr += r * r; sum_rz += r * z;
            n++;
            bin_end = bin + 1;
        }

        bool fit(double& a, double& b) const {
            if (n < 2) return false;
            double denom = n * sum_rr - sum_r * sum_r;
            if (std::abs(denom) < 1e-10) {
                a = 0.0;
                b = sum_z / n;
                return true;
            }
            a = (n * sum_rz - sum_r * sum_z) / denom;
            b = (sum_z * sum_rr - sum_r * sum_rz) / denom;
            return true;
        }
    };

    // 辅助 lambda：保存当前线段到 model.segments
    auto saveFitter = [&](OnlineFitter& f) {
        double seg_a, seg_b;
        if (f.n >= 2 && f.fit(seg_a, seg_b)) {
            // 法向量约束
            double normal_z = std::cos(std::atan(std::abs(seg_a)));
            if (normal_z < config_.min_normal_z) {
                return;  // 丢弃法向量不合格的线段
            }
            // 坡度约束
            if (std::abs(seg_a) > config_.max_slope) {
                seg_a = (seg_a > 0) ? config_.max_slope : -config_.max_slope;
            }
            if (static_cast<int>(model.segments.size()) < config_.max_segments_per_sector) {
                LineSegment seg;
                seg.a = seg_a; seg.b = seg_b;
                seg.bin_start = f.bin_start;
                seg.bin_end = f.bin_end;
                seg.num_points = f.n;
                seg.valid = true;
                model.segments.push_back(seg);
            }
        }
    };

    OnlineFitter fitter;
    fitter.reset(0);
    bool segment_active = false;

    // 全局拟合累加器（独立于线段，用于帧间平滑和回退）
    double g_sum_r = 0.0, g_sum_z = 0.0, g_sum_rr = 0.0, g_sum_rz = 0.0;
    int g_n = 0;
    double prev_z = -config_.sensor_height;
    bool has_prev = false;
    int consecutive_skip = 0;  // 连续跳过的 bin 数

    for (int b = 0; b < config_.num_bins; ++b) {
        if (!bins[b].valid || bins[b].count < config_.min_points_per_bin) {
            consecutive_skip++;
            // 连续多个空 bin 也应断开线段（避免跨越大间隙）
            if (segment_active && consecutive_skip >= 3) {
                saveFitter(fitter);
                segment_active = false;
            }
            continue;
        }

        double r = config_.min_range + (b + 0.5) * bin_size_;
        double z = config_.use_lowest_n_mean ? bins[b].getPrototypeZ() : bins[b].min_z;

        // 高度连续性检查
        if (has_prev && std::abs(z - prev_z) > config_.max_height_diff) {
            // 高度突变：断开当前线段，跳过此 bin
            if (segment_active) {
                saveFitter(fitter);
                segment_active = false;
            }
            consecutive_skip++;
            continue;
        }

        consecutive_skip = 0;

        // 全局累加（不受线段断裂影响）
        g_sum_r += r; g_sum_z += z; g_sum_rr += r * r; g_sum_rz += r * z;
        g_n++;
        prev_z = z;
        has_prev = true;

        // 增量线段生长
        if (!segment_active) {
            fitter.reset(b);
            fitter.add(r, z, b);
            segment_active = true;
            continue;
        }

        // 尝试将当前 bin 加入当前线段
        double seg_a, seg_b;
        if (fitter.n >= 2 && fitter.fit(seg_a, seg_b)) {
            double expected_z = seg_a * r + seg_b;
            double residual = std::abs(z - expected_z);

            if (residual < config_.segment_merge_dist) {
                fitter.add(r, z, b);
            } else {
                // 偏差过大：保存当前线段，开始新线段
                saveFitter(fitter);
                fitter.reset(b);
                fitter.add(r, z, b);
            }
        } else {
            // 还不够2个点，继续累积
            fitter.add(r, z, b);
        }
    }

    // 保存最后一个线段
    if (segment_active) {
        saveFitter(fitter);
    }

    // 全局模型：独立于线段，用于帧间 EMA 平滑和线段覆盖不到时的回退
    // 不被任何线段覆盖，保持全局拟合结果
    if (g_n < 2) {
        model.valid = false;
        model.a = 0.0;
        model.b = -config_.sensor_height;
        return;
    }

    double denom = g_n * g_sum_rr - g_sum_r * g_sum_r;
    if (std::abs(denom) < 1e-10) {
        model.a = 0.0;
        model.b = g_sum_z / g_n;
    } else {
        model.a = (g_n * g_sum_rz - g_sum_r * g_sum_z) / denom;
        model.b = (g_sum_z * g_sum_rr - g_sum_r * g_sum_rz) / denom;
    }

    if (std::abs(model.a) > config_.max_slope) {
        model.a = (model.a > 0) ? config_.max_slope : -config_.max_slope;
    }

    model.valid = true;
}

void FastGroundSegmentation::interpolateInvalidModels() {
    // 对于无效的扇区，使用邻近有效扇区的模型
    for (int s = 0; s < config_.num_sectors; ++s) {
        if (sector_models_[s].valid) {
            continue;
        }

        const SectorGroundModel* neighbor = getNeighborModel(s);
        if (neighbor) {
            sector_models_[s] = *neighbor;
        } else {
            // 没有邻近有效模型，使用默认水平地面
            sector_models_[s].a = 0.0;
            sector_models_[s].b = -config_.sensor_height;
            sector_models_[s].valid = true;
        }
    }
}

const SectorGroundModel* FastGroundSegmentation::getNeighborModel(int sector_idx) const {
    // 搜索最近的有效邻近扇区（左右各搜索最多4个）
    for (int offset = 1; offset <= 4; ++offset) {
        // 检查左邻居
        int left = (sector_idx - offset + config_.num_sectors) % config_.num_sectors;
        if (sector_models_[left].valid) {
            return &sector_models_[left];
        }

        // 检查右邻居
        int right = (sector_idx + offset) % config_.num_sectors;
        if (sector_models_[right].valid) {
            return &sector_models_[right];
        }
    }

    return nullptr;
}

bool FastGroundSegmentation::isGroundPoint(const PointType& pt,
                                            int sector_idx,
                                            int bin_idx,
                                            double distance) const {
    const auto& model = sector_models_[sector_idx];

    // 使用多线段模型查询高度（如果有线段）
    double expected_z;
    if (!model.segments.empty()) {
        expected_z = model.getSegmentHeight(distance, bin_idx);
    } else {
        expected_z = model.valid ? model.getHeight(distance) : -config_.sensor_height;
    }

    double diff = pt.z - expected_z;

    // 获取自适应阈值
    double threshold = getAdaptiveThreshold(distance);

    // 地面点应该在模型附近
    // 上方容忍 threshold，下方容忍 threshold * ground_below_factor
    // factor=1.0 完全对称，factor=2.0 是旧行为（下方放宽2倍）
    // 1.5 折中：允许噪声导致的略低，但不会在 roll 时过度放宽单侧
    return diff < threshold && diff > -threshold * config_.ground_below_factor;
}

void FastGroundSegmentation::segment(const pcl::PointCloud<PointType>::Ptr& input,
                                      pcl::PointCloud<PointType>::Ptr& ground,
                                      pcl::PointCloud<PointType>::Ptr& non_ground) {
    if (!initialized_) {
        configure(FGSConfig());
    }

    // 确保输出点云已分配
    if (!ground) ground = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    if (!non_ground) non_ground = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

    ground->clear();
    non_ground->clear();

    if (!input || input->empty()) {
        return;
    }

    // 预分配输出空间
    ground->reserve(input->size() / 2);
    non_ground->reserve(input->size() / 2);

    // Step 1: 重置栅格
    resetPolarGrid();

    // Step 2: 填充极坐标栅格 (单遍扫描)
    fillPolarGrid(input);

    // Step 3: 拟合地面模型
    fitGroundModels();

    // Step 4: 分类点云 (单遍扫描)
    for (const auto& pt : input->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        int sector_idx, bin_idx;
        double distance;
        getGridIndex(pt, sector_idx, bin_idx, distance);

        // 超出范围的点视为非地面
        if (distance < config_.min_range || distance > config_.max_range) {
            non_ground->push_back(pt);
            continue;
        }

        if (isGroundPoint(pt, sector_idx, bin_idx, distance)) {
            ground->push_back(pt);
        } else {
            non_ground->push_back(pt);
        }
    }

    // 可选：精细化二次拟合 (Zermas 2017)
    if (config_.enable_refinement) {
        refinementPass(input);
        // 重新分类
        ground->clear();
        non_ground->clear();
        ground->reserve(input->size() / 2);
        non_ground->reserve(input->size() / 2);
        for (const auto& pt : input->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            int sector_idx, bin_idx;
            double distance;
            getGridIndex(pt, sector_idx, bin_idx, distance);
            if (distance < config_.min_range || distance > config_.max_range) {
                non_ground->push_back(pt);
                continue;
            }
            if (isGroundPoint(pt, sector_idx, bin_idx, distance)) {
                ground->push_back(pt);
            } else {
                non_ground->push_back(pt);
            }
        }
    }

    // 设置点云属性
    ground->width = ground->size();
    ground->height = 1;
    ground->is_dense = true;

    non_ground->width = non_ground->size();
    non_ground->height = 1;
    non_ground->is_dense = true;
}

void FastGroundSegmentation::segmentNonGround(const pcl::PointCloud<PointType>::Ptr& input,
                                               pcl::PointCloud<PointType>::Ptr& non_ground) {
    if (!initialized_) {
        configure(FGSConfig());
    }

    if (!non_ground) {
        non_ground = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }
    non_ground->clear();

    if (!input || input->empty()) {
        return;
    }

    non_ground->reserve(input->size() / 2);

    // Step 1: 重置栅格
    resetPolarGrid();

    // Step 2: 填充极坐标栅格
    fillPolarGrid(input);

    // Step 3: 拟合地面模型
    fitGroundModels();

    // Step 4: 仅输出非地面点
    for (const auto& pt : input->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        int sector_idx, bin_idx;
        double distance;
        getGridIndex(pt, sector_idx, bin_idx, distance);

        if (distance < config_.min_range || distance > config_.max_range) {
            non_ground->push_back(pt);
            continue;
        }

        if (!isGroundPoint(pt, sector_idx, bin_idx, distance)) {
            non_ground->push_back(pt);
        }
    }

    // 可选：精细化二次拟合 (Zermas 2017)
    if (config_.enable_refinement) {
        refinementPass(input);
        non_ground->clear();
        non_ground->reserve(input->size() / 2);
        for (const auto& pt : input->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            int sector_idx, bin_idx;
            double distance;
            getGridIndex(pt, sector_idx, bin_idx, distance);
            if (distance < config_.min_range || distance > config_.max_range) {
                non_ground->push_back(pt);
                continue;
            }
            if (!isGroundPoint(pt, sector_idx, bin_idx, distance)) {
                non_ground->push_back(pt);
            }
        }
    }

    non_ground->width = non_ground->size();
    non_ground->height = 1;
    non_ground->is_dense = true;
}

void FastGroundSegmentation::smoothSectorModels() {
    // 只对全局模型 a/b 做扇区间加权平均
    // 不对 segments 做跨扇区平滑（相邻扇区线段覆盖范围不同，按索引对齐会产生错误）
    // 权重：中心扇区 0.6，左右邻居各 0.2
    std::vector<double> smoothed_a(config_.num_sectors);
    std::vector<double> smoothed_b(config_.num_sectors);

    for (int s = 0; s < config_.num_sectors; ++s) {
        if (!sector_models_[s].valid) {
            smoothed_a[s] = sector_models_[s].a;
            smoothed_b[s] = sector_models_[s].b;
            continue;
        }

        int left = (s - 1 + config_.num_sectors) % config_.num_sectors;
        int right = (s + 1) % config_.num_sectors;

        bool has_left = sector_models_[left].valid;
        bool has_right = sector_models_[right].valid;

        if (!has_left && !has_right) {
            smoothed_a[s] = sector_models_[s].a;
            smoothed_b[s] = sector_models_[s].b;
            continue;
        }

        double w_center = 0.6;
        double w_side = 0.2;

        double sum_a = w_center * sector_models_[s].a;
        double sum_b = w_center * sector_models_[s].b;
        double w_total = w_center;

        if (has_left) {
            sum_a += w_side * sector_models_[left].a;
            sum_b += w_side * sector_models_[left].b;
            w_total += w_side;
        }
        if (has_right) {
            sum_a += w_side * sector_models_[right].a;
            sum_b += w_side * sector_models_[right].b;
            w_total += w_side;
        }

        smoothed_a[s] = sum_a / w_total;
        smoothed_b[s] = sum_b / w_total;
    }

    for (int s = 0; s < config_.num_sectors; ++s) {
        sector_models_[s].a = smoothed_a[s];
        sector_models_[s].b = smoothed_b[s];
    }
}

void FastGroundSegmentation::refinementPass(const pcl::PointCloud<PointType>::Ptr& input) {
    // 保存当前模型用于分类
    std::vector<SectorGroundModel> saved_models = sector_models_;

    // 重置栅格（但不需要模型，因为我们用 saved_models 分类）
    for (int s = 0; s < config_.num_sectors; ++s) {
        for (int b = 0; b < config_.num_bins; ++b) {
            polar_grid_[s][b].reset();
        }
    }

    for (const auto& pt : input->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        int sector_idx, bin_idx;
        double distance;
        getGridIndex(pt, sector_idx, bin_idx, distance);

        if (distance < config_.min_range || distance > config_.max_range) {
            continue;
        }

        // 使用保存的模型判断是否为地面点
        const auto& model = saved_models[sector_idx];
        double expected_z;
        if (!model.segments.empty()) {
            expected_z = model.getSegmentHeight(distance, bin_idx);
        } else {
            expected_z = model.valid ? model.getHeight(distance) : -config_.sensor_height;
        }
        double diff = pt.z - expected_z;
        double threshold = getAdaptiveThreshold(distance);

        if (diff < threshold && diff > -threshold * 2.0) {
            polar_grid_[sector_idx][bin_idx].addPoint(pt.z);
        }
    }

    // 重新拟合
    for (int s = 0; s < config_.num_sectors; ++s) {
        sector_models_[s].valid = false;
        sector_models_[s].segments.clear();
    }
    fitGroundModels();
}

void FastGroundSegmentation::temporalSmooth() {
    double base_alpha = config_.temporal_alpha;

    if (has_prev_models_ &&
        static_cast<int>(prev_sector_models_.size()) == config_.num_sectors) {

        // 自适应 alpha：计算当前帧与上一帧的全局模型差异
        // 差异大（S弯变向、过弯 roll 切换）→ 提高 alpha，更信任当前帧
        // 差异小（直线稳定行驶）→ 保持低 alpha，更平滑
        double frame_alpha = base_alpha;
        if (config_.enable_adaptive_alpha) {
            double total_diff = 0.0;
            int valid_count = 0;
            for (int s = 0; s < config_.num_sectors; ++s) {
                if (sector_models_[s].valid && prev_sector_models_[s].valid) {
                    // 加权差异：斜率变化影响更大（乘以典型距离 10m 转换为高度差）
                    double da = std::abs(sector_models_[s].a - prev_sector_models_[s].a) * 10.0;
                    double db = std::abs(sector_models_[s].b - prev_sector_models_[s].b);
                    total_diff += da + db;
                    valid_count++;
                }
            }
            if (valid_count > 0) {
                double avg_diff = total_diff / valid_count;
                if (avg_diff > config_.adaptive_alpha_threshold) {
                    // 线性插值：diff 从 threshold 到 threshold*5 时，alpha 从 base 到 max
                    double ratio = (avg_diff - config_.adaptive_alpha_threshold)
                                 / (config_.adaptive_alpha_threshold * 4.0);
                    ratio = std::min(1.0, std::max(0.0, ratio));
                    frame_alpha = base_alpha + ratio * (config_.adaptive_alpha_max - base_alpha);
                }
            }
        }

        for (int s = 0; s < config_.num_sectors; ++s) {
            auto& cur = sector_models_[s];
            const auto& prev = prev_sector_models_[s];

            if (cur.valid && prev.valid) {
                cur.a = frame_alpha * cur.a + (1.0 - frame_alpha) * prev.a;
                cur.b = frame_alpha * cur.b + (1.0 - frame_alpha) * prev.b;
            } else if (!cur.valid && prev.valid) {
                cur.a = prev.a;
                cur.b = prev.b;
                cur.valid = true;
            }
        }
    }

    // 保存当前帧的全局模型供下一帧使用（只保存 a/b/valid）
    if (prev_sector_models_.size() != sector_models_.size()) {
        prev_sector_models_.resize(sector_models_.size());
    }
    for (int s = 0; s < config_.num_sectors; ++s) {
        prev_sector_models_[s].a = sector_models_[s].a;
        prev_sector_models_[s].b = sector_models_[s].b;
        prev_sector_models_[s].valid = sector_models_[s].valid;
        prev_sector_models_[s].segments.clear();
    }
    has_prev_models_ = true;
}

}  // namespace perception
