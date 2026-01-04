#include <utility.h>
#include <set>
// 地面分割处理

bool point_cmp(PointType a, PointType b) // PointType表明XYZ点云库的一个点坐标
{
    return a.z < b.z;
}

// 地面分割采用RANSAC算法
void lidar_cluster::RANSAC(const pcl::PointCloud<PointType>::Ptr &cloud_filtered,
                           pcl::PointCloud<PointType>::Ptr &cloud_filtered_out, int maxIterations, float thre)
{

    pcl::SACSegmentation<PointType> seg; // 点云库中实现RANSAC算法的对象
    // pcl::PointCloud<PointType>::Ptr  cloud_f (new pcl::PointCloud<PointType>);//提取除开平面点云的剩余点云集合
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                         // 点云索引   用于存储与平面模型拟合程度较高的点的索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);          // 存储拟合的平面模型的系数
    pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>()); // 提取平面的点云集

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 使用平面模型进行分割
    seg.setMethodType(pcl::SAC_RANSAC);    // RANSAC
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(thre);

    // 从剩余的点云中分割最大平面组成部分
    seg.setInputCloud(cloud_filtered);    // 传入点云数据
    seg.segment(*inliers, *coefficients); // TODO RANSC//将与平面模型拟合程度较高的点的索引保存到inliers对象中，将平面模型的系数保存到coefficients对象中

    // Extract the planar inliers from the input cloud  从输入云中提取平面内层
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud_filtered); // 从这个里面提取点集
    extract.setIndices(inliers);           // 要提取点集索引  这里给出了与地面点集重合度较高的索引
    extract.setNegative(false);            // setNegative表示，删除自己设置的Indices

    // Write the planar inliers to disk    将平面内存写入磁盘             表示平面组件的PointCloud：   输出了平面点云有多少
    extract.filter(*cloud_plane); // 设置完之后执行filter才能实现
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest    拆下平面内衬，取出其余部分
    extract.setNegative(true); // true代表保留剩余的点，方便下次循环使用。  取的是除了 `inliers` 中指定的点以外的所有点。这样做的目的是保留剩余的点，方便下次循环使用
    extract.filter(*cloud_filtered_out);

    // cloud_filtered = cloud_f;
}

// ground_segmentation     地面分段
void lidar_cluster::ground_segmentation(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                        pcl::PointCloud<PointType>::Ptr &g_not_ground_pc)
{

    g_not_ground_pc->points.clear();
    // 1.Msg to pointcloud   消息转换可使用点云库
    pcl::PointCloud<PointType> laserCloudIn, laserCloudIn_org;
    laserCloudIn = laserCloudIn_org = *in_pc; // 消息赋值
    // cloud = *cloud_Ptr;
    // cloud_Ptr = cloud.makeShared();
    //  For mark ground points and hold all points
    PointType point;
    g_all_pc = laserCloudIn.makeShared(); // 动态管理避免内存泄漏
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    //  2.Sort on Z-axis value.  按Z轴值排序。
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp); // 从小往大排
    // 3.Error point removal   删除错误点
    // As there are some error mirror reflection under the ground,            存在错误的误差镜面反射点云
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    // 由于地面下存在一些误差反射镜，
    // 此处不考虑2*sensor_height下的点
    // 根据高度对点进行排序，此处默认使用z轴
    pcl::PointCloud<PointType>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1 * sensor_height_) // 0.25
        {
            it++;
        }
        else
        {
            break;
        }
    }
    // remove below sensor_height_* - 1.5 points   拆下传感器下方的重量_*-1.5分
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    // 4. Extract init ground seeds. 提取初始地面种子
    extract_initial_seeds_(laserCloudIn); // 这个可能压根都没用上  数据都被清空了  不对  estimate_plane_用上了
    g_ground_pc = g_seeds_pc;             // 获取的地面种子存放到地面里面
    // 5. Ground plane fitter mainloop
    // 在这段代码中，是地面平面拟合的主循环。通过多次迭代来估计地面平面模型，
    // 并将点云数据根据与地面平面的距离进行分类，存储在`g_ground_pc`和`g_not_ground_pc`中
    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        // pointcloud to matrix       方便后续处理
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model             ground plane model
        // 首先，将点云数据`points`与平面的法线`normal_`进行点乘运算，
        // 得到一个VectorXf对象`result`。这个运算相当于将每个点的坐标与平面的法线进行内积运算，得到每个点到平面的距离。
        VectorXf result = points * normal_; // 平面的法线
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_) // 越大 就把越多的数据放进地面里面   所以分割不均可以考虑把数值放大
            {
                // g_all_pc->points[r].label = 1u; // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                // g_all_pc->points[r].label = 0u; // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }
    g_all_pc->clear();
}

// extract_initial_seeds_  extract_initial_seds（提取初始设置）_
//`extract_initial_seeds_`函数用于从排序后的点云数据`p_sorted`中提取初始的地面种子点。
void lidar_cluster::extract_initial_seeds_(const pcl::PointCloud<PointType> &p_sorted)
{
    // LPR is the mean of low point representative             LPR是代表低点的平均值
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.                 计算平均高度值。
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0            平均高度
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_            迭代掉高度小于这个的点  越大去除的点越多  可能飞地面点也去除了
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points       最后放进去的都是地面种子点
}

// estimate_plane_  估计_平面_           用于估计平面的方程参数
// 平面的法线向量可以用于描述平面的方向、倾斜程度等信息。在点云处理中，
// 通过估计平面的法线向量，可以确定地面的朝向和倾斜程度，从而实现地面分割、障碍物检测等任务。
// 而通过估计距离阈值，可以确定离地面的距离范围，从而将地面点和非地面点分离开来。
void lidar_cluster::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    // 在单程中创建协变矩阵。
    // TODO:比较效率。
    Eigen::Matrix3f cov;     // 协方差矩阵
    Eigen::Vector4f pc_mean; // 4维的均值向量pc_mean
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    ////奇异值分解：SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    // 使用最小奇异向量作为法线             通过取最小奇异向量作为平面的法线，可以确定平面的朝向。
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    // 平均地面种子值
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    // 根据法线。T*[x，y，z]=-d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0); // 可以得到平面方程中的常数项D。这样就可以将平面方程完整地表示出来。
    // set distance threhold to `th_dist - d`          Ax + By + Cz + D = 0
    // 将距离阈值设置为`th_dist-d`
    // th_dist_这个值越大 接受偏离地面程度越高  所以越大去除地面越多。。
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
    // 返回方程式参数
}

// 1. 创建一个3x3的协方差矩阵cov和一个4维的均值向量pc_mean。
// 2. 使用pcl库中的computeMeanAndCovarianceMatrix函数计算输入点云g_ground_pc的均值和协方差矩阵。
// 3. 使用奇异值分解（SVD）对协方差矩阵cov进行分解，计算其特征向量和特征值。
// 4. 使用最小奇异向量作为平面的法线normal_。
// 5. 提取pc_mean的前3个元素作为地面种子点的均值seeds_mean。
// 6. 根据平面方程的定义，将法线normal_与种子点均值seeds_mean的内积取负值作为平面方程的距离d_。
// 7. 将距离阈值th_dist_减去距离d_得到距离阈值th_dist_d_。
// 8. 返回平面方程的法线和距离阈值。
// 估计平面的法线和距离阈值的目的是将点云数据分割为地面和非地面两个部分。通过估计平面的法线，可以确定地面的朝向和倾斜程度。
// 而通过估计距离阈值，可以确定离地面的距离范围，从而将地面点和非地面点分离开来。
// 在激光雷达数据处理中，地面通常被认为是相对平坦的区域，而非地面则包括了障碍物、
// 建筑物等。通过估计平面的法线和距离阈值，可以将地面点从点云数据中提取出来，从而更好地进行地面分割、障碍物检测等任务。

// 平面的法线是指垂直于平面的向量。在三维空间中，一个平面可以由一个点和与该平面垂直的法线向量唯一确定。法线向量的方向与平面的朝向相同，长度为1。

// 对于平面的方程Ax + By + Cz + D = 0，其中A、B、C是平面的法向量的分量，D是平面方程的常数项。法线向量的分量就是(A, B, C)。

// 平面的法线向量可以用于描述平面的方向、倾斜程度等信息。在点云处理中，通过估计平面的法线向量，可以确定地面的朝向和倾斜程度，从而实现地面分割、障碍物检测等任务。
// 奇异值分解（Singular Value Decomposition，SVD）是一种矩阵分解的方法，可以将一个矩阵分解为三个矩阵的乘积。对于协方差矩阵cov，SVD可以得到其特征向量和特征值。

// SVD的原理是将一个矩阵分解为三个矩阵的乘积：A = U * S * V^T，其中A是待分解的矩阵，U和V是正交矩阵，S是对角矩阵。在S中，对角线上的元素称为奇异值，它们是按照从大到小的顺序排列的。

// 对于协方差矩阵cov，它是一个对称矩阵，可以进行SVD分解。通过SVD分解，可以得到U和V两个正交矩阵，它们的列向量是cov的特征向量。而S的对角线上的元素是cov的特征值的平方根。

// 在代码中，通过使用JacobiSVD类进行SVD分解，使用cov作为输入矩阵，获得其特征向量和特征值。通过取最小奇异向量作为平面的法线，可以确定平面的朝向。

// 平面方程的定义是Ax + By + Cz + D = 0，其中A、B、C是平面的法向量的分量，D是平面方程的常数项。法线normal_是平面的法向量，种子点均值seeds_mean是平面上的一个点。

// 将法线normal_与种子点均值seeds_mean的内积取负值作为平面方程的距离d_的意义是确定平面方程的常数项D。通过计算法线向量与一个平面上的点的内积，可以得到平面方程中的常数项D。这样就可以将平面方程完整地表示出来。

// 通过确定平面方程的常数项D，可以将点云数据中的点带入平面方程中进行判断，判断点是否在该平面上。对于一个点(x, y, z)，如果满足Ax + By + Cz + D = 0，那么该点就在平面上。而如果不满足该方程，那么该点就不在平面上。

// 在代码中，通过计算法线normal_与种子点均值seeds_mean的内积，并取负值，得到平面方程的常数项D，即距离d_。这样就可以用平面方程来判断点云数据中的点是否属于该平面。

// 这段代码是将点云数据存储在一个Eigen库的MatrixXf对象中。

// 首先，创建一个MatrixXf对象`points`，其大小为(laserCloudIn_org.points.size(), 3)，即点云数据的总点数乘以3（每个点有三个坐标）。

// 然后，使用一个循环遍历点云数据`laserCloudIn_org.points`中的每个点。对于每个点p，使用`points.row(j++) << p.x, p.y, p.z`将其坐标值（x、y、z）存储在MatrixXf对象的第j行中。

// 这段代码的目的是将点云数据从`laserCloudIn_org.points`中提取出来，并按照每个点的坐标值存储在MatrixXf对象中，以便后续对点云数据进行处理和计算。
// 这段代码用于对点云数据进行阈值过滤，筛选出符合地面模型的点。

// 首先，将点云数据`points`与平面的法线`normal_`进行点乘运算，得到一个VectorXf对象`result`。这个运算相当于将每个点的坐标与平面的法线进行内积运算，得到每个点到平面的距离。

// 然后，根据设定的阈值，对`result`进行过滤，将距离小于阈值的点保留下来，即将符合地面模型的点筛选出来。

// 通过这个阈值过滤，可以将点云数据中的地面点与非地面点分离开来，进一步进行地面分割和障碍物检测等任务。
