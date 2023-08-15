/** @file
@brief Contains classes for denoising lidar point clouds in different situations
like dust, snow, rain
@author Saman Mahmoodi
*/

#ifndef LIDAR_DENOISING_H
#define LIDAR_DENOISING_H

#include <pcl/kdtree/kdtree_flann.h>

namespace denoising
{
    template<typename PointCloudType>
    using KdTreePtr = typename pcl::KdTreeFLANN<PointCloudType>::Ptr;

    template<typename PointCloudType, typename PointCloud>
    class DRORFilter
    {
    public:
        DRORFilter() = default;
        explicit DRORFilter(double angularResultion) : angularResultion_{angularResultion} {}
        ~DRORFilter() = default;

        void operator()(typename PointCloud::Ptr& inputCloud, PointCloud& filteredCloud)
        {
            filteredCloud.clear();
            KdTreePtr<PointCloudType> kdTree(new pcl::KdTreeFLANN<PointCloudType>());

            kdTree->setInputCloud(inputCloud);

            std::for_each(
                inputCloud.begin(), inputCloud->end(),
                [this, &filteredCloud, &kdTree](auto const& point)
                {
                    auto const range = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
                    auto searchRadius = range;

                    if (searchRadius < minSearchRadius_)
                    {
                        searchRadius = minSearchRadius_;
                    }
                    else
                    {
                        searchRadius = radiusMultiplier_ * 2 * range * sin(angularResultion_);
                    }

                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;

                    auto const neighbors =
                        kdTree->radiusSearch(point, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

                    if (neighbors >= mindNeighbors_)
                    {
                        filteredCloud.push_back(point);
                    }
                });
        }

    private:
        double radiusMultiplier_{3};
        double angularResultion_{0.04 * M_PI / 180};
        double minSearchRadius_{0.04};
        uint64_t mindNeighbors_{3};
    };

    template<typename PointCloudType, typename PointCloud>
    class LIORFilter
    {
    public:
        LIORFilter() = default;

        void operator()(typename PointCloud::Ptr& inputCloud, PointCloud& filteredCloud)
        {
            filteredCloud.clear();
            KdTreePtr<PointCloudType> kdTree(new pcl::KdTreeFLANN<PointCloudType>());
            kdTree->setInputCloud(inputCloud);

            std::for_each(
                inputCloud->begin(), inputCloud->end(),
                [this, &filteredCloud, &kdTree](auto const& point)
                {
                    if (point.intensity < minIntensity_)
                    {
                        auto const range = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
                        auto searchRadius = range;

                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;

                        auto const neighbors =
                            kdTree->radiusSearch(point, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                        if (neighbors < mindNeighbors_)
                        {
                            filteredCloud.push_back(point);
                        }
                    }
                });
        }

    private:
        int minIntensity_{8};
        uint64_t mindNeighbors_{3};
    };

    template<typename PointCloudType, typename PointCloud>
    class LIDRORFilter
    {
    public:
        LIDRORFilter() = default;

        explicit LIDRORFilter(double minSearchRadius, double radiusMultiplier, int minNeighbors, float minIntensity)
            : minSearchRadius_{minSearchRadius},
              angularResultion_{minSearchRadius * M_PI / 180},
              minNeighbors_{minNeighbors},
              radiusMultiplier_{radiusMultiplier},
              minIntensity_{minIntensity}
        {
        }

        ~LIDRORFilter() = default;

        void operator()(typename PointCloud::Ptr& inputCloud, PointCloud& filteredCloud, PointCloud& noisyPointCloud)
        {
            filteredCloud.clear();
            KdTreePtr<PointCloudType> kdTree(new pcl::KdTreeFLANN<PointCloudType>());
            kdTree->setInputCloud(inputCloud);
            std::for_each(
                inputCloud->begin(), inputCloud->end(),
                [this, &filteredCloud, &kdTree, &noisyPointCloud](auto const& point)
                {
                    if (point.intensity > minIntensity_)
                    {
                        filteredCloud.push_back(point);
                    }
                    else
                    {
                        auto const range = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
                        auto searchRadius = range;
                        if (searchRadius < minSearchRadius_)
                        {
                            searchRadius = minSearchRadius_;
                        }
                        else
                        {
                            searchRadius = radiusMultiplier_ * 2 * range * sin(angularResultion_);
                        }

                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;

                        auto const neighbors =
                            kdTree->radiusSearch(point, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

                        if (neighbors < minNeighbors_)
                        {
                            noisyPointCloud.push_back(point);
                        }
                        else
                        {
                            filteredCloud.push_back(point);
                        }
                    }
                });
        }

    private:
        double radiusMultiplier_{0.011};
        double angularResultion_{0.25 * M_PI / 180};
        double minSearchRadius_{0.25};
        int minNeighbors_{5};
        float minIntensity_{100.0};
    };

} // namespace denoising

#endif // LIDAR_DENOISING_H
