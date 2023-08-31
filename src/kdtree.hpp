#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace kt
{

    template<typename Point, int Idx>
    struct CompareByDimension
    {
        bool operator()(Point const& p1, Point const& p2) const
        {
            if constexpr (Idx == 1)
            {
                return p1.x() < p2.x();
            }
            else if (Idx == 2)
            {
                return p1.y() < p2.y();
            }
            else
            {
                return p1.z() < p2.z();
            }
        }
    };

    template<typename Type>
    struct KDNode;

    template<typename Point>
    class KDTree
    {
    public:
        using PointCloud = std::vector<Point>;

        KDTree() : tree_{nullptr} {}
        void setInputCloud(PointCloud const& input)
        {
            PointCloud pointsCopy = input;
            tree_ = buildTree(pointsCopy, 0);
        }

        size_t radiusSearch(
            const Point& point,
            int index,
            double radius,
            std::vector<int>& kIndices,
            std::vector<float>& kSqrDistances,
            unsigned int maxNN = 0) const
        {
            return countNeighbors(point, radius, 0);
        }

    private:
        using KDNodePtr = std::unique_ptr<KDNode<typename Point::Type>>;

        KDNodePtr tree_;

        KDNodePtr buildKDTree(PointCloud& pointCloud, uint8_t depth)
        {
            if (pointCloud.empty()) return nullptr;
            auto axis = depth % dims_;

            std::sort(
                pointCloud.begin(), pointCloud.end(),
                [axis](auto const& p1, auto const p2)
                {
                    if (axis == 1) return p1.x() < p2.x();
                    if (axis == 2) return p1.y() < p2.y();
                    return p1.z() < p2.z();
                });

            int medianIndex = pointCloud.size() / 2;
            auto newNode = KDNodePtr(pointCloud[medianIndex], axis);

            newNode->left = buildTree(PointCloud(pointCloud.begin(), pointCloud.begin() + medianIndex), depth + 1);
            newNode->right = buildTree(PointCloud(pointCloud.begin() + medianIndex + 1, pointCloud.end()), depth + 1);

            return newNode;
        }

        int countNeighbors(const Point& point, double radius, int depth)
        {
            if (tree_ == nullptr)
            {
                return 0;
            }

            int axis = depth % 3; // Assuming 3D points

            double distance = calculateDistance(tree_->point, point);
            int count = (distance <= radius) ? 1 : 0;

            if (axis == 0   ? point.x() < tree_->point.x()
                : axis == 1 ? point.y() < tree_->point.y()
                            : point.z() < tree_->point.z())
            {
                count += countNeighbors(tree_->left, point, radius, depth + 1);
                if (distance > radius)
                {
                    count += countNeighbors(tree_->right, point, radius, depth + 1);
                }
            }
            else
            {
                count += countNeighbors(tree_->right, point, radius, depth + 1);
                if (distance > radius)
                {
                    count += countNeighbors(tree_->left, point, radius, depth + 1);
                }
            }

            return count;
        }

        double calculateDistance(Point const& p1, Point const& p2)
        {
            auto const dx = p1.x() - p2.x();
            auto const dy = p1.y() - p2.y();
            auto const dz = p1.z() - p2.z();
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        static constexpr uint8_t dims_{3};

        template<typename Type>
        struct KDNode
        {
            Point point;
            uint8_t axis;
            std::unique_ptr<KDNode> left{nullptr};
            std::unique_ptr<KDNode> right{nullptr};

            KDNode(Point&& p, uint8_t splitAxis) : point(std::move(p)), axis(splitAxis), left(nullptr), right(nullptr)
            {
            }
        };
    };
} // namespace kt

#endif // KDTREE_HPP
