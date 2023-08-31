#include <iostream>
#include "denoising.hpp"
#include <pcl/filters/extract_indices.h>
#include <stack>

int main()
{
    std::vector<std::string> dd = {"4", "13", "5", "/", "+"};
    evalRPN(dd);

    using PointType = pcl::PointXYZ;
    using CloudType = pcl::PointCloud<PointType>;

    CloudType::Ptr cloud(new CloudType);
    cloud->is_dense = false;
    PointType p;
    for (uint32_t i = 0; i < 5; i++)
    {
        p.x = p.y = p.z = static_cast<float>(i);
        cloud->push_back(p);
    }

    //    std::cout << "Cloud "

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.push_back(0);
    indices->indices.push_back(2);

    pcl::ExtractIndices<PointType> extractor;

    extractor.setInputCloud(cloud);
    extractor.setIndices(indices);
    extractor.setNegative(false);
    extractor.filter(*cloud);

    std::vector<int> c{0, 1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> i{0, 2};
    auto elem = c.at(0);
    std::remove_if(
        c.begin(), c.end(),
        [idx = 0, &i](auto e) mutable
        {
            auto const res = std::any_of(
                i.begin(), i.end(),
                [&idx, &e](auto const elem)
                {
                    if (elem == e) return true;
                });
            //            for (auto ii : i)
            //            {
            //                if (ii != idx) return true;
            //            }
            idx++;
            return res;
        });

    return 0;
}
