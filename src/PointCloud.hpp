#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

namespace pc
{
    template<typename T>
    struct Point
    {
    public:
        using Type = T;

        Point() = default;
        explicit Point(T x, T y, T z) : x_{x}, y_{y}, z_{z} {}

        auto& x()
        {
            return x_;
        }
        auto& y()
        {
            return x_;
        }
        auto& z()
        {
            return x_;
        }

    private:
        T x_{};
        T y_{};
        T z_{};
    };
} // namespace pc

#endif // POINTCLOUD_HPP
