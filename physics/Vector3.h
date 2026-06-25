#pragma once
#include <cmath>
#include <string>
#include <stdexcept>

namespace CarPhysics {

struct Vector3 {
    double x{0.0}, y{0.0}, z{0.0};

    Vector3() = default;
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Arithmetic
    Vector3 operator+(const Vector3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3 operator*(double s)         const { return {x*s,   y*s,   z*s  }; }
    Vector3 operator/(double s)         const {
        if (std::abs(s) < 1e-12) throw std::domain_error("Vector3 division by zero");
        return {x/s, y/s, z/s};
    }
    Vector3 operator-() const { return {-x, -y, -z}; }

    Vector3& operator+=(const Vector3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vector3& operator*=(double s)         { x*=s;   y*=s;   z*=s;   return *this; }

    bool operator==(const Vector3& o) const {
        return std::abs(x-o.x)<1e-9 && std::abs(y-o.y)<1e-9 && std::abs(z-o.z)<1e-9;
    }
    bool operator!=(const Vector3& o) const { return !(*this == o); }

    // Products
    double  dot(const Vector3& o)   const { return x*o.x + y*o.y + z*o.z; }
    Vector3 cross(const Vector3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }

    // Magnitude
    double lengthSq()     const { return x*x + y*y + z*z; }
    double length()       const { return std::sqrt(lengthSq()); }
    Vector3 normalized()  const {
        double len = length();
        if (len < 1e-12) return {0,0,0};
        return *this / len;
    }

    // Utilities
    Vector3 projectOnto(const Vector3& axis) const {
        double d = axis.lengthSq();
        if (d < 1e-12) return {};
        return axis * (dot(axis) / d);
    }

    static Vector3 lerp(const Vector3& a, const Vector3& b, double t) {
        return a + (b - a) * t;
    }

    std::string toString() const;

    static const Vector3 ZERO;
    static const Vector3 UP;
    static const Vector3 FORWARD;
    static const Vector3 RIGHT;
};

inline Vector3 operator*(double s, const Vector3& v) { return v * s; }

} 