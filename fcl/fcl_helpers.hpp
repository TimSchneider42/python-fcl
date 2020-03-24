#pragma once
#ifndef PYTHON_FCL_HELPERS_HPP
#define PYTHON_FCL_HELPERS_HPP

#include <fcl/fcl.h>

namespace python_fcl_helpers {
    inline fcl::Matrix3d make_matrix_3d(
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double zx, double zy, double zz)
    {
        fcl::Matrix3d m;
        m << xx, xy, xz,
             yx, yy, yz,
             zx, zy, zz;
        return m;
    }

    inline fcl::Transform3d mat3_to_transform(const fcl::Matrix3d& r) {
        fcl::Transform3d m;
        m.setIdentity();
        m.linear() = r;
        return m;
    }
    
    inline fcl::Transform3d vec3_to_transform(const fcl::Vector3d& t) {
        fcl::Transform3d m;
        m.setIdentity();
        m.translation() = t;
        return m;
    }
    
    inline fcl::Transform3d mat3_vec3_to_transform(const fcl::Matrix3d& r, const fcl::Vector3d& t) {
        fcl::Transform3d m;
        m.setIdentity();
        m.linear() = r;
        m.translation() = t;
        return m;
    }

    inline void set_transform_rotation(fcl::Transform3d* m, const fcl::Matrix3d& r) {
        m->linear() = r;
    }

    inline void set_transform_translation(fcl::Transform3d* m, const fcl::Vector3d& t) {
        m->translation() = t;
    }
}

#endif // PYTHON_FCL_HELPERS_HPP
