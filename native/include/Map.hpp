// Map definition.

#ifndef MAP_HPP_
#define MAP_HPP_

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>

// TODO. recover map concept.

// Coordinate used
typedef int32_t coordinate_t;

// A point from boost geometry to easily reuse algorithms.
typedef boost::geometry::model::d2::point_xy<coordinate_t> point_t;

// A bounding box easy to reuse.
typedef boost::geometry::model::box<point_t> bbox_t;

// UTILS
inline std::ostream & operator << (std::ostream & out, bbox_t const & box)
{
	out << '(' << box.min_corner().x() << ", " << box.min_corner().y() << ", "
			<< box.max_corner().x() << ", " << box.max_corner().y() << ')';
	return out;
}

#endif  // MAP_HPP_
