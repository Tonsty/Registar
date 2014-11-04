#include <CGAL/intersections.h>
#include <CGAL/Simple_cartesian.h>

#include <boost/variant.hpp>
#include <boost/variant/apply_visitor.hpp>

using namespace CGAL;

template <typename R>
struct Intersection_visitor {
  typedef void result_type;
  void operator()(const Point_2<R>& p) const
  {
    // handle point
  }
  void operator()(const Segment_2<R>& s) const
  {
    // handle segment
  }
};

template <typename R>
void foo (const Segment_2<R>& seg, const Line_2<R>& lin)
{
  // with C++11 support
  auto result = intersection(seg, lin);
  // without C++11
  // cpp11::result_of<R::Intersect_2(Segment_2<R>, Line_2<R>)>::type
  //   result = intersection(seg, lin);
  if (result) { boost::apply_visitor(Intersection_visitor<R>(), *result); }
  else {
    // no intersection
  }
  // alternatively:
  if (result) {
    if (const Segment_2<R>* s = boost::get<Segment_2<R> >(&*result)) {
      // handle segment
    } else {
      const Point_2<R>* p = boost::get<Point_2<R> >(&*result);
      // handle point
    }
  }
}

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2                     Point;
typedef K::Segment_2                   Segment;
typedef K::Line_2                      Line;


int main(int argc, char **argv)
{
	Segment seg;

	Line line;
	return 0;
}