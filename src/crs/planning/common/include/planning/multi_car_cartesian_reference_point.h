#ifndef SRC_CRS_PLANNING_COMMON_INCLUDE_PLANNING_MULTI_CAR_CARTESIAN_REFERENCE_POINT
#define SRC_CRS_PLANNING_COMMON_INCLUDE_PLANNING_MULTI_CAR_CARTESIAN_REFERENCE_POINT

#include <vector>
#include <string>
#include <planning/cartesian_reference_point.h>
namespace crs_planning
{
struct multi_car_cartesian_reference_point
{
  std::vector<std::string> namespaces;
  std::vector<cartesian_reference_point> points;
};
}  // namespace crs_planning
#endif /* SRC_CRS_PLANNING_COMMON_INCLUDE_PLANNING_SINGLE_CAR_CARTESIAN_REFERENCE_POINT */
