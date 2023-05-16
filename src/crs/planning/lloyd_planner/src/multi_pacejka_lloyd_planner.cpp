#define JC_VORONOI_IMPLEMENTATION
#define JCV_REAL_TYPE double
#define JCV_FABS fabs
#define JCV_ATAN2 atan2
#define JCV_CEIL ceil
#define JCV_FLOOR floor
#define JCV_FLT_MAX 1.7976931348623157E+308

#include "lloyd_planner/multi_pacejka_lloyd_planner.h"
#include <map>
#include <string>
#include <iostream>
namespace crs_planning
{
extern "C" {
void jcv_diagram_generate(int num_points, const jcv_point* points, const jcv_rect* rect, const jcv_clipper* clipper,
                          jcv_diagram* d);
extern const jcv_site* jcv_diagram_get_sites(const jcv_diagram* diagram);
extern void jcv_diagram_free(jcv_diagram* diagram);
}

template <typename T>
std::vector<T> MultiPacejkaLloydPlanner::arange(T start, T stop, T step)
{
  std::vector<T> values;
  for (T value = start; value < stop; value += step)
    values.push_back(value);
  return values;
}

std::vector<multi_car_cartesian_reference_point> MultiPacejkaLloydPlanner::getPlannedTrajectory(
    const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state, double timestamp)
{
  num_agents_ = state.size();
  std::cout << num_agents_;
  computeVoronoiDiagram(state);
  centroids_ = computeCentroids();
  check_change_in_centroid(0.2);
  std::vector<multi_car_cartesian_reference_point> pts;
  multi_car_cartesian_reference_point reference_pts;
  int i = 0;
  for (const auto& ns_and_state : state)
  {
    cartesian_reference_point ref_pt;
    ref_pt.x = centroids_[i].x;  // + ns_and_state.second.pos_x;
    ref_pt.y = centroids_[i].y;  // + ns_and_state.second.pos_y;
    reference_pts.points.push_back(ref_pt);
    reference_pts.namespaces.push_back(ns_and_state.first);
    i++;
  }
  pts.push_back(reference_pts);

  // update last previous reference
  previous_reference_ = centroids_;

  // Reset reached information
  car_to_reach_state_.clear();

  for (const auto car_name_to_state : state)
  {
    car_to_reach_state_[car_name_to_state.first] = false;  // Set goal reached to  false
  }

  return pts;
};

bool MultiPacejkaLloydPlanner::goalReached(
    const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> states,
    const std::vector<multi_car_cartesian_reference_point>& trajectory)
{
  bool found_not_reached = false;

  for (const auto& entry : states)
  {
    const auto& car_reached = car_to_reach_state_.find(entry.first);

    if (car_reached == car_to_reach_state_.end())
    {
      car_to_reach_state_[entry.first] = true;
    }
    else if (!car_reached->second)
    {
      auto last_entry = trajectory.at(trajectory.size() - 1);
      int car_idx = -1;
      for (int i = 0; i < last_entry.namespaces.size(); i++)
      {
        if (last_entry.namespaces[i] == car_reached->first)
        {
          car_idx = i;
          // Found car. Check target reached.
          break;
        }
      }

      if (car_idx == -1)
      {
        std::cout << "ERROR DID NOT FIND CAR FOR NAMESPACE" << car_reached->first << std::endl;
        continue;
      }
      else
      {
        auto last_pt = last_entry.points[car_idx];
        auto state = entry.second;

        car_reached->second = Eigen::Vector2d(state.pos_x - last_pt.x, state.pos_y - last_pt.y).norm() < 0.3;  // 30cm
        if (!car_reached->second)  // did not reach goal
          found_not_reached = true;
      }
    }
  }

  // Debugging
  for (const auto& entry : car_to_reach_state_)
  {
    std::cout << "[" << entry.first << "] - Reached Goal: " << entry.second << std::endl;
  }
  std::cout << "-----" << std::endl;

  return !found_not_reached;
}

double MultiPacejkaLloydPlanner::sensory_function(cartesian_reference_point p)
{
  double res = 5.0 * exp(-((p.x - 1.4) * (p.x - 1.4) / 1.5 + (p.y - 1.7) * (p.y - 1.7) / 1.5));

  return res;
}

std::vector<cartesian_reference_point>
MultiPacejkaLloydPlanner::createGrid(std::vector<cartesian_reference_point>& vor_vertices, int num_samples)
{
  auto mmx = std::minmax_element(
      vor_vertices.begin(), vor_vertices.end(),
      [](const cartesian_reference_point& a, const cartesian_reference_point& b) { return a.x < b.x; });
  auto mmy = std::minmax_element(
      vor_vertices.begin(), vor_vertices.end(),
      [](const cartesian_reference_point& a, const cartesian_reference_point& b) { return a.y < b.y; });
  double x_min = mmx.first->x;
  double x_max = mmx.second->x;
  double y_min = mmy.first->y;
  double y_max = mmy.second->y;

  double grid_area = (x_max - x_min) * (y_max - y_min);
  double num_boxes = (num_samples - 2.0) / 2.0;
  double grid_density = sqrt(grid_area / (num_boxes * 2.0));

  std::vector<double> x_range = arange(x_min, x_max, grid_density);
  std::vector<double> y_range = arange(y_min, y_max, grid_density);

  std::vector<cartesian_reference_point> grid;
  grid.reserve(x_range.size() * y_range.size());

  for (int i = 0; i < x_range.size(); i++)
  {
    for (int j = 0; j < y_range.size(); j++)
    {
      cartesian_reference_point p;
      p.x = x_range[i];
      p.y = y_range[j];
      grid.push_back(p);
    }
  }

  return grid;
}

bool MultiPacejkaLloydPlanner::pnpoly(std::vector<cartesian_reference_point>& vertices, cartesian_reference_point p)
{
  bool inside = false;

  for (int i = 0; i < vertices.size(); i++)
  {
    // i is the index of the first vertex, j is the next one
    int j = (i + 1) % vertices.size();

    // the vertices of the edge we are checking
    double xp0 = vertices[i].x;
    double yp0 = vertices[i].y;
    double xp1 = vertices[j].x;
    double yp1 = vertices[j].y;

    // check whether edge intersects a line from (-inf, y) to (x, y)

    // first check if line crosses the horizontal line at y in either direction
    if ((yp0 <= p.y) && (yp1 > p.y) || (yp1 <= p.y) && (yp0 > p.y))
    {
      // if so, get the point where it crosses that line (solve linear system)
      double cross = (xp1 - xp0) * (p.y - yp0) / (yp1 - yp0) + xp0;

      // check if it crosses to the left of our test point
      if (cross < p.x)
        inside = !inside;
    }
  }
  return inside;
}

cartesian_reference_point MultiPacejkaLloydPlanner::pointCenterOfMass(std::vector<cartesian_reference_point>& points,
                                                                      std::vector<double>& values, double mass)
{
  double x_sum = 0;
  double y_sum = 0;
  for (int i = 0; i < points.size(); i++)
  {
    x_sum += points[i].x * values[i];
    y_sum += points[i].y * values[i];
  }
  cartesian_reference_point com;

  com.x = x_sum / mass;
  com.y = y_sum / mass;

  return com;
}

std::vector<cartesian_reference_point> MultiPacejkaLloydPlanner::computeCentroids()
{
  std::vector<cartesian_reference_point> centroids(num_agents_);

  for (int i = 0; i < num_agents_; i++)
  {
    // create grid of vertices in bounding rectangle of polygon
    std::vector<cartesian_reference_point> vor_vertices = vor_cell_vertices_[i];
    std::vector<cartesian_reference_point> grid = createGrid(vor_vertices, 10000);  // 10000
    std::vector<cartesian_reference_point> points_in_polygon;

    // pick out which grid points are inside polygon
    for (auto p : grid)
    {
      bool inside = pnpoly(vor_vertices, p);
      if (inside)
      {
        points_in_polygon.push_back(p);
      }
    }

    // apply sensory function to points
    std::vector<double> sensory_values;

    std::transform(points_in_polygon.begin(), points_in_polygon.end(), std::back_inserter(sensory_values),
                   MultiPacejkaLloydPlanner::sensory_function);

    // sum up sensory values to get total mass
    double mass = std::accumulate(sensory_values.begin(), sensory_values.end(), 0.0);

    // multiply sensory values by x, y coordinates to obtain CoM
    cartesian_reference_point center_of_mass = pointCenterOfMass(points_in_polygon, sensory_values, mass);
    centroids[i] = center_of_mass;
  }

  if (!previous_reference_flag_)
  {
    previous_reference_ = initializePreviousCentroids(num_agents_);
    previous_reference_flag_ = true;
  }

  return centroids;
}

void MultiPacejkaLloydPlanner::computeVoronoiDiagram(
    const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state)
{
  jcv_rect tmp_bounding_box = { { -3.0, -3.0 }, { 3.0, 3.0 } };
  bounding_box_ = tmp_bounding_box;

  // setup voronoi variables
  jcv_diagram diagram;
  jcv_point points[num_agents_];
  const jcv_site* sites;
  jcv_graphedge* graph_edge;
  memset(&diagram, 0, sizeof(jcv_diagram));
  bool update_voronoi_flag = true;
  std::vector<cartesian_reference_point> initial_state(num_agents_);

  // set initial_state_
  if (initial_state_flag_)
  {
    initial_state_ = setInitialPosition(num_agents_, state);
    initial_state_flag_ = false;
  }

  // set positions of agents to seed sites
  int i = 0;
  std::map<double, int> agent_index_map;
  for (const auto& ns_and_state : state)
  {
    points[i].x = ns_and_state.second.pos_x;  // + ns_and_state.second.pos_x;
    points[i].y = ns_and_state.second.pos_y;  // + ns_and_state.second.pos_y;
    std::cout << points[i].x;
    std::cout << points[i].y;
    agent_index_map.insert(std::pair<double, int>(ns_and_state.second.pos_x, i));
    i++;
  }

  // generate voronoi diagram
  jcv_diagram_generate(num_agents_, (const jcv_point*)points, &bounding_box_, 0, &diagram);

  // extract partition data from diagram
  sites = jcv_diagram_get_sites(&diagram);

  // extract vertices of cells
  std::vector<std::vector<std::pair<cartesian_reference_point, cartesian_reference_point>>> tmp_vor_cell_edges(
      diagram.numsites);
  std::vector<std::vector<cartesian_reference_point>> tmp_vor_cell_vertices(diagram.numsites);
  for (int i = 0; i < num_agents_; i++)
  {
    graph_edge = sites[i].edges;
    std::cout << sites[i].p.x;
    std::cout << sites[i].p.y;
    while (graph_edge)
    {
      cartesian_reference_point a, b;
      a.x = (double)graph_edge->pos[0].x;
      a.y = (double)graph_edge->pos[0].y;
      b.x = (double)graph_edge->pos[1].x;
      b.y = (double)graph_edge->pos[1].y;
      std::pair<cartesian_reference_point, cartesian_reference_point> edge = std::make_pair(a, b);
      tmp_vor_cell_edges[i].push_back(edge);
      tmp_vor_cell_vertices[i].push_back(a);
      std::cout << a.x;
      std::cout << ",";
      std::cout << a.y;

      graph_edge = graph_edge->next;
    }
  }

  // sort sites to match input to voronoi calculation
  std::vector<int> new_order;
  for (int i = 0; i < num_agents_; i++)
  {
    int idx = agent_index_map.find(double(sites[i].p.x))->second;
    new_order.push_back(idx);
    std::cout << new_order[i];
  }

  std::vector<std::pair<int, std::vector<cartesian_reference_point>>> vor_cell_vertices_pair;

  for (int i = 0; i < num_agents_; i++)
  {
    vor_cell_vertices_pair.push_back(
        std::pair<int, std::vector<cartesian_reference_point>>(new_order[i], tmp_vor_cell_vertices[i]));
  }

  std::sort(vor_cell_vertices_pair.begin(), vor_cell_vertices_pair.end(),
            [](const std::pair<int, std::vector<cartesian_reference_point>>& a,
               const std::pair<int, std::vector<cartesian_reference_point>>& b) { return a.first < b.first; });

  std::vector<std::vector<cartesian_reference_point>> ordered_vor_cell_vertices;
  for (auto pair : vor_cell_vertices_pair)
  {
    ordered_vor_cell_vertices.push_back(pair.second);
  }

  if (update_voronoi_flag)
  {
    vor_cell_edges_ = tmp_vor_cell_edges;
    vor_cell_vertices_ = ordered_vor_cell_vertices;
  }

  jcv_diagram_free(&diagram);
}

std::vector<std::vector<double>> MultiPacejkaLloydPlanner::getVorEdgesX()
{
  std::vector<std::vector<double>> vor_edges_x;
  for (int i = 0; i < vor_cell_edges_.size(); i++)
  {
    std::vector<double> vor_edges_single_car_x;
    for (int j = 0; j < vor_cell_edges_[i].size(); j++)
    {
      vor_edges_single_car_x.push_back(vor_cell_edges_[i][j].first.x);
    }
    vor_edges_x.push_back(vor_edges_single_car_x);
  }
  return vor_edges_x;
}

std::vector<std::vector<double>> MultiPacejkaLloydPlanner::getVorEdgesY()
{
  std::vector<std::vector<double>> vor_edges_y;
  for (int i = 0; i < vor_cell_edges_.size(); i++)
  {
    std::vector<double> vor_edges_single_car_y;
    for (int j = 0; j < vor_cell_edges_[i].size(); j++)
    {
      vor_edges_single_car_y.push_back(vor_cell_edges_[i][j].first.y);
    }
    vor_edges_y.push_back(vor_edges_single_car_y);
  }
  return vor_edges_y;
}

void MultiPacejkaLloydPlanner::check_change_in_centroid(double precision)
{
  double euclidean_change = 0.0;
  for (int i = 0; i < num_agents_; i++)
  {
    euclidean_change += std::sqrt(std::pow((previous_reference_[i].x - centroids_[i].x), 2) +
                                  std::pow((previous_reference_[i].y - centroids_[i].y), 2));
  }
  if (euclidean_change < precision)
  {
    for (int i = 0; i < num_agents_; i++)
    {
      centroids_[i].x = initial_state_[i].x;  // + ns_and_state.second.pos_x;
      centroids_[i].y = initial_state_[i].y;  // + ns_and_state.second.pos_y;
    }
  }
}

std::vector<cartesian_reference_point> MultiPacejkaLloydPlanner::setInitialPosition(
    int num_agents, const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state)
{
  std::vector<cartesian_reference_point> initial_state(num_agents);
  int i = 0;
  for (const auto& ns_and_state : state)
  {
    initial_state[i].x = ns_and_state.second.pos_x;  // + ns_and_state.second.pos_x;
    initial_state[i].y = ns_and_state.second.pos_y;  // + ns_and_state.second.pos_y;
    i++;
  }
  return initial_state;
}

std::vector<cartesian_reference_point> MultiPacejkaLloydPlanner::initializePreviousCentroids(int num_agents)
{
  std::vector<cartesian_reference_point> previous_reference(num_agents);
  for (int i = 0; i < num_agents; i++)
  {
    previous_reference[i].x = 0.0;
    previous_reference[i].y = 0.0;
  }
  return previous_reference;
}

}  // namespace crs_planning
