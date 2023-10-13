#include <sbmpo/SBMPO.hpp>
#include <my_project/MyCustomModel.hpp>
#include <sbmpo/tools/PrintTool.hpp>

using namespace my_namespace;

int main(int argc, char ** argv) {

  sbmpo::SearchParameters params;
  /* Add in parameters here */
  params.max_iterations = 500000;
  params.max_generations = 1000;
  params.start_state = {1, 8, 0};
  params.goal_state = {15, 8, 0};
  params.sample_time = 0.10f;
  params.grid_resolution = {0.10f, 0.10f, 0.10f};
  params.samples = {
    {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {-1, 1, 0},
    {-1, 0, 0}, {-1, -1, 0}, {0, -1, 0}, {1, -1, 0},
    {1, 0, 1}, {1, 1, 1}, {0, 1, 1}, {-1, 1, 1},
    {-1, 0, 1}, {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
  };
  
  sbmpo::SBMPO<MyCustomModel> planner;

  Occupancy3D occupancy = Occupancy3D::fromFile("/sbmpo_ws/occupancy.csv");
  planner.model()->setOccupancy3D(occupancy);

  for (float x = 0; x < 20; x += 0.2)
  {
    for (float y = 0; y < 20; y += 0.2)
    {
      bool occ = occupancy.getOccupancy(x, y, 0);
      if (occ)
      {
        printf("(%.1f,%.1f) = %d", x, y, occ);
        printf("\n");
      }
    }
  }

  planner.run(params);

  sbmpo_io::print_parameters(params);
  sbmpo_io::print_results(planner.results());
  sbmpo_io::print_stats(planner.results());

  return 0;
}