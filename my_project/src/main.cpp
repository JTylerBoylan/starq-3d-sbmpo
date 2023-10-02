#include <sbmpo/SBMPO.hpp>
#include <my_project/MyCustomModel.hpp>
#include <sbmpo/tools/PrintTool.hpp>

using namespace my_namespace;

int main(int argc, char ** argv) {

  sbmpo::SearchParameters params;
  /* Add in parameters here */
  params.max_iterations = 5000;
  params.max_generations = 100;
  params.start_state = {0, 0};
  params.goal_state = {10, 10};
  params.sample_time = 0.5f;
  params.grid_resolution = {0.25f, 0.25f};
  params.samples = {
    {1, 0}, {1, 1}, {0, 1}, {-1, 1},
    {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
  };
  
  sbmpo::SBMPO<MyCustomModel> planner;
  planner.run(params);

  sbmpo_io::print_parameters(params);
  sbmpo_io::print_results(planner.results());
  sbmpo_io::print_stats(planner.results());

  return 0;
}