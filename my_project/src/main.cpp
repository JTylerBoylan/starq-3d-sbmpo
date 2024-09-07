#include <sbmpo/SBMPO.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>
#include <my_project/MyCustomModel.hpp>
#include <sbmpo/tools/PrintTool.hpp>
#include <sbmpo/tools/CSVTool.hpp>

using namespace my_namespace;

int main(int argc, char **argv)
{

  SDF3D sdf = SDF3D::fromFile("/sbmpo_ws/sdf.csv");
  sbmpo_benchmarks::Benchmark<MyCustomModel> benchmark("/sbmpo_ws/my_project/csv/");

  benchmark.model()->setSDF3D(sdf);
  benchmark.model()->set_min_distance(0.25);
  benchmark.set_verbose(true);
  benchmark.set_dynamic_sampling(
      std::bind(&MyCustomModel::getControlSamples, benchmark.model(), std::placeholders::_1));

  benchmark.benchmark();

  return 0;
}