#include <sbmpo/SBMPO.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>
#include <my_project/MyCustomModel.hpp>
#include <sbmpo/tools/PrintTool.hpp>
#include <sbmpo/tools/CSVTool.hpp>

using namespace my_namespace;

int main(int argc, char ** argv) {
  
  Occupancy3D occupancy = Occupancy3D::fromFile("/sbmpo_ws/occupancy.csv");
  sbmpo_benchmarks::Benchmark<MyCustomModel> benchmark("/sbmpo_ws/my_project/csv/");

  benchmark.model()->setOccupancy3D(occupancy);
  benchmark.set_verbose(true);

  for (float xi = 0.0; xi < 20.0; xi += 0.25)
  {
    for (float yi = 0.0; yi < 20.0; yi += 0.25)
    {
      bool occval = occupancy.getOccupancy(xi, yi, 0.5);
      if (occval)
      {
        printf("(X: %.2f, Y: %.2f) is occupied\n", xi, yi);
      }
    }
  }

  benchmark.benchmark();

  return 0;
}