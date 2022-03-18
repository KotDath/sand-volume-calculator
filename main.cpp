#include <open3d/Open3D.h>

double CalculateMedian(std::vector<Eigen::Vector3d> &points) {
  std::cout << "Calculation of Z axis median...\n";
  std::sort(points.begin(),
            points.end(),
            [](const Eigen::Vector3d &first,
               const Eigen::Vector3d &second) {
              return first.z() < second.z();
            });
  double medianZ = (points.begin() + points.size() / 2)->z();
  std::cout << "Z axis median: " << medianZ << std::endl;
  return medianZ;
}

std::shared_ptr<open3d::geometry::PointCloud>
CropByMedian(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud, double medianZ) {
  Eigen::Vector3d minBound = pointcloud->GetMinBound();;
  Eigen::Vector3d maxBound = pointcloud->GetMaxBound();
  minBound.z() = medianZ;

  open3d::geometry::AxisAlignedBoundingBox box{minBound, maxBound};
  return pointcloud->Crop(box);
}

double CalculateVolume(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {

  double result = 0;

  auto vertexLower = [](const Eigen::Vector3d &first, const Eigen::Vector3d &second) {
    return first.z() < second.z();
  };

  auto op = [&result, &mesh, &vertexLower](const Eigen::Vector3i &triange) {
    auto A = mesh->vertices_[triange[0]];
    auto B = mesh->vertices_[triange[1]];
    auto C = mesh->vertices_[triange[2]];

    auto lowestVertex = A;
    if (vertexLower(B, lowestVertex)) {
      lowestVertex = B;
    }

    if (vertexLower(C, lowestVertex)) {
      lowestVertex = C;
    }

    auto
        projS = 0.5 * (A.x() * B.y() + B.x() * C.y() + C.x() * A.y() - A.y() * B.x() - B.y() * C.x() - C.y() * A.x());
    result += projS * lowestVertex.z();


    auto highestVertex = A;
    if (vertexLower(highestVertex, B)) {
      highestVertex = B;
    }

    if (vertexLower(highestVertex, C)) {
      highestVertex = C;
    }

    result += projS * (highestVertex  .z() - lowestVertex.z()) / 3;
  };

  std::for_each(mesh->triangles_.begin(), mesh->triangles_.end(), op);
  return result;
}

int main(int argc, char *argv[]) {
  open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
  open3d::utility::ProgressBar bar(100, "Loading file:", true);
  open3d::io::ReadPointCloudOption opt([&bar](double value) {
    bar.SetCurrentCount(static_cast<size_t>(value));
    return true;
  });

  if (argc <= 1) {
    std::cout << "Error. Path to point cloud file is empty.\n";
  } else {
    auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud(argv[1], *pointcloud, opt);
    if (pointcloud->IsEmpty()) {
      std::cout << fmt::format("Error. File {} is unreadable, empty or not exist", argv[1]);
    } else {
      auto medianZ = CalculateMedian(pointcloud->points_);
      pointcloud = CropByMedian(pointcloud, medianZ);

      open3d::visualization::Visualizer vis;
      vis.CreateVisualizerWindow("sand_map", 800, 600);
      pointcloud->EstimateNormals();
      auto mesh = (open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pointcloud, 9));
      std::cout << "Volume:" << CalculateVolume(std::get<0>(mesh)) << std::endl;
      vis.AddGeometry(pointcloud);
      auto& view = vis.GetViewControl();
      view.CameraLocalRotate(60, 60, 0);
      if (argc > 2) {
        vis.CaptureScreenImage(argv[2]);
      }
      vis.Run();
      vis.DestroyVisualizerWindow();
    }

  }

  return 0;
}
