#ifndef RAM_UTILS_TRAJECTORY_FILES_MANAGER_1_HPP
#define RAM_UTILS_TRAJECTORY_FILES_MANAGER_1_HPP

#include <Eigen/Geometry>
#include <fstream>
#include <ram_utils/yaml_utils.hpp>
#include <string>
#include <vector>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkVector.h>

namespace ram_utils
{
using Pose = Eigen::Isometry3d;
using PosesPolygon = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
using PosesLayer = std::vector<PosesPolygon>;
using PosesTrajectory = std::vector<PosesLayer>;

using Polygon = vtkSmartPointer<vtkPolyData>;
using Polygons = std::vector<Polygon>;
using PolygonsVector = std::vector<Polygons>;

bool yamlFileToPolydata(const std::string yaml_file,
                        Polygon poly_data);

/**
 * @param[in] yaml_file the name of the YAML file to be loaded
 * @param[out] poly_data the poly data to be filled
 * @param[out] layer_count a vector representing the number of layers (size of the vector) and the
 * number of polygons inside each layer (unsigned value)
 * @return True if successful, false otherwise
 */
bool yamlFileToPolydata2(const std::string yaml_file,
                         Polygon poly_data,
                         std::vector<unsigned> &layer_count);

bool polydataToYamlFile(const std::string yaml_file,
                        const Polygon poly_data);

bool polydataToYamlFile2(const std::string yaml_file,
                         const Polygon poly_data);

void writeYAMLFile(const PosesTrajectory &trajectory,
                   const std::string file_name,
                   const bool no_linear = false,
                   const std::string header = "");
}

#endif
