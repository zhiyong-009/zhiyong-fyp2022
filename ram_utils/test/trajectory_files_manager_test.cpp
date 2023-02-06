#include <ram_utils/trajectory_files_manager_1.hpp>
#include <ros/package.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(TestSuite, testWriteYamlFile)
{
  //make poly_data
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> poly_data_2 = vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> polygon_array = vtkSmartPointer<vtkCellArray>::New();

  // First contour
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(0, 0, 0);
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(0, 2, 0);
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(2, 2, 0);
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(2, 0, 0);

  polygon_array->InsertNextCell(polygon);
  // Second contour
  polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(10, 10, 0);
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(11, 11, 0);
  polygon->GetPointIds()->InsertNextId(points->GetNumberOfPoints());
  points->InsertNextPoint(12, 10, 0);

  polygon_array->InsertNextCell(polygon);

  poly_data->SetPoints(points);
  poly_data->SetPolys(polygon_array);

  const std::string yaml_file(ros::package::getPath("ram_utils") + "/test/testWriteYamlFile.yaml");
  // std::string yaml_file = "testWriteYamlFile.yaml";
  EXPECT_TRUE(ram_utils::polydataToYamlFile(yaml_file, poly_data));
}

TEST(TestSuite, testReadEmptyPolydata)
{
  //make poly_data
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  const std::string yaml_file(ros::package::getPath("ram_utils") + "/test/testWriteYamlFile2.yaml");
  EXPECT_FALSE(ram_utils::polydataToYamlFile(yaml_file, poly_data));
}

TEST(TestSuite, testReadYamlFile)
{
  //make poly_data
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  const std::string yaml_file(ros::package::getPath("ram_utils") + "/test/testReadYamlFile.yaml");
  bool result = ram_utils::yamlFileToPolydata(yaml_file, poly_data);
  bool is_empty = (poly_data->GetNumberOfCells() == 0) ? true : false;
  EXPECT_TRUE(result);
  EXPECT_FALSE(is_empty);
}

TEST(TestSuite, testReadEmptyYamlFile)
{
  //make poly_data
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  const std::string yaml_file(ros::package::getPath("ram_utils") + "/test/testReadEmptyYamlFile.yaml");
  bool result = ram_utils::yamlFileToPolydata(yaml_file, poly_data);
  bool is_empty = (poly_data->GetNumberOfCells() == 0) ? true : false;
  EXPECT_FALSE(result);
  EXPECT_TRUE(is_empty);
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "trajectory_files_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
