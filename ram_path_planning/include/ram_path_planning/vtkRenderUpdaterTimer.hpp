#ifndef RAM_PATH_PLANNING_VTK_RENDER_UPDATER_TIMER_HPP
#define RAM_PATH_PLANNING_VTK_RENDER_UPDATER_TIMER_HPP

#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>

#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/donghong_ding.hpp>

using Polygon = vtkSmartPointer<vtkPolyData>;
using PolygonVector = std::vector<Polygon>;
using Layer = std::vector<PolygonVector>;

class vtkRendererUpdaterTimer : public vtkCommand
{
public:
  static vtkRendererUpdaterTimer *New()
  {
    vtkRendererUpdaterTimer *cb = new vtkRendererUpdaterTimer;
    return cb;
  }

  vtkSmartPointer<vtkPolyData> mesh_ = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkStripper> stripper_;
  Layer current_layer_;

  virtual void Execute(vtkObject *caller,
                       unsigned long,
                       void * vtkNotUsed(callData))
  {
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    vtkRenderer * renderer = iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

    int representation(1);
    vtkActor* last_actor(renderer->GetActors()->GetLastActor());

    if (last_actor)
      representation = last_actor->GetProperty()->GetRepresentation();

    renderer->RemoveAllViewProps(); // Remove all actors

    // Add mesh
    if (mesh_)
    {
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputData(mesh_);

      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);

      actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
      actor->GetProperty()->SetOpacity(0.5);
      actor->GetProperty()->SetRepresentation(representation);
      renderer->AddActor(actor);
    }

    if (stripper_)
    {
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(stripper_->GetOutputPort());
      mapper->ScalarVisibilityOff();

      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);

      actor->GetProperty()->SetColor(1, 0, 0);
      actor->GetProperty()->SetLineWidth(2.0);
      actor->GetProperty()->SetRepresentation(representation);
      renderer->AddActor(actor);
    }

    // Add all polygons
    for (unsigned i = 0; i < current_layer_.size(); ++i)
    {
      unsigned color_index(0);
      for (auto polygon : current_layer_[i])
      {
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polygon);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        double rgb[3];
        getColorFromIndex(color_index++, rgb);

        actor->GetProperty()->SetColor(rgb);
        actor->GetProperty()->SetRepresentation(representation);
        renderer->AddActor(actor);
      }
    }
    iren->GetRenderWindow()->Render();
  }

private:
  void getColorFromIndex(const unsigned i,
                         double rgb[3])
  {
    switch (i % 6)
    {
      case 0:
        rgb[0] = 0.3;
        rgb[1] = 0.3;
        rgb[2] = 1;
        break;
      case 1:
        rgb[0] = 0.3;
        rgb[1] = 1;
        rgb[2] = 0.3;
        break;
      case 2:
        rgb[0] = 0.3;
        rgb[1] = 1;
        rgb[2] = 1;
        break;
      case 3:
        rgb[0] = 1;
        rgb[1] = 0.3;
        rgb[2] = 0.3;
        break;
      case 4:
        rgb[0] = 1;
        rgb[1] = 0.3;
        rgb[2] = 1;
        break;
      case 5:
        rgb[0] = 1;
        rgb[1] = 1;
        rgb[2] = 0.3;
        break;
      default:
        rgb[0] = 1;
        rgb[1] = 1;
        rgb[2] = 1;
        break;
    }
  }
};

#endif
