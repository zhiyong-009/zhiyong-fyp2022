#ifndef RAM_QT_GUIS_ALGORITHMS_WIDGETS_POLYGON_OFFSETS_HPP
#define RAM_QT_GUIS_ALGORITHMS_WIDGETS_POLYGON_OFFSETS_HPP

#include <mutex>

#include <ram_utils/file_extension.hpp>
#include <ram_path_planning/polygon_offsets.hpp>
#include <ram_path_planning/PolygonOffsetsAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace ram_qt_guis
{
class PolygonOffsetsWidget : public QWidget
{
Q_OBJECT
  public:
  PolygonOffsetsWidget();
  virtual ~PolygonOffsetsWidget();

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  std::string fillGoal(ram_path_planning::PolygonOffsetsGoal &goal);

Q_SIGNALS:
  void valueChanged();

protected Q_SLOTS:
  void browseFiles();
  void joinTypeChanged();

private:
  QVBoxLayout *main_layout_;

  QLineEdit *file_;
  QSpinBox *number_of_layers_;
  QDoubleSpinBox *height_between_layers_;
  QDoubleSpinBox *deposited_material_width_;
  QDoubleSpinBox *connection_value_;
  QDoubleSpinBox *safe_distance_;
  QDoubleSpinBox *offset_factor_;
  QCheckBox *towards_interior_;
  QComboBox *connection_type_;
  QCheckBox *change_angle_;
  QCheckBox *reverse_origin_path_;
  QCheckBox *discontinous_trajectory_;
  QCheckBox *avoid_trajectories_crossing_;
  QCheckBox *automatic_reverse_path_;
  QSpinBox *number_of_passes_;
  QComboBox *join_type_;
  QComboBox *end_type_;
  QDoubleSpinBox *arc_tolerance_;
  QDoubleSpinBox *miter_limit_;
  QSpinBox *arc_points_;
};

}

#endif
