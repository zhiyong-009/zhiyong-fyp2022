#ifndef RAM_QT_GUIS_ALGORITHMS_WIDGETS_FOLLOW_POSES_WIDGET_HPP
#define RAM_QT_GUIS_ALGORITHMS_WIDGETS_FOLLOW_POSES_WIDGET_HPP

#include <mutex>

#include <ram_utils/file_extension.hpp>
#include <ram_path_planning/follow_poses.hpp>
#include <ram_path_planning/FollowPosesAction.h>
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
class FollowPosesWidget : public QWidget
{
Q_OBJECT
  public:
  FollowPosesWidget();
  virtual ~FollowPosesWidget();

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  std::string fillGoal(ram_path_planning::FollowPosesGoal &goal);

Q_SIGNALS:
  void valueChanged();

protected Q_SLOTS:
  void browseFiles();
  void enableDisableDuplicateLayers();

private:
  QVBoxLayout *main_layout_;
  QLineEdit *file_;
  QCheckBox *duplicate_layer_;
  QWidget *duplicate_layer_widget_;
  QSpinBox *number_of_layers_;
  QDoubleSpinBox *height_between_layers_;
  QCheckBox *invert_one_of_two_layers_;
  QSpinBox *arc_points_;
  QSpinBox *rotate_poses_;

  const double default_height_between_layers_ = 1;
};

}

#endif
