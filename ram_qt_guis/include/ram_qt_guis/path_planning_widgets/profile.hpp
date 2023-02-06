#ifndef RAM_QT_GUIS_ALGORITHMS_WIDGETS_PROFILE_HPP
#define RAM_QT_GUIS_ALGORITHMS_WIDGETS_PROFILE_HPP

#include <mutex>

#include <ram_utils/file_extension.hpp>
#include <ram_path_planning/profile.hpp>
#include <ram_path_planning/ProfileAction.h>
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
class ProfileWidget : public QWidget
{
Q_OBJECT
  public:
  ProfileWidget();
  virtual ~ProfileWidget();

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  std::string fillGoal(ram_path_planning::ProfileGoal &goal);

Q_SIGNALS:
  void valueChanged();

protected Q_SLOTS:
  void browseFiles();

private:
  QVBoxLayout *main_layout_;

  QLineEdit *file_;
  QDoubleSpinBox *height_between_layers_;
  QDoubleSpinBox *deposited_material_width_;
  QCheckBox *towards_interior_;
  QCheckBox *slice_along_path_;
  QSpinBox *number_of_passes_;
  QSpinBox *angle_percentage_;
  QComboBox *angle_type_;
  QSpinBox *arc_points_;
};

}

#endif
