#ifndef RAM_QT_GUIS_ALGORITHMS_WIDGETS_CONTOURS_HPP
#define RAM_QT_GUIS_ALGORITHMS_WIDGETS_CONTOURS_HPP

#include <mutex>

#include <ram_utils/file_extension.hpp>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/ContoursAction.h>
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
class ContoursWidget : public QWidget
{
Q_OBJECT
  public:
  ContoursWidget();
  virtual ~ContoursWidget();

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  std::string fillGoal(ram_path_planning::ContoursGoal &goal);

Q_SIGNALS:
  void valueChanged();

protected Q_SLOTS:
  void browseFiles();
  void fileChanged();

private:
  QVBoxLayout *main_layout_;

  QLineEdit *file_;
  QWidget *number_of_layers_widget_;
  QSpinBox *number_of_layers_;
  QDoubleSpinBox *height_between_layers_;
  QDoubleSpinBox *deposited_material_width_;

  QWidget *slicing_direction_widget_;
  QDoubleSpinBox *slicing_direction_x_;
  QDoubleSpinBox *slicing_direction_y_;
  QDoubleSpinBox *slicing_direction_z_;

  const double default_height_between_layers_ = 1;
  const double default_deposited_material_width_ = 1;
  const double default_slicing_direction_x_ = 0;
  const double default_slicing_direction_y_ = 0;
  const double default_slicing_direction_z_ = 1;
};

}

#endif
