#ifndef RAM_QT_GUIS_MODIFY_WIDGETS_MODIFY_POSES_HPP
#define RAM_QT_GUIS_MODIFY_WIDGETS_MODIFY_POSES_HPP

#include <mutex>
#include <ram_qt_guis/pose.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace ram_qt_guis
{
class ModifyPoses : public QWidget
{
Q_OBJECT
  public:
  ModifyPoses(QVBoxLayout* layout,
              const unsigned mode);
  virtual ~ModifyPoses();

  QDialogButtonBox *button_box_;

Q_SIGNALS:
  void enable(const bool);

protected Q_SLOTS:
  void modifyPose();
  void poseAbsRel(const int index);
  void modifyPolygonStart();
  void modifyPolygonEnd();
  void modifyMovementType();
  void movementTypeChanged();
  void modifyApproachType();
  void modifyBlendRadius();
  void blendRadiusAbsRel(const int index);
  void modifySpeed();
  void speedAbsRel(const int index);
  void modifyLaserPower();
  void laserPowerAbsRel(const int index);
  void modifyFeedRate();
  void feedRateAbsRel(const int index);

private:
  QVBoxLayout *layout_;
  QWidget *pose_widget_;
  QWidget *polygon_end_widget_;
  QWidget *movement_type_widget_;
  QWidget *polygon_start_widget_;
  QWidget *approach_type_widget_;
  QWidget *blend_radius_widget_;
  QWidget *speed_widget_;
  QWidget *laser_power_widget_;
  QWidget *feed_rate_widget_;

  const unsigned mode_;

public:
  QCheckBox *pose_modify_;
  QComboBox *pose_abs_rel_;
  Pose *pose_;

  QCheckBox *polygon_start_modify_;
  QComboBox *polygon_start_;

  QCheckBox *polygon_end_modify_;
  QComboBox *polygon_end_;

  QCheckBox *movement_type_modify_;
  QComboBox *movement_type_;

  QCheckBox *approach_type_modify_;
  QComboBox *approach_type_;

  QCheckBox *blend_radius_modify_;
  QComboBox *blend_radius_abs_rel_;
  QSpinBox *blend_radius_;

  QCheckBox *speed_modify_;
  QComboBox *speed_abs_rel_;
  QDoubleSpinBox *speed_;
  double speed_conversion_factor_; // Used by speed_

  QCheckBox *laser_power_modify_;
  QComboBox *laser_power_abs_rel_;
  QSpinBox *laser_power_;

  QCheckBox *feed_rate_modify_;
  QComboBox *feed_rate_abs_rel_;
  QDoubleSpinBox *feed_rate_;
};

}

#endif
