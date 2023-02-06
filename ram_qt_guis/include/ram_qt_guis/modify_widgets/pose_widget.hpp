#ifndef RAM_QT_GUIS_MODIFY_WIDGETS_POSE_WIDGET_HPP
#define RAM_QT_GUIS_MODIFY_WIDGETS_POSE_WIDGET_HPP

#include <Eigen/Geometry>

#include <rviz/panel.h>

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

namespace ram_qt_guis
{
class Pose : public QWidget
{
Q_OBJECT
  public:
  Pose(const QString name,
       QWidget* parent = NULL);

  void save(rviz::Config);
  void load(const rviz::Config&);

  void setTranslationEnabled(const bool);
  void setText(const QString name);
  void getPose(Eigen::Affine3d &pose);
  Eigen::Affine3d getPose();
  void resetPose();

  QVBoxLayout* getLayout();

Q_SIGNALS:
  void valueChanged();

protected Q_SLOTS:
  void computePose();

protected:
  const double deg2rad_;
  const double rad2deg_;

  Eigen::Affine3d convertXYZWPRtoMatrix(const double x,
                                        const double y,
                                        const double z,
                                        const double w,
                                        const double p,
                                        const double r);

  Eigen::Affine3d pose_;

  QVBoxLayout *layout_;
  QLabel *label_pose_;
  QDoubleSpinBox *pose_x_;
  QDoubleSpinBox *pose_y_;
  QDoubleSpinBox *pose_z_;
  QDoubleSpinBox *pose_w_;
  QDoubleSpinBox *pose_p_;
  QDoubleSpinBox *pose_r_;

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
