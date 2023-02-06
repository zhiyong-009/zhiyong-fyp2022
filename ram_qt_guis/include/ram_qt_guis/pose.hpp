#ifndef RAM_QT_GUIS_POSE_HPP
#define RAM_QT_GUIS_POSE_HPP

#include <Eigen/Geometry>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <rviz/panel.h>

namespace ram_qt_guis
{

class Pose : public QWidget
{
  Q_OBJECT

public:
  enum Mode
  {
    READ_ONLY,
    READ_WRITE,
    READ_WRITE_ORIENTATION
  };

  enum PoseType
  {
    MATRIX,
    FANUC,
    MOTOMAN
  };

  Pose(const QString name,
       const QString description,
       const Mode mode = READ_WRITE,
       const PoseType pose_type = FANUC,
       const Eigen::Isometry3d &pose = Eigen::Isometry3d::Identity(),
       QWidget *parent = nullptr);

  Eigen::Isometry3d pose();

  void save(rviz::Config);
  void load(const rviz::Config &);

Q_SIGNALS:
  void valueChanged(const Eigen::Isometry3d &m);
  void setPose(const Eigen::Isometry3d &m);

protected Q_SLOTS:
  void modeChanged(const unsigned index);
  void updatePose(const Eigen::Isometry3d &m);
  void matrixChanged();
  void fanucChanged();
  void motomanChanged();

private:
  Mode mode_;
  Eigen::Isometry3d pose_;
  void updateMatrixGUI();
  void updateFanucGUI();
  void updateMotomanGUI();

  QComboBox *mode_select_;
  std::vector<QDoubleSpinBox *> matrix_;
  std::vector<QDoubleSpinBox *> fanuc_;
  std::vector<QDoubleSpinBox *> motoman_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
