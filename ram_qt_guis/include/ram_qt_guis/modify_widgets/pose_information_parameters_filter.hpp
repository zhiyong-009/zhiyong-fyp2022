#ifndef RAM_QT_GUIS_MODIFY_WIDGETS_POSE_INFORMATION_PARAMETERS_FILTER_HPP
#define RAM_QT_GUIS_MODIFY_WIDGETS_POSE_INFORMATION_PARAMETERS_FILTER_HPP

#include <QRadioButton>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace ram_qt_guis
{
class PoseInformationParametersFilter : public QWidget
{
Q_OBJECT
public:
  PoseInformationParametersFilter(QVBoxLayout *layout,
                                  const QString &help_string = "");
  virtual ~PoseInformationParametersFilter();

  QDialogButtonBox *button_box_;
  bool getStartStatus();
  bool getEndStatus();
  bool getStartEndStatus();
  bool getLastPoseInLayerStatus();

private:
  QVBoxLayout *layout_;
  QRadioButton *start_poses_;
  QRadioButton *end_poses_;
  QRadioButton *star_end_poses_;
  QRadioButton *last_pose_in_layer_;

  const QString help_string_;

};
}

#endif
