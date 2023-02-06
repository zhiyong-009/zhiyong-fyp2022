#ifndef RAM_QT_GUIS_MODIFY_HPP
#define RAM_QT_GUIS_MODIFY_HPP

#include <eigen_conversions/eigen_msg.h>
#include <mutex>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTableView>
#include <QTableWidget>
#include <QTabWidget>
#include <QtConcurrent/QtConcurrentRun>
#include <QtCore/QStringListModel>
#include <QtGui/QStandardItemModel>
#include <QTreeWidget>
#include <QtWidgets>
#include <QVBoxLayout>
#include <ram_display/UpdateSelection.h>
#include <ram_modify_trajectory/AddPoses.h>
#include <ram_modify_trajectory/ChangeLayerHeight.h>
#include <ram_modify_trajectory/DeleteSelectedPoses.h>
#include <ram_modify_trajectory/GetPosesFromInformation.h>
#include <ram_modify_trajectory/GetPosesFromLayer.h>
#include <ram_modify_trajectory/GetPosesFromLayersList.h>
#include <ram_modify_trajectory/GetPosesFromTrajectory.h>
#include <ram_modify_trajectory/ModifySelectedPoses.h>
#include <ram_modify_trajectory/PushPullAngle.h>
#include <ram_modify_trajectory/ReflectSelectedPoses.h>
#include <ram_modify_trajectory/ResetSelectedPoses.h>
#include <ram_modify_trajectory/RotateSelectedPoses.h>
#include <ram_modify_trajectory/ScaleSelectedPoses.h>
#include <ram_modify_trajectory/ShiftPoses.h>
#include <ram_modify_trajectory/TrajectoryInterruption.h>
#include <ram_modify_trajectory/TweakBlendRadius.h>
#include <ram_modify_trajectory/SimplifyTrajectory.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_qt_guis/modify_widgets/blend_radius_table.hpp>
#include <ram_qt_guis/modify_widgets/modify_poses.hpp>
#include <ram_qt_guis/modify_widgets/range_list_selection.hpp>
#include <ram_qt_guis/modify_widgets/pose_information_parameters_filter.hpp>
#include <ram_utils/GetLayerSize.h>
#include <ram_utils/GetNumberOfLayersLevels.h>
#include <ram_utils/GetTrajectorySize.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

namespace ram_qt_guis
{
class Modify : public rviz::Panel
{
Q_OBJECT
public:
  Modify(QWidget *parent = NULL);
  virtual ~Modify();
  void connectToService(ros::ServiceClient &client);
  void connectToServices();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void trajReceived(const ram_msgs::AdditiveManufacturingTrajectory::Ptr &msg);

  void clearLayout(QLayout *layout,
                   bool delete_widgets = true);
  void changeGUIToSelectionMode();
  void changeGUIToRangeListSelection(const QString help_string,
                                     const unsigned min,
                                     const unsigned max);
  void changeGUIToInformationFilterSelection(const QString help_string);
  void changeGUIToOperationSelection();
  void changeGUIToModifyPoses();
  void changeGUIToRotatePoses();
  void changeGUIToReflectPoses();
  void changeGUIToScalePoses();
  void changeGUIToShiftPoses();
  void changeGUIToTrajectoryInterruption();
  void changeGUIToHeightModification();
  void changeGUIToTweakBlendRadius();
  void changeGUIToSimplifyTrajectory();
  void changeGUIToPushPullAngle();
  void getSelection();
  void getSelectionFromInformationFilter();

protected Q_SLOTS:
  void selectionModeSelected();
  void propagateSelection();
  void operationSelected();
  void modifyPoses();
  void rotatePoses();
  void reflectPoses();
  void scalePoses();
  void shiftPoses();
  void trajectoryInterruption();
  void heightModification();

  void load(const rviz::Config &config);
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void updateTemporarySelection(std::vector<unsigned> selection);

protected:
  QVBoxLayout *layout_;
  // UI for information filter selection
  PoseInformationParametersFilter * information_filter_selection_ui_;

  // UI for range list selection
  ModifyRangeListSelection *range_list_selection_ui_;

  // Selection mode
  QVector<QRadioButton *> selection_buttons_;

  // Modify pose UI
  ModifyPoses *modify_poses_ui_;

  // Rotate pose UI
  QDoubleSpinBox *rotation_angle_;
  QDoubleSpinBox *rotation_point_x_;
  QDoubleSpinBox *rotation_point_y_;

  // Reflect pose UI
  QDoubleSpinBox *reflect_vector_x_;
  QDoubleSpinBox *reflect_vector_y_;

  QDoubleSpinBox *reflect_point_x_;
  QDoubleSpinBox *reflect_point_y_;

  // Scaling pose UI
  QDoubleSpinBox *scale_factor_;
  QDoubleSpinBox *scale_center_x_;
  QDoubleSpinBox *scale_center_y_;

  // Layer shift UI
  QDoubleSpinBox *shift_direction_angle_;
  QDoubleSpinBox *shift_angle_z_;

  // Trajectory interruption
  QCheckBox *traj_interruption_add_pose_;

  // Change layers height
  QComboBox *mode_;
  QDoubleSpinBox *new_height_;

  // Tweak blend radius
  BlendRadiusTable *blend_radius_table_;

  // Simplify trajectory
  QDoubleSpinBox *simplify_trajectory_threshold_;

  // Push pull angle
  QDoubleSpinBox *push_pull_angle_;

  // Operation selection
  std::vector<QRadioButton *> operations_;
  std::vector<QRadioButton *> geometric_operations_;

  // Selection
  std::vector<ram_msgs::AdditiveManufacturingPose> selected_poses_;
  unsigned selection_mode_;
  unsigned layer_level_; // Only makes sense if selection mode is 2
  bool is_propagating_;  // Only makes sense if selection mode is 2
  std::vector<unsigned> layers_to_propagate_; // Only makes sense if selection mode is 2
  std::vector<unsigned> relative_indices_; // Only makes sense if selection mode is 2

  ros::NodeHandle nh_;

  // Subscribe to trajectory topic
  ros::Subscriber traj_;
  std::mutex trajectory_mutex_;
  ram_msgs::AdditiveManufacturingTrajectory trajectory_;

  // Selection visualization
  ros::ServiceClient update_selection_client_;

  // Pose selector services
  ros::ServiceClient get_layer_size_client_;
  ros::ServiceClient get_number_of_layers_client_;
  ros::ServiceClient get_poses_from_layer_client_;
  ros::ServiceClient get_poses_from_layers_list_client_;
  ros::ServiceClient get_poses_from_trajectory_client_;
  ros::ServiceClient get_trajectory_size_client_;
  ros::ServiceClient get_poses_from_information_;

  // Modify services
  ros::ServiceClient modify_selected_poses_client_;
  ros::ServiceClient delete_selected_poses_client_;
  ros::ServiceClient add_poses_client_;
  ros::ServiceClient reset_selected_poses_client_;
  ros::ServiceClient rotate_selected_poses_client_;
  ros::ServiceClient reflect_selected_poses_client_;
  ros::ServiceClient scale_selected_poses_client_;
  ros::ServiceClient shift_poses_client_;
  ros::ServiceClient trajectory_interruption_client_;
  ros::ServiceClient change_layer_height_client_;
  ros::ServiceClient tweak_blend_radius_client_;
  ros::ServiceClient simplify_trajectory_client_;
  ros::ServiceClient push_pull_angle_client_;
};

}

#endif
