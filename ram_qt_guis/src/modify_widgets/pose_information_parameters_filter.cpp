#include <ram_qt_guis/modify_widgets/pose_information_parameters_filter.hpp>

namespace ram_qt_guis
{
PoseInformationParametersFilter::PoseInformationParametersFilter(QVBoxLayout *layout,
                                                                 const QString &help_string) :
    layout_(layout),
    help_string_(help_string)
{
  setObjectName("ModifyPoseInformationParametersFilter");
  layout_->addWidget(new QLabel(help_string_));

  QHBoxLayout *start_poses_layout = new QHBoxLayout;
  start_poses_ = new QRadioButton;
  QLabel *start_poses_label = new QLabel("Select all 'Start' poses");
  start_poses_label->setToolTip("Select every poses within the trajectory that are 'Ploygon Start'");
  start_poses_layout->addWidget(start_poses_label);
  start_poses_layout->addWidget(start_poses_);

  QHBoxLayout *end_poses_layout = new QHBoxLayout;
  end_poses_ = new QRadioButton;
  QLabel *end_poses_label = new QLabel("Select all 'End' poses");
  start_poses_label->setToolTip("Select every poses within the trajectory that are 'Polygon End'");
  end_poses_layout->addWidget(end_poses_label);
  end_poses_layout->addWidget(end_poses_);

  QHBoxLayout *start_end_poses_layout = new QHBoxLayout;
  star_end_poses_ = new QRadioButton;
  QLabel *start_end_poses_label = new QLabel("Select all 'Start' AND 'End' poses");
  start_end_poses_label->setToolTip("Select every poses within the trajectory that are simultaneously "
                                    "'Polygon Start' AND 'Polygon End'");
  start_end_poses_layout->addWidget(start_end_poses_label);
  start_end_poses_layout->addWidget(star_end_poses_);

  QHBoxLayout *last_pose_in_layer_layout = new QHBoxLayout;
  last_pose_in_layer_ = new QRadioButton;
  QLabel *last_pose_in_layer_label = new QLabel("Select last pose of layers");
  last_pose_in_layer_label->setToolTip("Select every last pose of each layer");
  last_pose_in_layer_layout->addWidget(last_pose_in_layer_label);
  last_pose_in_layer_layout->addWidget(last_pose_in_layer_);

  layout_->addLayout(start_poses_layout);
  layout_->addLayout(end_poses_layout);
  layout_->addLayout(start_end_poses_layout);
  layout_->addLayout(last_pose_in_layer_layout);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  layout_->addStretch(1);
  layout_->addWidget(button_box_);
}

PoseInformationParametersFilter::~PoseInformationParametersFilter()
{
}

bool PoseInformationParametersFilter::getEndStatus()
{
  return end_poses_->isChecked();
}

bool PoseInformationParametersFilter::getStartEndStatus()
{
  return star_end_poses_->isChecked();
}

bool PoseInformationParametersFilter::getStartStatus()
{
  return start_poses_->isChecked();
}

bool PoseInformationParametersFilter::getLastPoseInLayerStatus()
{
  return last_pose_in_layer_->isChecked();
}

}
