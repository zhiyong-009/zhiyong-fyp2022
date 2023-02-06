#include <ram_qt_guis/modify_widgets/blend_radius_table.hpp>

namespace ram_qt_guis
{

BlendRadiusTable::BlendRadiusTable()
{
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &BlendRadiusTable::displayMessageBox, this, &BlendRadiusTable::displayMessageBoxHandler);

  table_ = new QTableWidget;
  layout->addWidget(table_);
  table_->setColumnCount(4);

  const QString alpha = QChar(0xb1, 0x03);
  const QString header_1 = alpha + " min";
  const QString header_2 = alpha + " max";
  const QString header_3 = "Mode";
  const QString header_4 = "Blend radius";
  const QStringList header = {header_1, header_2, header_3, header_4};

  table_->setHorizontalHeaderLabels(header);
  table_->resizeColumnsToContents();
  table_->setColumnWidth(0, 70);
  table_->setColumnWidth(1, 70);
  table_->setColumnWidth(2, 88);

  add_row_ = new QPushButton("Add row");
  connect(add_row_, &QPushButton::clicked, this, &BlendRadiusTable::addRow);
  remove_row_ = new QPushButton("Remove row");
  connect(remove_row_, &QPushButton::clicked, this, &BlendRadiusTable::removeRow);
  QHBoxLayout *buttons(new QHBoxLayout);
  buttons->addWidget(remove_row_);
  buttons->addWidget(add_row_);
  layout->addLayout(buttons);
  Q_EMIT addRow();
}

BlendRadiusTable::~BlendRadiusTable()
{
}

void BlendRadiusTable::fillRequest(ram_modify_trajectory::TweakBlendRadius::Request &req)
{
  // req.poses are not filled here
  ram_modify_trajectory::BlendRadiusModification row;
  for (int i(0); i < table_->rowCount(); ++i)
  {
    row.min_range = static_cast<QSpinBox*>(table_->cellWidget(i, 0))->value() * M_PI / 180.0; // degrees to radians
    row.max_range = static_cast<QSpinBox*>(table_->cellWidget(i, 1))->value() * M_PI / 180.0; // degrees to radians
    row.absolute = static_cast<QComboBox*>(table_->cellWidget(i, 2))->currentIndex();
    row.blend_radius_value = static_cast<QSpinBox*>(table_->cellWidget(i, 3))->value();
    req.new_blend_radius.emplace_back(row);
  }
}

void BlendRadiusTable::addRow()
{
  QTableWidget *t(table_);

  const int max_rows(8);
  if (t->rowCount() >= max_rows)
    return;

  if (t->rowCount() != 0)
  {
    QSpinBox *angle_max_last_row(static_cast<QSpinBox *>(t->cellWidget(t->rowCount() - 1, 1)));
    if (angle_max_last_row->value() >= 180)
    {
      Q_EMIT displayMessageBox("Cannot add row",
                               "To be able to add a row the maximum value of the last row must be < 180°",
                               "",
                               QMessageBox::Icon::Warning);
      return;
    }
  }

  remove_row_->setEnabled(true);
  t->setRowCount(t->rowCount() + 1);

  if (t->rowCount() >= max_rows)
    add_row_->setEnabled(false);
  if (t->rowCount() == 1)
    remove_row_->setEnabled(false);

  QSpinBox *angle_min(new QSpinBox);
  QSpinBox *angle_max(new QSpinBox);
  angle_min->setSuffix(" °");
  angle_max->setSuffix(angle_min->suffix());
  angle_min->setRange(0, 179);
  angle_max->setRange(1, 180);
  t->setCellWidget(t->rowCount() - 1, 0, angle_min);
  t->setCellWidget(t->rowCount() - 1, 1, angle_max);

  QComboBox *mode(new QComboBox);
  mode->addItem("Relative");
  mode->addItem("Absolute");
  t->setCellWidget(t->rowCount() - 1, 2, mode);

  QSpinBox *value(new QSpinBox);
  t->setCellWidget(t->rowCount() - 1, 3, value);

  connect(mode, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ](const int index)
  {
    if (index == 0)
      value->setRange(-100, 100);
    else
      value->setRange(0, 100);
  });
  Q_EMIT mode->currentIndexChanged(0);

  if (t->rowCount() == 1)
  {
    connect(angle_max, qOverload<int>(&QSpinBox::valueChanged), this, [ = ](const int v)
    {
      angle_min->setMaximum(v - 1);
    });
    angle_max->setValue(90);
    return;
  }

  angle_min->setRange(0, 180);
  angle_min->setReadOnly(true);
  QSpinBox *angle_max_prev_row(static_cast<QSpinBox *>(t->cellWidget(t->rowCount() - 2, 1)));
  connect(angle_max_prev_row, qOverload<int>(&QSpinBox::valueChanged), angle_min, &QSpinBox::setValue);
  connect(angle_min, qOverload<int>(&QSpinBox::valueChanged), this, [ = ](const int v)
  {
    if (v < 179)
      angle_max->setMinimum(v + 1);
    else
      angle_max->setMinimum(180);
  });

  connect(angle_max, qOverload<int>(&QSpinBox::valueChanged), this, [ = ](const int v)
  {
    angle_max_prev_row->setMaximum(v - 1);
  });

  Q_EMIT angle_max_prev_row->valueChanged(angle_max_prev_row->value());
}

void BlendRadiusTable::removeRow()
{
  QTableWidget *t(table_);
  if (t->rowCount() == 1)
    return;

  add_row_->setEnabled(true);
  t->setRowCount(t->rowCount() - 1);

  QSpinBox *angle_max_prev_row(static_cast<QSpinBox *>(t->cellWidget(t->rowCount() - 1, 1)));
  angle_max_prev_row->setMaximum(180);

  if (t->rowCount() == 1)
    remove_row_->setEnabled(false);
}

void BlendRadiusTable::displayMessageBoxHandler(const QString title,
    const QString text,
    const QString info,
    const QMessageBox::Icon icon)
{
  const bool old_state(isEnabled());
  setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(text);
  msg_box.setInformativeText(info);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  setEnabled(old_state);
}


}
