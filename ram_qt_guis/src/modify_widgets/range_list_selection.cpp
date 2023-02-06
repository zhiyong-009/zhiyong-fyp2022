#include <ram_qt_guis/modify_widgets/range_list_selection.hpp>

namespace ram_qt_guis
{
ModifyRangeListSelection::ModifyRangeListSelection(QVBoxLayout* layout,
                                                   const QString help_string,
                                                   const unsigned min_val,
                                                   const unsigned max_val,
                                                   std::vector<unsigned> locked) :
        layout_(layout),
        help_string_(help_string),
        min_val_(min_val),
        max_val_(max_val)
{
  setObjectName("ModifyRangeListSelection");
  layout_->addWidget(new QLabel(help_string_));

  QTabWidget *tab_widget = new QTabWidget;
  layout_->addWidget(tab_widget);

  QHBoxLayout *all_none_buttons = new QHBoxLayout;
  all_button_ = new QPushButton("All");
  none_button_ = new QPushButton("None");
  all_none_buttons->addWidget(all_button_);
  all_none_buttons->addWidget(none_button_);
  layout_->addLayout(all_none_buttons);

  QHBoxLayout *pair_odd_buttons(new QHBoxLayout);
  pair_ = new QPushButton("Pair");
  pair_->setToolTip("Selects all elements with a pair number. Deselects all other elements");
  odd_ = new QPushButton("Odd");
  odd_->setToolTip("Selects all elements with an odd number. Deselects all other elements");
  pair_odd_buttons->addWidget(pair_);
  pair_odd_buttons->addWidget(odd_);
  layout_->addLayout(pair_odd_buttons);

  connect(pair_, &QPushButton::clicked, this, [ = ]()
  {
    bool pair(true); // First element is pair
    for (auto &c : checkboxes_)
    {
      c->blockSignals(true);
      if (c->isEnabled() && pair)
        c->setChecked(true);
      else if (c->isEnabled())
        c->setChecked(false);

      pair = !pair;
    }

    for (auto &c: checkboxes_)
      c->blockSignals(false);
    Q_EMIT updateSelectionFromTicks();
  });

  connect(odd_, &QPushButton::clicked, this, [ = ]()
  {
    bool pair(true); // First element is pair
    for (auto &c : checkboxes_)
    {
      c->blockSignals(true);
      if (c->isEnabled() && pair)
        c->setChecked(false);
      else if (c->isEnabled())
        c->setChecked(true);

      pair = !pair;
    }

    for (auto &c: checkboxes_)
      c->blockSignals(false);
    Q_EMIT updateSelectionFromTicks();
  });

  QWidget *range_select = new QWidget;
  QVBoxLayout *range_select_layout = new QVBoxLayout(range_select);
  range_select->setLayout(range_select_layout);

  range_select_layout->addWidget(new QLabel("Select first and last:"));
  min_box_ = new QSpinBox;
  min_box_->setRange(min_val_, max_val_);
  max_box_ = new QSpinBox;
  max_box_->setRange(min_val_, max_val_);
  connect(min_box_, qOverload<int>(&QSpinBox::valueChanged), this, &ModifyRangeListSelection::tweakRangeMin);
  connect(max_box_, qOverload<int>(&QSpinBox::valueChanged), this, &ModifyRangeListSelection::tweakRangeMax);

  QHBoxLayout *min_box_layout = new QHBoxLayout;
  min_box_layout->addWidget(new QLabel("First"));
  min_box_layout->addWidget(min_box_);
  QHBoxLayout *max_box_layout = new QHBoxLayout;
  max_box_layout->addWidget(new QLabel("Last"));
  max_box_layout->addWidget(max_box_);

  range_select_layout->addLayout(min_box_layout);
  range_select_layout->addLayout(max_box_layout);

  QHBoxLayout *add_remove_invert_buttons = new QHBoxLayout;
  add_button_ = new QPushButton("Add");
  remove_button_ = new QPushButton("Remove");
  invert_button_ = new QPushButton("Invert");
  add_remove_invert_buttons->addWidget(add_button_);
  add_remove_invert_buttons->addWidget(remove_button_);
  add_remove_invert_buttons->addWidget(invert_button_);

  range_select_layout->addLayout(add_remove_invert_buttons);
  range_select_layout->addStretch(1);

  QWidget *scroll_widget = new QWidget;
  QVBoxLayout *checkboxes_layout = new QVBoxLayout;
  scroll_widget->setLayout(checkboxes_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  tab_widget->addTab(scroll_area, "Check-boxes");
  tab_widget->addTab(range_select, "Range select");

  for (unsigned i(min_val_); i <= max_val_; ++i)
  {
    QCheckBox *checkbox = new QCheckBox;
    checkbox->setText(QString::fromStdString(std::to_string(i)));
    checkboxes_.push_back(checkbox);
  }

  for (auto checkbox : checkboxes_)
  {
    checkboxes_layout->addWidget(checkbox);
    connect(checkbox, qOverload<int>(&QCheckBox::stateChanged), this, &ModifyRangeListSelection::updateSelectionFromTicks);
  }

  selection_.resize(checkboxes_.size());

  if (!locked.empty())
  {
    std::sort(locked.begin(), locked.end());
    if (locked.back() < checkboxes_.size())
    {
      for (unsigned i(0); i < locked.size(); ++i)
      {
        checkboxes_[locked[i]]->setEnabled(false);
        checkboxes_[locked[i]]->setChecked(true);
      }
    }
    updateSelectionFromTicks();
  }

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(1);
  layout_->addWidget(button_box_);

  connect(add_button_, &QPushButton::clicked, this, &ModifyRangeListSelection::addButton);
  connect(remove_button_, &QPushButton::clicked, this, &ModifyRangeListSelection::removeButton);
  connect(invert_button_, &QPushButton::clicked, this, &ModifyRangeListSelection::invertButton);
  connect(all_button_, &QPushButton::clicked, this, &ModifyRangeListSelection::selectAll);
  connect(none_button_, &QPushButton::clicked, this, &ModifyRangeListSelection::selectNone);
}

ModifyRangeListSelection::~ModifyRangeListSelection()
{
}

std::vector<unsigned>
ModifyRangeListSelection::getSelection()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);
  std::vector<unsigned> selection;

  for (unsigned i(0); i < selection_.size(); ++i)
    if (selection_[i] == true)
      selection.push_back(i);

  return selection;
}

void
ModifyRangeListSelection::updateTicksFromSelection()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  Q_EMIT layout_->parentWidget()->setEnabled(false);

  // Block signals from check boxes to prevent nested infinite calls
  std::vector<QSignalBlocker> blockers;
  for (auto checkbox : checkboxes_)
    blockers.push_back(QSignalBlocker(checkbox));

  for (unsigned i(0); i < checkboxes_.size(); ++i)
    checkboxes_[i]->setChecked(selection_[i]);

  Q_EMIT layout_->parentWidget()->setEnabled(true);
  Q_EMIT selectionChanged(getSelection());
}

void ModifyRangeListSelection::tweakRangeMin()
{
  if (min_box_->value() > max_box_->value())
    max_box_->setValue(min_box_->value());
}

void ModifyRangeListSelection::tweakRangeMax()
{
  if (min_box_->value() > max_box_->value())
    min_box_->setValue(max_box_->value());
}

void
ModifyRangeListSelection::updateSelectionFromTicks()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  Q_EMIT layout_->parentWidget()->setEnabled(false);

  for (unsigned i(0); i < selection_.size(); ++i)
    selection_[i] = checkboxes_[i]->isChecked();

  Q_EMIT layout_->parentWidget()->setEnabled(true);
  Q_EMIT selectionChanged(getSelection());
}

void
ModifyRangeListSelection::addButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      selection_[i] = true;
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::removeButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      if (checkboxes_[i]->isEnabled())
        selection_[i] = false;
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::invertButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      if (checkboxes_[i]->isEnabled())
        selection_[i] = !selection_[i];
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::selectAll()
{
  for (auto &c : checkboxes_)
  {
    if (c->isEnabled())
    {
      c->blockSignals(true);
      c->setChecked(true);
      c->blockSignals(false);
    }
  }

  // Update selection
  Q_EMIT updateSelectionFromTicks();
}

void
ModifyRangeListSelection::selectNone()
{
  for (auto &c : checkboxes_)
  {
    if (c->isEnabled())
    {
      c->blockSignals(true);
      c->setChecked(false);
      c->blockSignals(false);
    }
  }

  // Update selection
  Q_EMIT updateSelectionFromTicks();
}

bool
ModifyRangeListSelection::checkRange()
{
  if (min_box_->value() > max_box_->value())
  {
    QMessageBox::warning(this, ("Wrong range"),
                         ("Wrong selection, the \"last\" value must be superior or equal to the \"first\" value."),
                         QMessageBox::Ok);
    return false;
  }

  return true;
}

}

