#ifndef RAM_QT_GUIS_MODIFY_WIDGETS_RANGE_LIST_SELECTION_HPP
#define RAM_QT_GUIS_MODIFY_WIDGETS_RANGE_LIST_SELECTION_HPP

#include <mutex>

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace ram_qt_guis
{
class ModifyRangeListSelection : public QWidget
{
Q_OBJECT
  public:
  ModifyRangeListSelection(QVBoxLayout* layout,
                           const QString help_string = "",
                           const unsigned min_val = 0,
                           const unsigned max_val = 0,
                           std::vector<unsigned> locked = std::vector<unsigned>());

  virtual ~ModifyRangeListSelection();
  void updateTicksFromSelection();
  std::vector<unsigned> getSelection();
  QDialogButtonBox *button_box_;

Q_SIGNALS:
  void selectionChanged(std::vector<unsigned> selection);

protected Q_SLOTS:
  void tweakRangeMin();
  void tweakRangeMax();
  void updateSelectionFromTicks();
  void addButton();
  void removeButton();
  void invertButton();
  void selectAll();
  void selectNone();

private:
  bool checkRange();

  QVBoxLayout *layout_;
  QSpinBox *min_box_;
  QSpinBox *max_box_;
  QPushButton *add_button_;
  QPushButton *remove_button_;
  QPushButton *invert_button_;
  QPushButton *pair_;
  QPushButton *odd_;
  QPushButton *all_button_;
  QPushButton *none_button_;

  const QString help_string_;
  const unsigned min_val_;
  const unsigned max_val_;
  std::recursive_mutex selection_mutex_;
  std::vector<unsigned> selection_;
  std::vector<QCheckBox *> checkboxes_;
};

}

#endif
