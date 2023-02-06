#ifndef RAM_QT_GUIS_MODIFY_WIDGETS_BLEND_RADIUS_TABLE
#define RAM_QT_GUIS_MODIFY_WIDGETS_BLEND_RADIUS_TABLE

#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <ram_modify_trajectory/TweakBlendRadius.h>

namespace ram_qt_guis
{

class BlendRadiusTable : public QWidget
{
  Q_OBJECT

public:
  BlendRadiusTable();
  ~BlendRadiusTable();
  void fillRequest(ram_modify_trajectory::TweakBlendRadius::Request &req);

Q_SIGNALS:
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void addRow();
  void removeRow();
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

private:
  QTableWidget *table_;
  QPushButton *add_row_;
  QPushButton *remove_row_;
};


}

#endif
