#ifndef RAM_QT_GUIS_TREE_BUTTON
#define RAM_QT_GUIS_TREE_BUTTON

#include <QPushButton>
#include <QTreeWidgetItem>

namespace ram_qt_guis
{

class TreeButton : public QPushButton
{
  Q_OBJECT

public:
  TreeButton(const QString text, QTreeWidget *parent,
             QTreeWidgetItem *item);

private Q_SLOTS:
  void pressed();

private:
  QTreeWidgetItem *item_;
};

}

#endif
