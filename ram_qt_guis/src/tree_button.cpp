#include <ram_qt_guis/tree_button.hpp>

namespace ram_qt_guis
{

TreeButton::TreeButton(const QString text, QTreeWidget *parent, QTreeWidgetItem *item):
  QPushButton(text, parent),
  item_(item)
{
  setFlat(true);
  connect(this, &TreeButton::clicked, this, &TreeButton::pressed);
}

void TreeButton::pressed()
{
  if (item_)
    item_->setExpanded(!item_->isExpanded());
}

}
