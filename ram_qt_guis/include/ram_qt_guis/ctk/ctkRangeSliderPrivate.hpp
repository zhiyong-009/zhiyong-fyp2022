#ifndef _ctkRangeSliderPrivate_HPP
#define _ctkRangeSliderPrivate_HPP

// CTK include
#include "ram_qt_guis/ctk/ctkRangeSlider.hpp"

class ctkRangeSlider;

class ctkRangeSliderPrivate
{
  Q_DECLARE_PUBLIC(ctkRangeSlider);
protected:
  ctkRangeSlider* const q_ptr;
public:
  /// Boolean indicates the selected handle
  ///   True for the minimum range handle, false for the maximum range handle
  enum Handle {
    NoHandle = 0x0,
    MinimumHandle = 0x1,
    MaximumHandle = 0x2
  };
  Q_DECLARE_FLAGS(Handles, Handle);

  ctkRangeSliderPrivate(ctkRangeSlider& object);
  void init();

  /// Return the handle at the given pos, or none if no handle is at the pos.
  /// If a handle is selected, handleRect is set to the handle rect.
  /// otherwise return NoHandle and handleRect is set to the combined rect of
  /// the min and max handles
  Handle handleAtPos(const QPoint& pos, QRect& handleRect)const;

  /// Copied verbatim from QSliderPrivate class (see QSlider.cpp)
  int pixelPosToRangeValue(int pos) const;
  int pixelPosFromRangeValue(int val) const;

  /// Draw the bottom and top sliders.
  void drawMinimumSlider( QStylePainter* painter ) const;
  void drawMaximumSlider( QStylePainter* painter ) const;

  /// End points of the range on the Model
  int m_MaximumValue;
  int m_MinimumValue;

  /// End points of the range on the GUI. This is synced with the model.
  int m_MaximumPosition;
  int m_MinimumPosition;

  /// Controls selected ?
  QStyle::SubControl m_MinimumSliderSelected;
  QStyle::SubControl m_MaximumSliderSelected;

  /// See QSliderPrivate::clickOffset.
  /// Overrides this ivar
  int m_SubclassClickOffset;

  /// See QSliderPrivate::position
  /// Overrides this ivar.
  int m_SubclassPosition;

  /// Original width between the 2 bounds before any moves
  float m_SubclassWidth;

  ctkRangeSliderPrivate::Handles m_SelectedHandles;

  /// When symmetricMoves is true, moving a handle will move the other handle
  /// symmetrically, otherwise the handles are independent.
  bool m_SymmetricMoves;

  QString m_HandleToolTip;

private:
  Q_DISABLE_COPY(ctkRangeSliderPrivate);
};

#endif
