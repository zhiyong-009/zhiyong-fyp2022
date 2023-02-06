//! Copyright B2A Technology
//! This file and its content are the property of B2A Technology.
//! Any reproduction or use outside B2A Technology is prohibited.
//!
//! @file ctkRangeSliderPrivate.cpp

// CTK include
#include "ram_qt_guis/ctk/ctkRangeSliderPrivate.hpp"

// Qt include
#include <QStyleOptionSlider>

// --------------------------------------------------------------------------
ctkRangeSliderPrivate::ctkRangeSliderPrivate(ctkRangeSlider& object)
  :q_ptr(&object)
{
  this->m_MinimumValue = 0;
  this->m_MaximumValue = 100;
  this->m_MinimumPosition = 0;
  this->m_MaximumPosition = 100;
  this->m_MinimumSliderSelected = QStyle::SC_None;
  this->m_MaximumSliderSelected = QStyle::SC_None;
  this->m_SubclassClickOffset = 0;
  this->m_SubclassPosition = 0;
  this->m_SubclassWidth = 0.0;
  this->m_SelectedHandles = 0;
  this->m_SymmetricMoves = false;
}

// --------------------------------------------------------------------------
void ctkRangeSliderPrivate::init()
{
  Q_Q(ctkRangeSlider);
  this->m_MinimumValue = q->minimum();
  this->m_MaximumValue = q->maximum();
  this->m_MinimumPosition = q->minimum();
  this->m_MaximumPosition = q->maximum();
  q->connect(q, SIGNAL(rangeChanged(int,int)), q, SLOT(onRangeChanged(int,int)));
}

// --------------------------------------------------------------------------
ctkRangeSliderPrivate::Handle ctkRangeSliderPrivate::handleAtPos(const QPoint& pos, QRect& handleRect)const
{
  Q_Q(const ctkRangeSlider);

  QStyleOptionSlider option;
  q->initStyleOption( &option );

  // The functinos hitTestComplexControl only know about 1 handle. As we have
  // 2, we change the position of the handle and test if the pos correspond to
  // any of the 2 positions.

  // Test the MinimumHandle
  option.sliderPosition = this->m_MinimumPosition;
  option.sliderValue    = this->m_MinimumValue;

  QStyle::SubControl minimumControl = q->style()->hitTestComplexControl(
    QStyle::CC_Slider, &option, pos, q);
  QRect minimumHandleRect = q->style()->subControlRect(
      QStyle::CC_Slider, &option, QStyle::SC_SliderHandle, q);

  // Test if the pos is under the Maximum handle
  option.sliderPosition = this->m_MaximumPosition;
  option.sliderValue    = this->m_MaximumValue;

  QStyle::SubControl maximumControl = q->style()->hitTestComplexControl(
    QStyle::CC_Slider, &option, pos, q);
  QRect maximumHandleRect = q->style()->subControlRect(
      QStyle::CC_Slider, &option, QStyle::SC_SliderHandle, q);

  // The pos is above both handles, select the closest handle
  if (minimumControl == QStyle::SC_SliderHandle &&
      maximumControl == QStyle::SC_SliderHandle)
    {
    int minDist = 0;
    int maxDist = 0;
    if (q->orientation() == Qt::Horizontal)
      {
      minDist = pos.x() - minimumHandleRect.left();
      maxDist = maximumHandleRect.right() - pos.x();
      }
    else //if (q->orientation() == Qt::Vertical)
      {
      minDist = minimumHandleRect.bottom() - pos.y();
      maxDist = pos.y() - maximumHandleRect.top();
      }
    Q_ASSERT( minDist >= 0 && maxDist >= 0);
    minimumControl = minDist < maxDist ? minimumControl : QStyle::SC_None;
    }

  if (minimumControl == QStyle::SC_SliderHandle)
    {
    handleRect = minimumHandleRect;
    return MinimumHandle;
    }
  else if (maximumControl == QStyle::SC_SliderHandle)
    {
    handleRect = maximumHandleRect;
    return MaximumHandle;
    }
  handleRect = minimumHandleRect.united(maximumHandleRect);
  return NoHandle;
}

// --------------------------------------------------------------------------
// Copied verbatim from QSliderPrivate::pixelPosToRangeValue. See QSlider.cpp
//
int ctkRangeSliderPrivate::pixelPosToRangeValue( int pos ) const
{
  Q_Q(const ctkRangeSlider);
  QStyleOptionSlider option;
  q->initStyleOption( &option );

  QRect gr = q->style()->subControlRect( QStyle::CC_Slider,
                                            &option,
                                            QStyle::SC_SliderGroove,
                                            q );
  QRect sr = q->style()->subControlRect( QStyle::CC_Slider,
                                            &option,
                                            QStyle::SC_SliderHandle,
                                            q );
  int sliderMin, sliderMax, sliderLength;
  if (option.orientation == Qt::Horizontal)
    {
    sliderLength = sr.width();
    sliderMin = gr.x();
    sliderMax = gr.right() - sliderLength + 1;
    }
  else
    {
    sliderLength = sr.height();
    sliderMin = gr.y();
    sliderMax = gr.bottom() - sliderLength + 1;
    }

  return QStyle::sliderValueFromPosition( q->minimum(),
                                          q->maximum(),
                                          pos - sliderMin,
                                          sliderMax - sliderMin,
                                          option.upsideDown );
}

//---------------------------------------------------------------------------
int ctkRangeSliderPrivate::pixelPosFromRangeValue( int val ) const
{
  Q_Q(const ctkRangeSlider);
  QStyleOptionSlider option;
  q->initStyleOption( &option );

  QRect gr = q->style()->subControlRect( QStyle::CC_Slider,
                                            &option,
                                            QStyle::SC_SliderGroove,
                                            q );
  QRect sr = q->style()->subControlRect( QStyle::CC_Slider,
                                            &option,
                                            QStyle::SC_SliderHandle,
                                            q );
  int sliderMin, sliderMax, sliderLength;
  if (option.orientation == Qt::Horizontal)
    {
    sliderLength = sr.width();
    sliderMin = gr.x();
    sliderMax = gr.right() - sliderLength + 1;
    }
  else
    {
    sliderLength = sr.height();
    sliderMin = gr.y();
    sliderMax = gr.bottom() - sliderLength + 1;
    }

  return QStyle::sliderPositionFromValue( q->minimum(),
                                          q->maximum(),
                                          val,
                                          sliderMax - sliderMin,
                                          option.upsideDown ) + sliderMin;
}

//---------------------------------------------------------------------------
// Draw slider at the bottom end of the range
void ctkRangeSliderPrivate::drawMinimumSlider( QStylePainter* painter ) const
{
  Q_Q(const ctkRangeSlider);
  QStyleOptionSlider option;
  q->initMinimumSliderStyleOption( &option );

  option.subControls = QStyle::SC_SliderHandle;
  option.sliderValue = m_MinimumValue;
  option.sliderPosition = m_MinimumPosition;
  if (q->isMinimumSliderDown())
    {
    option.activeSubControls = QStyle::SC_SliderHandle;
    option.state |= QStyle::State_Sunken;
    }
#ifdef Q_OS_MAC
  // On mac style, drawing just the handle actually draws also the groove.
  QRect clip = q->style()->subControlRect(QStyle::CC_Slider, &option,
                                          QStyle::SC_SliderHandle, q);
  painter->setClipRect(clip);
#endif
  painter->drawComplexControl(QStyle::CC_Slider, option);
}

//---------------------------------------------------------------------------
// Draw slider at the top end of the range
void ctkRangeSliderPrivate::drawMaximumSlider( QStylePainter* painter ) const
{
  Q_Q(const ctkRangeSlider);
  QStyleOptionSlider option;
  q->initMaximumSliderStyleOption( &option );

  option.subControls = QStyle::SC_SliderHandle;
  option.sliderValue = m_MaximumValue;
  option.sliderPosition = m_MaximumPosition;
  if (q->isMaximumSliderDown())
    {
    option.activeSubControls = QStyle::SC_SliderHandle;
    option.state |= QStyle::State_Sunken;
    }
#ifdef Q_OS_MAC
  // On mac style, drawing just the handle actually draws also the groove.
  QRect clip = q->style()->subControlRect(QStyle::CC_Slider, &option,
                                          QStyle::SC_SliderHandle, q);
  painter->setClipRect(clip);
#endif
  painter->drawComplexControl(QStyle::CC_Slider, option);
}