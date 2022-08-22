#ifndef VIEWPORT_MOUSE_EVENT_H
#define VIEWPORT_MOUSE_EVENT_H

#include <QMouseEvent>
#include <QWheelEvent>
#include <QWindow>

namespace Ogre
{
class Viewport;
}

namespace rviz
{

class ViewportMouseEvent
{
 public:
  ViewportMouseEvent() {}

  /** Constructor for use with a QMouseEvent. */
  ViewportMouseEvent(Ogre::Viewport* vp, QMouseEvent* e, int lx, int ly)
  : viewport( vp )
  , type( e->type() )
  , x( e->x() )
  , y( e->y() )
  , wheel_delta( 0 )
  , acting_button( e->button() )
  , buttons_down( e->buttons() )
  , modifiers( e->modifiers() )
  , last_x( lx )
  , last_y( ly )
  {
  }

  // Qt has a separate QWheelEvent for mousewheel events which is not
  // a subclass of QMouseEvent, but has a lot of overlap with it.

  /** Constructor for use with a QWheelEvent. */
  ViewportMouseEvent(Ogre::Viewport* vp, QWheelEvent* e, int lx, int ly)
  : viewport( vp )
  , type( e->type() )
  , x( e->x() )
  , y( e->y() )
  , wheel_delta( e->delta() )
  , acting_button( Qt::NoButton )
  , buttons_down( e->buttons() )
  , modifiers( e->modifiers() )
  , last_x( lx )
  , last_y( ly )
  {
  }

  // Convenience functions for getting the state of the buttons and
  // modifiers at the time of the event.  For the button which caused
  // a press or release event, use acting_button.
  bool left() { return buttons_down & Qt::LeftButton; }
  bool middle() { return buttons_down & Qt::MidButton; }
  bool right() { return buttons_down & Qt::RightButton; }

  bool shift() { return modifiers & Qt::ShiftModifier; }
  bool control() { return modifiers & Qt::ControlModifier; }
  bool alt() { return modifiers & Qt::AltModifier; }

  // Convenience functions to tell if the event is a mouse-down or
  // mouse-up event and which button caused it.
  bool leftUp() { return type == QEvent::MouseButtonRelease && acting_button == Qt::LeftButton; }
  bool middleUp() { return type == QEvent::MouseButtonRelease && acting_button == Qt::MidButton; }
  bool rightUp() { return type == QEvent::MouseButtonRelease && acting_button == Qt::RightButton; }

  bool leftDown() { return type == QEvent::MouseButtonPress && acting_button == Qt::LeftButton; }
  bool middleDown() { return type == QEvent::MouseButtonPress && acting_button == Qt::MidButton; }
  bool rightDown() { return type == QEvent::MouseButtonPress && acting_button == Qt::RightButton; }
  Ogre::Viewport* viewport;
  QEvent::Type type;
  int x;
  int y;
  int wheel_delta;
  Qt::MouseButton acting_button; // The button which caused the event.  Can be Qt::NoButton (move or wheel events).
  Qt::MouseButtons buttons_down;
  Qt::KeyboardModifiers modifiers;
  int last_x;
  int last_y;
};

} // namespace rviz

#endif // VIEWPORT_MOUSE_EVENT_H
