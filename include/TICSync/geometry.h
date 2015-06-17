/* 
 * copyright {
 * TICSync - A Clock Synchronization Library
 * Copyright 2012 Isis Innovation Limited
 * University of Oxford
 * 
 * Authors: Alastair Harrison
 *          arh@robots.ox.ac.uk
 *
 * See included LICENSE file for license details
 * }
 */

#ifndef __geometry_h
#define __geometry_h

#include "TICSync/Core.h"
#include "TICSync/dataTypes.h"

namespace TICSync
{

/**
 * Useful geometry operations that are used in computing
 * the convex hulls
 */
template <class T>
class geom
{
public:

  static bool leftOn(const Point<T>& a, const Point<T>& b, const Point<T>& c)
  {
    T crossProdMag((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y()));
    return crossProdMag >= 0.0;
  }

  static bool leftOn(const Line<T>& seg, const Point<T>& c)
  {
    return leftOn(seg.p1, seg.p2, c);
  }

  static bool rightOn(const Point<T>& a, const Point<T>& b, const Point<T>& c)
  {
    T crossProdMag((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y()));
    return crossProdMag <= 0.0;
  }

  static bool rightOn(const Line<T>& seg, const Point<T>& c)
  {
    return rightOn(seg.p1, seg.p2, c);
  }

};

} // TICSync


#endif // __geometry_h
