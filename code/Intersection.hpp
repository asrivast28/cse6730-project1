#ifndef INTERSECTION_HPP_
#define INTERSECTION_HPP_

#include "TrafficEvent.hpp"

#include <cstdlib>
#include <queue>
#include <vector>


class TrafficParameters;

class Intersection {
public:
  enum
  SignalState {
    GREEN_THRU,
    GREEN_LEFT,
    YELLOW,
    RED
  };

  Intersection();

public:
  void
  addToQueue(
    const Vehicle&
  );

  unsigned
  queueSize(
    const Vehicle&
  ) const;

  const Vehicle&
  viewFrontVehicle(
    const Vehicle&
  ) const;

  Vehicle
  getFrontVehicle(
    const Vehicle&
  );

  void
  increaseGroupSize(
    const Vehicle&
  );

  void
  decreaseGroupSize(
    const Vehicle&
  );

  unsigned
  groupSize(
    const Vehicle&
  ) const;

  Intersection::SignalState
  getSignalState(
    const Vehicle&,
    const double,
    const TrafficParameters&
  );

public:
  static
  void
  setOccupied(
    const Vehicle&
  );

  static
  void
  setClear(
    const Vehicle&
  );

  static
  bool
  isClear(
    const Vehicle&
  );

private:
  std::vector<std::queue<Vehicle> > m_queue;
  std::vector<unsigned> m_groupSize;

private:
  static std::vector<bool> m_occupied;
}; // class IntersectionState

#endif // INTERSECTION_HPP_
