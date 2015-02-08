#ifndef TRAFFICEVENT_HPP_
#define TRAFFICEVENT_HPP_


#include "Event.hpp"

class TrafficEvent : public Event {
public:
  TrafficEvent(
    const double,
    const Vehicle&,
    const bool
  );

  virtual
  ~TrafficEvent() = 0;

protected:
  const Vehicle m_vehicle;
  const bool m_continued;
};


class ArrivalEvent : public TrafficEvent {
public:
  ArrivalEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
};

class EnteredEvent : public TrafficEvent {
public:
  EnteredEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
};

class DepartureEvent : public TrafficEvent {
public:
  DepartureEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
};

#endif // TRAFFICEVENT_HPP_
