#ifndef TRAFFICEVENT_HPP_
#define TRAFFICEVENT_HPP_


#include "Event.hpp"

typedef enum { Tenth = 10, Eleventh, Twelfth, Thirteenth, Fourteenth, Fifteenth } Street;

/**
 * @brief Structure for storing all the information for a vehicle.
 */
typedef struct {
  unsigned id; // ID of the vehicle.
  Street origin; // Originating street of the vehicle.
  Street destination; // Destination street of the vehicle.
  double entryTime; // Time at which the vehicle entered the area of interest.
  double exitTime; // Time at which the vehicle exited from the area of interest.
  double startWaiting; // Time at which the vehicle started waiting at the previous intersection.
  double endWaiting; // Time at which the vehicle passed the previous intersection.
  double totalWaiting; // Total waiting time for the vehicle;
	double currentPosition; // Indicate the current position of the vehicle.
} Vehicle;


/**
 * @brief Base class for all the traffic events.
 */
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
}; // class TrafficEvent


/**
 * @brief Class for intersection arrival event.
 */
class ArrivalEvent : public TrafficEvent {
public:
  ArrivalEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
}; // class ArrivalEvent

/**
 * @brief Class for intersection entry event.
 */
class EnteredEvent : public TrafficEvent {
public:
  EnteredEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
}; // class EnteredEvent

/**
 * @brief Class for intersection departure event.
 */
class DepartureEvent : public TrafficEvent {
public:
  DepartureEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process();
}; // class DepartureEvent

#endif // TRAFFICEVENT_HPP_
