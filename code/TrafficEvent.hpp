#ifndef TRAFFICEVENT_HPP_
#define TRAFFICEVENT_HPP_


#include "Event.hpp"

typedef enum { Tenth = 10, Eleventh, Twelfth, Thirteenth, Fourteenth, Fifteenth } Street;
typedef enum { Left_turn, Right_turn, GO_THROUGH} TurnDirection;

/**
 * @brief Structure for storing all the information for a vehicle.
 */
struct Vehicle {
  unsigned id; // ID of the vehicle.
  Street origin; // Originating street of the vehicle.
  Street destination; // Destination street of the vehicle.
  Street position; // Indicate the current position of the vehicle.
  double entryTime; // Time at which the vehicle crossed the area of interest.
  double exitTime; // Time at which the vehicle exited from the area of interest.
  double waitingSince; // Time at which the vehicle started waiting.
  double totalWaiting; // Total waiting time for the vehicle.
  TurnDirection turndirection;
};


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
  process(
    Simulation&
  );
}; // class ArrivalEvent

/**
 * @brief Class for intersection entry event.
 */
class CrossedEvent : public TrafficEvent {
public:
  CrossedEvent(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process(
    Simulation&
  );
}; // class CrossedEvent

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
  process(
    Simulation&
  );
}; // class DepartureEvent

/**
 * @brief Class for intersection arrival event.
 */
class ArrivalEventLeft : public TrafficEvent {
public:
  ArrivalEventLeft(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process(
    Simulation&
  );
}; // class ArrivalEventLeft

/**
 * @brief Class for intersection entry event.
 */
class CrossedEventLeft : public TrafficEvent {
public:
  CrossedEventLeft(
    const double,
    const Vehicle&,
    const bool = false
  );

  void
  process(
    Simulation&
  );
}; // class CrossedEventLeft

#endif // TRAFFICEVENT_HPP_
