#ifndef EVENT_HPP_
#define EVENT_HPP_

#include <cmath>
#include <memory>

class Simulation;

/**
 * @brief Class containing simulation event attributes.
 */
class Event {
public:
  Event(
    const double
  );

  Event(
    const Event&
  );

  virtual
  double
  timestamp() const;

  virtual
  void
  process(
    Simulation&
  ) = 0;

  virtual
  ~Event();

private:
	double m_timestamp;					// event time stamp
}; // class Event

class EventComparator {
public:
  bool
  operator()(
    const std::shared_ptr<Event>& e1,
    const std::shared_ptr<Event>& e2
  )
  {
    return std::isgreater(e1->timestamp(), e2->timestamp());
  }
}; // class EventComparator

#endif // EVENT_HPP_
