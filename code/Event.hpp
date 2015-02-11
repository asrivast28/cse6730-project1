#ifndef EVENT_HPP_
#define EVENT_HPP_

#include <memory>

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
  process() = 0;

  virtual
  ~Event();

private:
	double m_timestamp;					// event time stamp
};

class EventComparator {
public:
  bool
  operator()(
    const std::shared_ptr<Event>& e1,
    const std::shared_ptr<Event>& e2
  )
  {
    return (e1->timestamp() < e2->timestamp());
  }

};

#endif // EVENT_HPP_
