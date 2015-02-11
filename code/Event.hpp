#ifndef EVENT_HPP_
#define EVENT_HPP_

#include <memory>

typedef enum { Tenth = 10, Eleventh, Twelfth, Thirteenth, Fourteenth, Fifteenth } Street;

/**
 * @brief Structure for storing all the information for a vehicle.
 */
typedef struct {
  unsigned id; // ID of the vehicle.
  Street origin; // Originating street of the vehicle.
  Street destination; // Destination street of the vehicle.
  unsigned entryTime; // Time at which the vehicle entered the area of interest.
  unsigned exitTime; // Time at which the vehicle exited from the area of interest.
  double startWaiting; // Time at which the vehicle started waiting at the previous intersection.
  double endWaiting; // Time at which the vehicle passed the previous intersection.
  double totalWaiting; // Total waiting time for the vehicle;
	double currentPosition; // Indicate the current position of the vehicle.
} Vehicle;

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
