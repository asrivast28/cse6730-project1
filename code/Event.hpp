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
 * @brief Class for storing event data.
 */
class EventData {
public:
  enum
  EventType {
    ARRIVAL,
    ENTERED,
    DEPARTURE
  };

public:
  EventData(
    const Vehicle&,
    const EventType,
    const bool = false
  );

  const Vehicle&
  vehicle() const;

  bool
  continued() const;

  ~EventData();

private:
  Vehicle m_vehicle;
	EventType m_eventType;
  bool m_continued;
};

/**
 * @brief Class containing simulation event attributes.
 */
class Event {
public:
  Event(
    const double,
    void (*)(const EventData&),
    const EventData& 
  );

  Event(
    const Event&
  );

  void
  callback() const;

  double
  timestamp() const;

  ~Event();

  friend
  bool
  operator<(
    const Event&,
    const Event&
  );

private:
	double m_timestamp;					// event time stamp
	void (*m_callback) (const EventData&);		// handler callback
  EventData m_eventData;						// application data
};
