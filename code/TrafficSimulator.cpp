#include "ProgramOptions.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <queue>

#define SIMULATION_TIME 14400
#define NB_INTER_ARRIVAL_TIME 20

#define INTERSECTION_CROSS_TIME 1
#define ROAD_TRAVEL_TIME 3

typedef enum { Tenth = 10, Eleventh, Twelfth, Thirteenth, Fourteenth, Fifteenth } Street;

/*
 * Generate a uniform random number in the range [0,1)
 */
double urand(void) {
	double x;
	while ((x = (rand() / (double) RAND_MAX)) >= 1.0);		// loop until x < 1
	return x;
}

/*
 * Generate a random number from an exponential distribution
 * with a given mean.
 *
 * @param mean the mean of the distribution
 *
 * @return a number draw from exponential distribution
 */
double randexp(double mean) {
	return (-1 * mean) * (log(1.0 - urand()));
}

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
    Vehicle* const,
    const EventType,
    const bool = false
  );

  const Vehicle*
  vehicle() const;

  bool
  continued() const;

private:
  std::unique_ptr<Vehicle> m_vehicle;
	EventType m_eventType;
  bool m_continued;
};

EventData::EventData(
  Vehicle* const vehicle,
  const EventType eventType,
  const bool continued
) : m_vehicle(vehicle),
  m_eventType(eventType),
  m_continued(continued)
{
}

const Vehicle*
EventData::vehicle(
) const
{
  return m_vehicle.get();
}

bool
EventData::continued(
) const
{
  return m_continued;
}

/**
 * @brief Class containing simulation event attributes. 
 */
class Event {
public:
  Event(
    const double,
    void (*)(EventData*),
    EventData* const
  );

  void
  callback() const;

  double
  timestamp() const;
    
  friend
  bool
  operator<(
    const Event&,
    const Event&
  );

private:
	double m_timestamp;					// event time stamp
	void (*m_callback) (const EventData* const);		// handler callback
  std::unique_ptr<EventData> m_eventData;						// application data
};

Event::Event(
  const double timestamp,
  void (*callback)(const EventData* const),
  EventData* const eventData 
) : m_timestamp(timestamp),
  m_callback(callback),
  m_eventData(eventData)
{
}

Event::timestamp(
) const
{
  return m_timestamp;
}

void
Event::callback(
) const
{
  m_callback(m_eventData.get());
}

bool
operator<(
  const Event& e1,
  const Event& e2
)
{
  return (e1.m_timestamp < e2.m_timestamp);
}


double simtime = 0;
std::priority_queue<Event> FEL;
std::queue<Vehicle> STATS_nbCrossed;
std::queue<Vehicle*> north_q;

// schedule the event at time-stamp, and provide a callback to its handler
void
schedule(
  double timestamp,
  EventData* const eventData,
  void (*callback) (const EventData* const)
)
{
  FEL.push(Event(timestamp, callback, eventData));
}

// returns the current simulation time
double current_time() {
	return simtime;
}

class Intersection {
public:
  enum
  SignalState {
    GREEN_THRU,
    GREEN_LEFT,
    YELLOW,
    RED
  };

public:
  void
  updateSignalStates();

  Intersection::SignalState
  getSignalState(
    const Vehicle* const 
  ) const;

private:
  SignalState[Street::Fifteenth] m_states;
}; // class IntersectionState

void
Intersection::updateSignalStates(
)
{
	double timer = current_time();
	int segement = timer / 10;
	int residual = segement % 2;
  SignalState state = (residual == 0) ? GREEN_THRU : RED;
  for (int s = Street::Tenth; s < Street::Fifteenth; ++s) {
    m_states[s] = state;
  }
}

Intersection::SignalState
Intersection::getSignalState(
  const Vehicle* const v
) const
{
  return m_states[v->currentPosition];
}

Intersection intersection;

void
arrival(
  const EventData* const arrivalData 
)
{
  Vehicle* v = arrivalData->vehicle(); 
  if (!arrivalData.continued()) {
    v->entryTime = current_time();	 // set time vehicle start waiting (now)
    v->currentPosition = 10.0;
  }
  else {
  }

	//if it is the begining point, could schedule a new arrival event
	// Compute the time-stamp of the new arrival, and only schedule a new arrival
	// if it is less than the maximum allowed time-stamp
	//just for simplicity, assume the vehicle only coming from the beginning point
  if (!arrivalData.continued()) {
    // Compute the time stamp of the new arrival.
    double ts = current_time() + randexp(NB_INTER_ARRIVAL_TIME);

    // Schedule new arrival only if the computed time stamp is less than maximum.
    if (ts < SIMULATION_TIME) {

      // create new arrival event

      Vehicle* newVehicle = new Vehicle();
      newVehicle->id = v->id + 1;

      EventData* newArrival = new EventData(newVehicle, EventData::ARRIVAL);

      schedule(ts, newArrival, arrival);
    }
  }

  // if intersect state is GREEN_THRU and there are no cars in the queue, then the vehicle can immediately
  // enter the intersection,to schedule the entered event

  intersection.updateSignalStates();
  Intersection::SignalState signal = intersection.getSignalState(v);
  // Get_current_Queque(v);                        

  if ((signal == GREEN_THRU) && (north_q.size() == 0)) {

    // the vehicle will have "entered" the intersection in Cross time units
		double ts = current_time() + INTERSECTION_CROSS_TIME;
        
		if (ts < SIMULATION_TIME) {

      v->endWaiting = current_time();     // vehicle stops waiting in the queue
      v->currentPosition += 1.0;
			// create new Entered event to simulation the vehicle ENTERED the bridge
			EventData *enteredIntersection = new EventData(v, EventData::ENTERED, true);

			schedule(ts, enteredIntersection, entered);
      // update the group size of the intersection
      //Increase_Group_size(v);
		}
	}
	// otherwise, the vehicle waits behind them.
	else {
    v->startWaiting = current_time();
    north_q.push(v);
	}
}

void
entered(
  const EventData* const enteredData 
)
{
  intersection.updateSignalStates();
  Intersection::SignalState signal = intersection.getSignalState(v);
  // Get_current_Queque(v);                        
  if ((signal == GREEN_THRU) && (north_q.size() == 0)) {
    Vehicle* v = north_q.front();
    v->endWaiting = current_time();
    v->totalWaiting += (v->endWaiting - v->startWaiting);
    v->currentPosition += 1.0;
    north_q.pop();

    double ts = current_time() + INTERSECTION_CROSS_TIME;
    if (ts < SIMULATION_TIME) {
			EventData *newEntered = new EventData(v, EventData::ENTERED, true);
      schedule(ts, newEntered, entered);
			// increment the size of the group of the intersection
			//Increase_Group_size(v);
    }
    ts = current_time() + ROAD_TRAVEL_TIME;
    if (ts < SIMULATION_TIME) {
			EventData *departureEvent =new EventData(enteredData->vehicle(), EventData::DEPARTURE);
      schedule(ts, departureEvent, departure);
    }
  }
}

void
departure(
  const EventData* const departureData 
)
{
	//Decrease_Group_size(departure_data->eventParam.departure_event.vehicle);
	Vehicle* v = departureData->vehicle();
  if (v->currentPosition != 11.0) {
    v->currentPosition += 1.0;
    double ts = current_time();
    if (ts < SIMULATION_TIME) {
      EventData* newArrival = new EventData();
    }
  }
  else {
    v->exitTime = current_time();
    // Collect data from the vehicle.
  }
}

void
runSimulation(
)
{
  while (FEL.size() > 0) {
    const Event& event = FEL.top(); 
    simtime = event.timestamp();
    event.callback();
    FEL.pop();
  }
}

/**
 * @brief  Main function which is called to start the simulator.  
 *
 * @param argc  Number of arguments passed to the simulator.
 * @param argv  Actual arguments passed to the simulator.
 *
 * @return Returns 0 if successful else returns a positive error code. 
 */
int
main(
  int argc,
  char** argv
)
{
  ProgramOptions options;
  try {
    options.parse(argc, argv);
  }
  catch (po::error& pe) {
    std::cerr << pe.what() << std::endl;
    return 1;
  }

  // Seed the random number generator.
  srand((unsigned int)time(NULL));
    
  // Create the first arrival on the queue and schedule it.
  EventData* newArrival = new EventData(new Vehicle(), EventData::ARRIVAL);

  // Set timestamp of the first arrival.
  double startTime = randexp(NB_INTER_ARRIVAL_TIME);

  schedule(startTime, newArrival, arrival);

  // Run the simulation.
  runSimulation();
    
  return 0;
}
