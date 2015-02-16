#include "TrafficEvent.hpp"

#include "Distribution.hpp"
#include "Intersection.hpp"
#include "ProgramOptions.hpp"
#include "Simulation.hpp"

#include <iostream>
#include <queue>

#define INTERSECTION_CROSS_TIME 1
#define SIMULATION_TIME 14400
#define NB_INTER_ARRIVAL_TIME 20
#define ROAD_TRAVEL_TIME 3

/**
 * @brief Class for logging debug messages.
 */
class Log {
public:
  Log() { }

  template<typename T>
  Log&
  operator<<(
    const T& t
  )
  {
#ifndef NDEBUG
    std::cout << t;
#endif
    return *this;
  }

  ~Log()
  {
#ifndef NDEBUG
    std::cout << std::endl;
#endif
  }
};


Intersection intersection;
Simulation simulation;


/* -----------------------------------------  Implementation of TrafficEvent methods. ----------------------------------------- */
TrafficEvent::TrafficEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : Event(timestamp),
  m_vehicle(vehicle),
  m_continued(continued)
{
}

TrafficEvent::~TrafficEvent(
)
{
}


/* -----------------------------------------  Implementation of ArrivalEvent methods. ----------------------------------------- */
ArrivalEvent::ArrivalEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
  Log() << "Scheduled arrival event for vehicle id: " << vehicle.id << " at time: " << timestamp;
}

void
ArrivalEvent::process(
)
{
  Log() << "Processing arrival event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  Vehicle v(m_vehicle);
  v.entryTime = simulation.currentTime();	 // set time vehicle start waiting (now)

  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.signalState(m_vehicle);
  unsigned queueSize = intersection.queueSize(m_vehicle);

  if (signal == Intersection::GREEN_THRU) {
    double ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
    if (ts < SIMULATION_TIME) {
      if (queueSize > 0) {
        Vehicle v(intersection.frontVehicle(m_vehicle));
        v.endWaiting = simulation.currentTime();
        v.totalWaiting += (v.endWaiting - v.startWaiting);
        simulation.schedule(new CrossedEvent(ts, v));
      }
      else { 
        Vehicle v(m_vehicle);
        v.endWaiting = simulation.currentTime();
        v.totalWaiting += (v.startWaiting - v.endWaiting);
        double ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
        simulation.schedule(new CrossedEvent(ts, v));
      }
    }
  }
  if ((signal != Intersection::GREEN_THRU) || (queueSize > 0)) {
    v.startWaiting = simulation.currentTime();
    intersection.addToQueue(v);
  }
  //if it is the begining point, could schedule a new arrival event
  // Compute the time-stamp of the new arrival, and only schedule a new arrival
  // if it is less than the maximum allowed time-stamp
  //just for simplicity, assume the vehicle only coming from the beginning point
  double ts = simulation.currentTime() + randexp(NB_INTER_ARRIVAL_TIME);
  // Schedule new arrival only if the computed time stamp is less than maximum.
  if (ts < SIMULATION_TIME) {
    // create new arrival event with a new vehicle
    Vehicle newVehicle;
    newVehicle.id = v.id + 1;
    newVehicle.currentPosition = 10;
    simulation.schedule(new ArrivalEvent(ts, newVehicle));
  }
}


/* -----------------------------------------  Implementation of CrossedEvent methods. ----------------------------------------- */
CrossedEvent::CrossedEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
  Log() << "Scheduled crossed event for vehicle id: " << vehicle.id << " at time: " << timestamp;
}

void
CrossedEvent::process(
)
{
  Log() << "Processing crossed event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  double ts = simulation.currentTime() + ROAD_TRAVEL_TIME;
  if (ts < SIMULATION_TIME) {
    Vehicle v(m_vehicle);
    v.currentPosition += 1;
    simulation.schedule(new DepartureEvent(ts, v));
  }
  unsigned queueSize = intersection.queueSize(m_vehicle);

  if (queueSize > 0) {
    intersection.updateSignalStates(simulation.currentTime());
    Intersection::SignalState signal = intersection.signalState(m_vehicle);
    if (signal == Intersection::GREEN_THRU) {
      ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
      if (ts < SIMULATION_TIME) {
        Vehicle v(intersection.frontVehicle(m_vehicle));
        v.endWaiting = simulation.currentTime();
        v.totalWaiting += (v.endWaiting - v.startWaiting);
        simulation.schedule(new CrossedEvent(ts, v));
      }
    }
  }
}


/* -----------------------------------------  Implementation of DepartureEvent methods. ----------------------------------------- */
DepartureEvent::DepartureEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
  Log() << "Scheduled departure event for vehicle id: " << vehicle.id << " at time: " << timestamp;
}

void
DepartureEvent::process(
)
{
  Log() << "Processing departure event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  intersection.decreaseGroupSize(m_vehicle);

	Vehicle v(m_vehicle);
  if (v.currentPosition != 11) {
    v.currentPosition += 1;
    double ts = simulation.currentTime();
    if (ts < SIMULATION_TIME) {
			simulation.schedule(new ArrivalEvent(ts, v));
    }
  }
  else {
    v.exitTime = simulation.currentTime();
    Log() << "Vehicle with id: " << v.id << " exited at time: " << v.exitTime;
    // Collect data from the vehicle.
  }
}

/* -----------------------------------------  Implementation of ArrivalEventLeft methods. ----------------------------------------- */
ArrivalEventLeft::ArrivalEventLeft(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
ArrivalEventLeft::process(
)
{
}


/* -----------------------------------------  Implementation of CrossedEventLeft methods. ----------------------------------------- */
CrossedEventLeft::CrossedEventLeft(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
CrossedEventLeft::process(
)
{
}


/* -----------------------------------------  Implementation of DepartureEventLeft methods. ----------------------------------------- */
DepartureEventLeft::DepartureEventLeft(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
DepartureEventLeft::process(
)
{
}
/* -----------------------------------------  Implementation of main function. ----------------------------------------- */
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
  std::srand(std::time(0));

  Vehicle firstV;
  firstV.id = 0;
  firstV.currentPosition = 10;

  // Set timestamp of the first arrival.
  double startTime = randexp(NB_INTER_ARRIVAL_TIME);

  // Set timestamp of the first arrival.
  simulation.schedule(new ArrivalEvent(startTime, firstV));

  // Run the simulation.
  simulation.run();

  return 0;
}
