#include "TrafficEvent.hpp"

#include "Distribution.hpp"
#include "Intersection.hpp"
#include "Simulation.hpp"
#include "TrafficParameters.hpp"

#include <iostream>
#include <queue>

#define INTERSECTION_CROSS_TIME 1
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
TrafficParameters parameters;


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
  Simulation& simulation
)
{
  Log() << "Processing arrival event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();

  // get the signal state for the intersection where the vehicle has arrived
  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.signalState(m_vehicle);
  // get the queue size for the intersection where the vehicle has arrived
  unsigned queueSize = intersection.queueSize(m_vehicle);

  // if the signal is green then schedule crossed events
  if (signal == Intersection::GREEN_THRU) {
    // schedule the crossed event after the time it takes to cross an intersection
    double ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
    // schedule new arrival only if the computed time stamp is less than the cutoff
    if (std::isless(ts, parameters.cutoffTime())) {
      if (queueSize > 0) {
        // if there is a queue then schedule crossed event for the first vehicle
        Vehicle v(intersection.frontVehicle(m_vehicle));
        // end waiting for the vehicle, if it was waiting
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        // reset waiting time
        v.waitingSince = 0.0;
        simulation.schedule(new CrossedEvent(ts, v));
      }
      else {
        // if there is no queue then schedule crossed event for this vehicle
        simulation.schedule(new CrossedEvent(ts, m_vehicle));
      }
    }
  }
  if ((signal != Intersection::GREEN_THRU) || (queueSize > 0)) {
    // if the signal state is not green or if there is a queue,
    // add the current vehicle to the back of the queue
    Vehicle v(m_vehicle);
    v.waitingSince = simulation.currentTime();
    intersection.addToQueue(v);
  }
  // compute timestamp for arrival of the next vehicle
  double ts = simulation.currentTime() + randexp(NB_INTER_ARRIVAL_TIME);
  // schedule new arrival only if the computed time stamp is less than the cutoff
  if (std::isless(ts, parameters.cutoffTime())) {
    // create new arrival event with a new vehicle and value initialize the struct
    Vehicle newVehicle = {};
    // assign the next id to this vehicle
    newVehicle.id = m_vehicle.id + 1;
    //just for simplicity, assume the vehicle only coming from the beginning point
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
  Simulation& simulation
)
{
  Log() << "Processing crossed event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  // compute timestamp for scheduling departure event for this vehicle
  double ts = simulation.currentTime() + ROAD_TRAVEL_TIME;
  // schedule departure for this vehicle only if the computed time stamp is less than the cutoff
  if (std::isless(ts, parameters.cutoffTime())) {
    simulation.schedule(new DepartureEvent(ts, m_vehicle));
  }

  unsigned queueSize = intersection.queueSize(m_vehicle);
  if (queueSize > 0) {
    intersection.updateSignalStates(simulation.currentTime());
    Intersection::SignalState signal = intersection.signalState(m_vehicle);
    // if there is a queue behind the vehicle and the signal is still green,
    // schedule crossed event for the next vehicle in the queue
    if (signal == Intersection::GREEN_THRU) {
      ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
      if (std::isless(ts, parameters.cutoffTime())) {
        Vehicle v(intersection.frontVehicle(m_vehicle));
        // end waiting for the vehicle
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        v.waitingSince = 0.0;
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
  Simulation& simulation
)
{
  Log() << "Processing departure event for vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  intersection.decreaseGroupSize(m_vehicle);

	Vehicle v(m_vehicle);
  // increment the position of the current vehicle
  v.currentPosition += 1;
  if (v.currentPosition != 11) {
    double ts = simulation.currentTime();
    // schedule an arrival event for this vehicle at the next intersection
    if (std::isless(ts, parameters.cutoffTime())) {
			simulation.schedule(new ArrivalEvent(ts, v));
    }
  }
  else {
    v.exitTime = simulation.currentTime();
    Log() << "Vehicle with id: " << v.id << " exited at time: " << v.exitTime << " Total waiting time: " << v.totalWaiting;
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
  Simulation& simulation
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
  Simulation& simulation
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
  Simulation& simulation
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
  try {
    parameters.parse(argc, argv);
  }
  catch (po::error& pe) {
    std::cerr << pe.what() << std::endl;
    return 1;
  }

  // Seed the random number generator.
  std::srand(parameters.randomSeed());

  // create a new simulation object
  Simulation simulation;

  // compute timestamp of the first arrival
  double startTime = randexp(NB_INTER_ARRIVAL_TIME);


  // create the first vehicle and schedule an arrival event at the start time
  Vehicle firstV = {};
  firstV.id = 0;
  firstV.currentPosition = 10;
  simulation.schedule(new ArrivalEvent(startTime, firstV));

  // Run the simulation.
  simulation.run();

  return 0;
}
