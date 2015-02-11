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


Intersection intersection;
Simulation simulation;
std::queue<Vehicle> northQueue;


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
}

void
ArrivalEvent::process(
)
{
  Vehicle v(m_vehicle);
  if (!m_continued) {
    v.entryTime = simulation.currentTime();	 // set time vehicle start waiting (now)
    v.currentPosition = 10.0;
  }
  else {
  }

	//if it is the begining point, could schedule a new arrival event
	// Compute the time-stamp of the new arrival, and only schedule a new arrival
	// if it is less than the maximum allowed time-stamp
	//just for simplicity, assume the vehicle only coming from the beginning point
  if (!m_continued) {
    // Compute the time stamp of the new arrival.
    double ts = simulation.currentTime() + randexp(NB_INTER_ARRIVAL_TIME);

    // Schedule new arrival only if the computed time stamp is less than maximum.
    if (ts < SIMULATION_TIME) {

      // create new arrival event

      Vehicle newVehicle;
      newVehicle.id = v.id + 1;

      simulation.schedule(new ArrivalEvent(ts, newVehicle));
    }
  }

  // if intersect state is GREEN_THRU and there are no cars in the queue, then the vehicle can immediately
  // enter the intersection,to schedule the entered event

  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.getSignalState(v);
  // Get_current_Queque(v);

  if ((signal == Intersection::GREEN_THRU) && (northQueue.size() == 0)) {

    // the vehicle will have "entered" the intersection in Cross time units
		double ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;

		if (ts < SIMULATION_TIME) {

      v.endWaiting = simulation.currentTime();     // vehicle stops waiting in the queue
      v.currentPosition += 1.0;
			// create new Entered event to simulation the vehicle ENTERED the bridge
			simulation.schedule(new EnteredEvent(ts, v, true));
      // update the group size of the intersection
      //Increase_Group_size(v);
		}
	}
	// otherwise, the vehicle waits behind them.
	else {
    v.startWaiting = simulation.currentTime();
    northQueue.push(v);
	}
}


/* -----------------------------------------  Implementation of EnteredEvent methods. ----------------------------------------- */
EnteredEvent::EnteredEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
EnteredEvent::process(
)
{
  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.getSignalState(m_vehicle);
  // Get_current_Queque(v);
  if ((signal == Intersection::GREEN_THRU) && (northQueue.size() > 0)) {
    Vehicle& v = northQueue.front();
    v.endWaiting = simulation.currentTime();
    v.totalWaiting += (v.endWaiting - v.startWaiting);
    v.currentPosition += 1.0;

    double ts = simulation.currentTime() + INTERSECTION_CROSS_TIME;
    if (ts < SIMULATION_TIME) {
      simulation.schedule(new EnteredEvent(ts, v, true));
			// increment the size of the group of the intersection
			//Increase_Group_size(v);
    }
    ts = simulation.currentTime() + ROAD_TRAVEL_TIME;
    if (ts < SIMULATION_TIME) {
      simulation.schedule(new DepartureEvent(ts, m_vehicle));
    }
    northQueue.pop();
  }
}


/* -----------------------------------------  Implementation of DepartureEvent methods. ----------------------------------------- */
DepartureEvent::DepartureEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
DepartureEvent::process(
)
{
	//Decrease_Group_size(departure_data->eventParam.departure_event.vehicle);
	Vehicle v(m_vehicle);
  if (v.currentPosition != 11.0) {
    v.currentPosition += 1.0;
    double ts = simulation.currentTime();
    if (ts < SIMULATION_TIME) {
			simulation.schedule(new ArrivalEvent(ts, v));
    }
  }
  else {
    v.exitTime = simulation.currentTime();
    // Collect data from the vehicle.
  }
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
  srand((unsigned int)time(NULL));

  Vehicle firstV;

  // Set timestamp of the first arrival.
  double startTime = randexp(NB_INTER_ARRIVAL_TIME);

  // Set timestamp of the first arrival.
  simulation.schedule(new ArrivalEvent(startTime, firstV));

  // Run the simulation.
  simulation.run();

  return 0;
}
