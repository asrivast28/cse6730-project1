#include "Event.hpp"
#include "ProgramOptions.hpp"
#include "SimulationEngine.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <queue>

#define SIMULATION_TIME 14400
#define NB_INTER_ARRIVAL_TIME 20

#define INTERSECTION_CROSS_TIME 1
#define ROAD_TRAVEL_TIME 3

SimulationEngine engine;

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


std::queue<Vehicle> northQueue;

class Intersection {
public:
  enum
  SignalState {
    GREEN_THRU,
    GREEN_LEFT,
    YELLOW,
    RED
  };

  Intersection();

public:
  void
  updateSignalStates();

  Intersection::SignalState
  getSignalState(
    const Vehicle&
  ) const;

private:
  std::vector<SignalState> m_states;
}; // class IntersectionState

Intersection::Intersection(
) : m_states(static_cast<size_t>(Street::Fifteenth))
{
}

void
Intersection::updateSignalStates(
)
{
	double timer = engine.currentTime();
	int segement = timer / 10;
	int residual = segement % 2;
  SignalState state = (residual == 0) ? GREEN_THRU : RED;
  for (int s = Street::Tenth; s < Street::Fifteenth; ++s) {
    m_states[s] = state;
  }
}

Intersection::SignalState
Intersection::getSignalState(
  const Vehicle& v
) const
{
  return m_states[v.currentPosition];
}

Intersection intersection;

void arrival(const EventData&);
void entered(const EventData&);
void departure(const EventData&);

void
arrival(
  const EventData& arrivalData
)
{
  Vehicle v(arrivalData.vehicle());
  if (!arrivalData.continued()) {
    v.entryTime = engine.currentTime();	 // set time vehicle start waiting (now)
    v.currentPosition = 10.0;
  }
  else {
  }

	//if it is the begining point, could schedule a new arrival event
	// Compute the time-stamp of the new arrival, and only schedule a new arrival
	// if it is less than the maximum allowed time-stamp
	//just for simplicity, assume the vehicle only coming from the beginning point
  if (!arrivalData.continued()) {
    // Compute the time stamp of the new arrival.
    double ts = engine.currentTime() + randexp(NB_INTER_ARRIVAL_TIME);

    // Schedule new arrival only if the computed time stamp is less than maximum.
    if (ts < SIMULATION_TIME) {

      // create new arrival event

      Vehicle newVehicle;
      newVehicle.id = v.id + 1;

      EventData newArrival(newVehicle);

      engine.schedule(ts, newArrival, arrival);
    }
  }

  // if intersect state is GREEN_THRU and there are no cars in the queue, then the vehicle can immediately
  // enter the intersection,to schedule the entered event

  intersection.updateSignalStates();
  Intersection::SignalState signal = intersection.getSignalState(v);
  // Get_current_Queque(v);

  if ((signal == Intersection::GREEN_THRU) && (northQueue.size() == 0)) {

    // the vehicle will have "entered" the intersection in Cross time units
		double ts = engine.currentTime() + INTERSECTION_CROSS_TIME;

		if (ts < SIMULATION_TIME) {

      v.endWaiting = engine.currentTime();     // vehicle stops waiting in the queue
      v.currentPosition += 1.0;
			// create new Entered event to simulation the vehicle ENTERED the bridge
			EventData enteredIntersection(v, true);

			engine.schedule(ts, enteredIntersection, entered);
      // update the group size of the intersection
      //Increase_Group_size(v);
		}
	}
	// otherwise, the vehicle waits behind them.
	else {
    v.startWaiting = engine.currentTime();
    northQueue.push(v);
	}
}

void
entered(
  const EventData& enteredData
)
{
  intersection.updateSignalStates();
  Intersection::SignalState signal = intersection.getSignalState(enteredData.vehicle());
  // Get_current_Queque(v);
  if ((signal == Intersection::GREEN_THRU) && (northQueue.size() > 0)) {
    Vehicle& v = northQueue.front();
    v.endWaiting = engine.currentTime();
    v.totalWaiting += (v.endWaiting - v.startWaiting);
    v.currentPosition += 1.0;

    double ts = engine.currentTime() + INTERSECTION_CROSS_TIME;
    if (ts < SIMULATION_TIME) {
			EventData newEntered(v, true);
      engine.schedule(ts, newEntered, entered);
			// increment the size of the group of the intersection
			//Increase_Group_size(v);
    }
    ts = engine.currentTime() + ROAD_TRAVEL_TIME;
    if (ts < SIMULATION_TIME) {
			EventData departureEvent(enteredData.vehicle());
      engine.schedule(ts, departureEvent, departure);
    }
    northQueue.pop();
  }
}

void
departure(
  const EventData& departureData
)
{
	//Decrease_Group_size(departure_data->eventParam.departure_event.vehicle);
	Vehicle v(departureData.vehicle());
  if (v.currentPosition != 11.0) {
    v.currentPosition += 1.0;
    double ts = engine.currentTime();
    if (ts < SIMULATION_TIME) {
      EventData newArrival(v);
			engine.schedule(ts, newArrival, arrival);
    }
  }
  else {
    v.exitTime = engine.currentTime();
    // Collect data from the vehicle.
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

  Vehicle firstV;

  // Create the first arrival on the queue and schedule it.
  EventData newArrival(firstV);

  // Set timestamp of the first arrival.
  double startTime = randexp(NB_INTER_ARRIVAL_TIME);

  engine.schedule(startTime, newArrival, arrival);

  // Run the simulation.
  engine.run();

  return 0;
}
