#include "TrafficEvent.hpp"

#include "Distribution.hpp"
#include "Intersection.hpp"
#include "Simulation.hpp"
#include "TrafficParameters.hpp"

#include <iostream>
#include <queue>

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
Intersection intersectionLeft;
TrafficParameters parameters;
std::vector<Vehicle> exitedVehicles;

bool
aggressiveDriver() {
  double x = urand();
  return std::isless(x, 0.05);
}



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
  
  Vehicle v(m_vehicle);
  v.entryTime = simulation.currentTime();  // set entry time of the vehicle
  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.signalState(m_vehicle);
  unsigned queueSize = intersection.queueSize(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);

  // then add the vehicle to the queue first
  intersection.addToQueue(m_vehicle);

  // check if it is green light and no vehicles in the opposite direction
  if ((signal == Intersection::GREEN_THRU) && Intersection::isClear(m_vehicle)) {
    bool isAggressive = aggressiveDriver();
    double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));
    // if there are no vehicles in the opposite directions or the driver is aggressive
    if ((queueSizeLeft != 0) && isAggressive) {
      // schedule the crossed event for the 
      // pop out the vehicle in the queue first
      Vehicle v(intersectionLeft.frontVehicle(m_vehicle));
      Intersection::setOccupied(m_vehicle);
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEventLeft(ts, v));
      }
    }
    else {
      // else if there are no vehicles in the opposite directions or the driver is not aggressive
      // start the go straight direction vehicle
      // start the opposite vehicle to accross 
      Intersection::setOccupied(m_vehicle);
      Vehicle v(intersection.frontVehicle(m_vehicle));
      v.totalWaiting += (simulation.currentTime() - v.waitingSince);
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        simulation.schedule(new CrossedEvent(ts, v));
      }
    }
  }

  // if it is the begining point, could schedule a new arrival event
  // Compute the time-stamp of the new arrival, and only schedule a new arrival
  // if it is less than the maximum allowed time-stamp
  // just for simplicity, assume the vehicle only coming from the beginning point
  double ts = simulation.currentTime() + randexp(parameters.get("NB_INTER_ARRIVAL_TIME"));
  // Schedule new arrival only if the computed time stamp is less than maximum.
  if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
    // create new arrival event with a new vehicle
    Vehicle newVehicle;
    newVehicle.id = m_vehicle.id + 1;
    newVehicle.currentPosition = Street::Tenth;
    // set_final_destination_Through_v(newVehicle);
    // Set_final_destination_Turnderiction(newVehicle);
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
  Vehicle v(m_vehicle);
  // finished the crossing, set the states to be clear
  Intersection::setClear(m_vehicle);

  //if the vehicle are accrossing the intersection and are entering the next intersection
  if (m_vehicle.currentPosition !=m_vehicle.destination){
    double ts = simulation.currentTime() + parameters.get("ROAD_TRAVEL_TIME");
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      // increment the position of the current vehicle
      v.currentPosition = static_cast<Street>(static_cast<int>(v.currentPosition) + 1);
      simulation.schedule(new DepartureEvent(ts, v));
    }
  }
  // if the vehicle are making the right turn,just schedule its departure
  else {
    double ts = simulation.currentTime();
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      simulation.schedule(new DepartureEvent(ts, v));
    }
  }

  // schedule the next enter event for its following cars 
  unsigned queueSize = intersection.queueSize(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);
  intersection.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersection.signalState(m_vehicle);
  bool leftAggressive = aggressiveDriver();

  // if there are cars in the queue, and the intersection is clear
  if (signal == Intersection::GREEN_THRU) {
    if (queueSize > 0) {
      if (!leftAggressive || (queueSizeLeft==0)) {
        double ts = simulation.currentTime() + randexp(parameters.get("ROAD_TRAVEL_TIME"));
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          Intersection::setOccupied(m_vehicle);
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEvent(ts, v));
        }
      }
      else { // if the aggressive driver and queuesize is not 0, start the left turn off
        double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          Intersection::setOccupied(m_vehicle);
          Vehicle v(intersectionLeft.frontVehicle(m_vehicle));
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEventLeft(ts, v));
        }
      }
    }
    else { // if the queuesize is 0, and the left turn lane has vehicle, start them off
      if (queueSizeLeft != 0) {
        double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME")))  {
          Intersection::setOccupied(m_vehicle);
          Vehicle v(intersectionLeft.frontVehicle(m_vehicle));
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEventLeft(ts, v));
        }
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
  v.currentPosition = static_cast<Street>(static_cast<int>(v.currentPosition) + 1);
  if (v.currentPosition != Street::Eleventh) {
    double ts = simulation.currentTime();
    // schedule an arrival event for this vehicle at the next intersection
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
			simulation.schedule(new ArrivalEvent(ts, v));
    }
  }
  else {
    v.exitTime = simulation.currentTime();
    Log() << "Vehicle with id: " << v.id << " exited at time: " << v.exitTime << " Total waiting time: " << v.totalWaiting;
    // Collect data from the vehicle.
    exitedVehicles.push_back(v);
  }

}


void setLeftTurnCurrentPosition(Vehicle &v){
  double x = urand(); 
  if (std::isless(x, 0.25)) {
    v.currentPosition = Street::Eleventh;
  }
  else if (std::isgreaterequal(x, 0.25) && std::isless(x, 0.5)) {
    v.currentPosition = Street::Twelfth;
  }
  else if (std::isgreaterequal(x, 0.5) && std::isless(x, 0.75)) {
    v.currentPosition = Street::Thirteenth;
  }
  else {
    v.currentPosition = Street::Fourteenth;
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
  Log() << "Processing arrival event for Left turn vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  
  // get the intersection data
  intersectionLeft.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersectionLeft.signalState(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);
  unsigned queueSize = intersection.queueSize(m_vehicle);

  Log() << "Size of the left queue is: " << queueSizeLeft;

  // then add the vehicle to the queue first
  intersectionLeft.addToQueue(m_vehicle);

  //check if it is green light and no vehicles in the opposite direction
  if ((signal == Intersection::GREEN_THRU) && Intersection::isClear(m_vehicle)) {
    bool isAggressive = aggressiveDriver();
    // if there are no vehicles in the opposite directions or the driver is aggressive
    double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));
    
    if ((queueSize == 0) || isAggressive) {
      //schedule the crossed event
      //pop out the vehicle in the queue first
      Vehicle v(intersection.frontVehicle(m_vehicle));
      Intersection::setOccupied(m_vehicle);
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEventLeft(ts, v));
      }
    }
    else {
      // else if there are vehicles in the opposite directions and the driver is not aggressive
      // add it to the queue , start the opposite direction vehicle
      //else if (queuesize!=0 && !isAggressive)
      //start the opposite vehicle to accross 
      Intersection::setOccupied(m_vehicle);
      Vehicle v(intersection.frontVehicle(m_vehicle));
      v.totalWaiting += (simulation.currentTime() - v.waitingSince);
      simulation.schedule(new CrossedEvent(ts, v));
    }
  }
   
  //schedule arrival event for random intersection
  double ts = simulation.currentTime() + randexp(parameters.get("NB_INTER_ARRIVAL_TIME"));
  // Schedule new arrival only if the computed time stamp is less than maximum.
  if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
    // create new arrival event with a new vehicle
    Vehicle newVehicle;
    newVehicle.id = m_vehicle.id + 1;
    setLeftTurnCurrentPosition(newVehicle);
    simulation.schedule(new ArrivalEvent(ts, newVehicle));
  }
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
  // set the intersection to clear
  Intersection::setClear(m_vehicle);
  // update the signal time 
  intersectionLeft.updateSignalStates(simulation.currentTime());
  Intersection::SignalState signal = intersectionLeft.signalState(m_vehicle);
  unsigned queueSize = intersection.queueSize(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);

  bool nextAggressive = aggressiveDriver();
  if (signal == Intersection::GREEN_THRU) {
    //if the left-queue is not empty and next driver is aggressive  
    if (queueSizeLeft != 0) {
      if (nextAggressive || (queueSize == 0)) {
        Vehicle v(intersectionLeft.frontVehicle(m_vehicle));
        double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));

        Intersection::setOccupied(m_vehicle);
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          simulation.schedule(new CrossedEventLeft(ts,v));
        }
      }
      else {
        //if not aggressive and opposite got vehicle
        double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));
        Vehicle v(intersection.frontVehicle(m_vehicle));
        Intersection::setOccupied(m_vehicle);
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          simulation.schedule(new CrossedEvent(ts,v));
        }
      }
    } // queuesizeleft is 0, and opposite got vehicle, start them off
    else {
      if (queueSize != 0){
        double ts = simulation.currentTime() + randexp(parameters.get("INTERSECTION_CROSS_TIME"));
        Vehicle v(intersection.frontVehicle(m_vehicle));
        Intersection::setOccupied(m_vehicle);
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          simulation.schedule(new CrossedEvent(ts,v));
        }
      }
    }
  }
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
  double startTime = randexp(parameters.get("NB_INTER_ARRIVAL_TIME"));

  // create the first vehicle and schedule an arrival event at the start time
  Vehicle firstV = {};
  firstV.id = 0;
  firstV.currentPosition = Street::Tenth;
  simulation.schedule(new ArrivalEvent(startTime, firstV));

  Vehicle firstLeft = {};
  firstLeft.id = 0;
  firstLeft.currentPosition=Street::Eleventh;

  simulation.schedule(new ArrivalEventLeft(startTime,firstLeft));

  // Run the simulation.
  simulation.run();

  // calculate average waiting time
  if (exitedVehicles.size() > 0) {
    double averageWaiting = 0.0;
    for (const Vehicle& v : exitedVehicles) {
      averageWaiting += v.totalWaiting;
    }
    averageWaiting /= exitedVehicles.size();

    std::cout << "Average waiting time for " << exitedVehicles.size() << " vehicles, that crossed the stretch, was: " << averageWaiting << std::endl;
  }
  else {
    std::cout << "No vehicles crossed the stretch! Something went wrong." << std::endl;
  }

  return 0;
}
