#include "TrafficEvent.hpp"

#include "Distribution.hpp"
#include "Intersection.hpp"
#include "Simulation.hpp"
#include "TrafficParameters.hpp"

#include <cstdio>
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


static Distribution distribution;
static Intersection intersection;
static Intersection intersectionLeft;
static TrafficParameters parameters;
static std::vector<Vehicle> exitedVehicles;
static unsigned vehicleIds = 0;

static
int
uniqueId() {
  return vehicleIds++;
}

static
bool
aggressiveDriver() {
  double x = distribution.urand();
  return std::isless(x, parameters.get("AGGRESSIVE_DRIVER_LIKELIHOOD"));
}

static
void
setLeftTurnCurrentPosition(
  Vehicle &v
)
{
  double x = distribution.urand();
  if (std::isless(x, 0.25)) {
    v.position = Street::Eleventh;
  }
  else if (std::isgreaterequal(x, 0.25) && std::isless(x, 0.5)) {
    v.position = Street::Twelfth;
  }
  else if (std::isgreaterequal(x, 0.5) && std::isless(x, 0.75)) {
    v.position = Street::Thirteenth;
  }
  else {
    v.position = Street::Fourteenth;
  }
  v.origin = v.position;
}


static
void
setRandomJoinVehicleInitialPosition(Vehicle &v) {
  double x = distribution.urand();
  if (std::isless(x, parameters.get("RANDOM_JOIN_VEHICLE_TENTH"))) {
    v.position = Street::Tenth;
  }
  else if (std::isgreaterequal(x,parameters.get("RANDOM_JOIN_VEHICLE_TENTH") ) && std::isless(x, parameters.get("RANDOM_JOIN_VEHICLE_ELEVENTH"))) {
    v.position = Street::Eleventh;
  }
  else if (std::isgreaterequal(x, parameters.get("RANDOM_JOIN_VEHICLE_ELEVENTH")) && std::isless(x, parameters.get("RANDOM_JOIN_VEHICLE_TWELFTH"))) {
    v.position = Street::Twelfth;
  }
  else if (std::isgreaterequal(x, parameters.get("RANDOM_JOIN_VEHICLE_TWELFTH")) && std::isless(x, parameters.get("RANDOM_JOIN_VEHICLE_THRITEENTH"))) {
    v.position = Street::Thirteenth;
  }
  
  else {
    v.position = Street::Fourteenth;
  }
}


static
void
setRandomTurnDirection(Vehicle &v) {
  double x = distribution.urand();
  if (std::isless(x, parameters.get("LEFT_TURN_PROBABILITY"))) {
    v.turndirection = TurnDirection::Left_turn;
  }
  else if (std::isgreaterequal(x,parameters.get("LEFT_TURN_PROBABILITY") ) && std::isless(x, parameters.get("RIGHT_TURN_PROBABILITY"))) {
    v.turndirection = TurnDirection::Right_turn;
  }
  else if 
  (std::isgreaterequal(x, parameters.get("RIGHT_TURN_PROBABILITY")) && std::isless(x, parameters.get("GO_THROUGH_PROBABILITY"))) {
    v.turndirection = TurnDirection::GO_THROUGH;
  }
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
}

void
ArrivalEvent::process(
  Simulation& simulation
)
{
  Log() << "Arriving thru vehicle id: " << m_vehicle.id << " at " << m_vehicle.position << " at time: " << simulation.currentTime();

  
  Intersection::SignalState signal = intersection.getSignalState(m_vehicle, simulation.currentTime(), parameters);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);

  Vehicle arrivedV(m_vehicle);
  // set entry time of the vehicle
  arrivedV.entryTime = simulation.currentTime();
  // then add the vehicle to the queue first
  arrivedV.waitingSince = simulation.currentTime();
  ////determine whether the vehicle is going to make the right turn or not
  // if make the left
 //if (m_vehicle.turndirection!=TurnDirection::Left_turn){
  intersection.addToQueue(arrivedV);

  const Vehicle& frontV = intersection.viewFrontVehicle(m_vehicle);
  
  // check if it is green light and no vehicles in the opposite direction
  // and the vehicle is not making the left turn 
  if ((signal == Intersection::GREEN_THRU) && Intersection::isClear(m_vehicle) && frontV.turndirection == GO_THROUGH) {
    bool isAggressive = aggressiveDriver();
    double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
    // if there are no vehicles in the opposite directions or the driver is aggressive
    if ((queueSizeLeft != 0) && isAggressive) {
      // schedule the crossed event for the
      // pop out the vehicle in the queue first
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        Vehicle v(intersectionLeft.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEventLeft(ts, v));
      }
    }
    else {
      // else if there are no vehicles in the opposite directions or the driver is not aggressive
      // start the go straight direction vehicle
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        Vehicle v(intersection.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEvent(ts, v));
      }
    }
    // one of the vehicles is going to cross so we can set the intersection state to occupied
    Intersection::setOccupied(m_vehicle);
  }  
 // if the vehicle is making the left turn, schedule its crossed event in order to 
  // initiate its next crossed actions
 
  else if (frontV.turndirection!=GO_THROUGH){
    // schedule its crossed action so as let the action to triggle another crossed event
     if (frontV.turndirection == Left_turn){
       double ts = simulation.currentTime();//can go to the left turn lane immediatly;
       Vehicle v(intersection.getFrontVehicle(m_vehicle));
       v.totalWaiting += (simulation.currentTime() - v.waitingSince);
       simulation.schedule(new CrossedEvent(ts, v));
       }

        else {
        double ts = simulation.currentTime()+distribution.normal(parameters.get("RIGHT_TURN_DISAPPEAR_TIME"));//can go to the left turn lane immediatly;
        Vehicle v(intersection.getFrontVehicle(m_vehicle));
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEvent(ts, v));
        }
        } 

}


  if (!m_continued) {
    // if it is the begining point, could schedule a new arrival event
    // Compute the time-stamp of the new arrival, and only schedule a new arrival
    // if it is less than the maximum allowed time-stamp
    // just for simplicity, assume the vehicle only coming from the beginning point
    double ts = simulation.currentTime() + distribution.exponential(parameters.get("NB_INTER_ARRIVAL_TIME"));
    // Schedule new arrival only if the computed time stamp is less than maximum.
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      // create new arrival event with a new vehicle
      Vehicle newVehicle = {};
      newVehicle.id = uniqueId();
      newVehicle.origin = m_vehicle.position;
      newVehicle.position = m_vehicle.position;
      setRandomTurnDirection(newVehicle);
      simulation.schedule(new ArrivalEvent(ts, newVehicle));
    }
  }
}




/* -----------------------------------------  Implementation of CrossedEvent methods. ----------------------------------------- */
CrossedEvent::CrossedEvent(
  const double timestamp,
  const Vehicle& vehicle,
  const bool continued
) : TrafficEvent(timestamp, vehicle, continued)
{
}

void
CrossedEvent::process(
  Simulation& simulation
)
{
  Log() << "Crossing thru vehicle id: " << m_vehicle.id << " at time: " << simulation.currentTime();
  // finished the crossing, set the states to be clear
  Intersection::setClear(m_vehicle);

  //if the vehicle are accrossing the intersection and are entering the next intersection
  if (m_vehicle.position != Street::Fifteenth && m_vehicle.turndirection == GO_THROUGH) {
    double ts = simulation.currentTime() + parameters.get("SECTION_TRAVEL_TIME");
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      // increment the position of the current vehicle and schedule the departure event
      simulation.schedule(new DepartureEvent(ts, m_vehicle));
    }
  }
  else { // if the vehicle are making the right or left turn, just schedule its departure
    Vehicle v(m_vehicle);
    v.exitTime = simulation.currentTime();
    Log() << "Vehicle with id: " << v.id << " took a right turn at time: " << v.exitTime << " Total waiting time: " << v.totalWaiting;
    exitedVehicles.push_back(v);
  }

  // schedule the next enter event for its following cars
  unsigned queueSize = intersection.queueSize(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);
  Intersection::SignalState signal = intersection.getSignalState(m_vehicle, simulation.currentTime(), parameters);
  // if there are cars in the queue, and the intersection is clear
  if (signal == Intersection::GREEN_THRU) {
    if ((queueSize > 0) && (intersection.viewFrontVehicle(m_vehicle).turndirection == GO_THROUGH)) {
      bool leftAggressive = aggressiveDriver();
      if (!leftAggressive || (queueSizeLeft == 0)) {
        Intersection::setOccupied(m_vehicle);
        double ts = simulation.currentTime() + parameters.get("SECTION_TRAVEL_TIME");
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          Vehicle v(intersection.getFrontVehicle(m_vehicle));
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEvent(ts, v));
        }
      }
    }
    else if (queueSizeLeft != 0) { // if the queuesize is 0, and the left turn lane has vehicle, start them off
      double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME")))  {
        Vehicle v(intersectionLeft.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEventLeft(ts, v));
        Intersection::setOccupied(m_vehicle);
      }
    }
  }
  // else if the front vehicle of the go through lane is making the right or left turn, just schedule its 
  // crossed actions to triggle the following event
 
  else if (queueSize > 0) {
    const Vehicle& frontV = intersection.viewFrontVehicle(m_vehicle);
    if (frontV.turndirection!=GO_THROUGH) {
    // schedule its crossed action so as let the action to triggle another crossed event
       if (frontV.turndirection == Left_turn) {
         double ts = simulation.currentTime();//can go to the left turn lane immediatly;
         Vehicle v(intersection.getFrontVehicle(m_vehicle));
         v.totalWaiting += (simulation.currentTime() - v.waitingSince);
         simulation.schedule(new CrossedEvent(ts, v));
       }
       else {
         double ts = simulation.currentTime()+distribution.normal(parameters.get("RIGHT_TURN_DISAPPEAR_TIME"));//can go to the left turn lane immediatly;
         Vehicle v(intersection.getFrontVehicle(m_vehicle));
         if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
         v.totalWaiting += (simulation.currentTime() - v.waitingSince);
         simulation.schedule(new CrossedEvent(ts, v));
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
}

void
DepartureEvent::process(
  Simulation& simulation
)
{
  Log() << "Departing vehicle id: " << m_vehicle.id << " from " << m_vehicle.position << " at time: " << simulation.currentTime();
  intersection.decreaseGroupSize(m_vehicle);

	Vehicle v(m_vehicle);
  // increment the position of the current vehicle
  v.position = static_cast<Street>(static_cast<int>(v.position) + 1);
  if (v.position != Street::Fifteenth) {
    double ts = simulation.currentTime();
    // schedule an arrival event for this vehicle at the next intersection
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      setRandomTurnDirection(v);  // every time the vehicle re-enter the intersection, change its turn-direction randomly
			simulation.schedule(new ArrivalEvent(ts, v, true));
    }
  }
  else {
    v.exitTime = simulation.currentTime();
    Log() << "Vehicle with id: " << v.id << " exited at time: " << v.exitTime << " Total waiting time: " << v.totalWaiting;
    // Collect data from the vehicle.
    exitedVehicles.push_back(v);
    //schedule random arrival event to simulate the vehicles that come in between the intersection
    //including the vehicle traveling from the right turn from the east and west direction
    //and thane vehicle that come from along the way(like from the parking lots,residential apartments etc)
    if (!m_continued) {
    double ts = simulation.currentTime() + distribution.normal(parameters.get("RANDOM_JOIN_VEHICLE_ARRIVAL_TIME"));
    // Schedule new arrival only if the computed time stamp is less than maximum.
    if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
      // create new arrival event with a new vehicle
      Vehicle newVehicle = {};
      newVehicle.id = uniqueId();
      newVehicle.origin = m_vehicle.position;
      setRandomJoinVehicleInitialPosition(newVehicle);
      setRandomTurnDirection(newVehicle);
      simulation.schedule(new ArrivalEvent(ts, newVehicle,true));
  }
}
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
  Log() << "Arriving left turn vehicle id: " << m_vehicle.id << " at " << m_vehicle.position << " at time: " << simulation.currentTime();

  // get the intersection data
  Intersection::SignalState signal = intersectionLeft.getSignalState(m_vehicle, simulation.currentTime(), parameters);
  unsigned queueSize = intersection.queueSize(m_vehicle);

  Vehicle arrivedV(m_vehicle);
  // set entry time of the vehicle
  arrivedV.entryTime = simulation.currentTime();
  // then add the vehicle to the queue first
  arrivedV.waitingSince = simulation.currentTime();
  intersectionLeft.addToQueue(arrivedV);

  //check if it is green light and no vehicles in the opposite direction
  if ((signal == Intersection::GREEN_THRU) && Intersection::isClear(m_vehicle)) {
    bool isAggressive = aggressiveDriver();
    // if there are no vehicles in the opposite directions or the driver is aggressive
    double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");

    if ((queueSize == 0) || isAggressive) {
      //schedule the crossed event
      //pop out the vehicle in the queue first
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        Vehicle v(intersectionLeft.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEventLeft(ts, v));
        Intersection::setOccupied(m_vehicle);
      }
    }
    else {
      // else if there are vehicles in the opposite directions and the driver is not aggressive
      // add it to the queue , start the opposite direction vehicle
      //else if (queuesize!=0 && !isAggressive)
      //start the opposite vehicle to accross
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        Vehicle v(intersection.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEvent(ts, v));
        Intersection::setOccupied(m_vehicle);
      }
    }
  }

  //schedule left arrival event for random intersection
  double ts = simulation.currentTime() + distribution.exponential(parameters.get("LEFT_INTER_ARRIVAL_TIME"));
  // Schedule new arrival only if the computed time stamp is less than maximum.
  if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
    // create new arrival event with a new vehicle
    Vehicle newVehicle = {};
    newVehicle.id = uniqueId();
    setLeftTurnCurrentPosition(newVehicle);
    simulation.schedule(new ArrivalEventLeft(ts, newVehicle));
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
  Vehicle leftV(m_vehicle);
  leftV.exitTime = simulation.currentTime();
  Log() << "Vehicle with id: " << leftV.id << " took a left turn at time: " << leftV.exitTime << " Total waiting time: " << leftV.totalWaiting;
  exitedVehicles.push_back(leftV);
  // set the intersection to clear
  Intersection::setClear(m_vehicle);
  // update the signal time
  Intersection::SignalState signal = intersectionLeft.getSignalState(m_vehicle, simulation.currentTime(), parameters);
  unsigned queueSize = intersection.queueSize(m_vehicle);
  unsigned queueSizeLeft = intersectionLeft.queueSize(m_vehicle);

  bool nextAggressive = aggressiveDriver();
  if (signal == Intersection::GREEN_THRU) {
    //if the left-queue is not empty and next driver is aggressive
    if (queueSizeLeft != 0) {
      if (nextAggressive || (queueSize == 0)) {
        double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          Vehicle v(intersectionLeft.getFrontVehicle(m_vehicle));
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEventLeft(ts,v));
          Intersection::setOccupied(m_vehicle);
        }
      }
      else { // if not aggressive and opposite got vehicle
        double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
        if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
          Vehicle v(intersection.getFrontVehicle(m_vehicle));
          v.totalWaiting += (simulation.currentTime() - v.waitingSince);
          simulation.schedule(new CrossedEvent(ts,v));
          Intersection::setOccupied(m_vehicle);
        }
      }
    }
    else if (queueSize != 0) { // queueSizeLeft is 0, and opposite got vehicle, start them off
      double ts = simulation.currentTime() + parameters.get("INTERSECTION_CROSS_TIME");
      if (std::isless(ts, parameters.get("SIMULATION_CUTOFF_TIME"))) {
        Vehicle v(intersection.getFrontVehicle(m_vehicle));
        v.totalWaiting += (simulation.currentTime() - v.waitingSince);
        simulation.schedule(new CrossedEvent(ts,v));
        Intersection::setOccupied(m_vehicle);
      }
    }
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
  try {
    parameters.parse(argc, argv);
  }
  catch (po::error& pe) {
    fprintf(stderr, "%s", pe.what());
    return 1;
  }

  // Seed the distribution generator.
  distribution.seed(parameters.randomSeed());

  // create a new simulation object
  Simulation simulation;

  // compute timestamp of the first arrival
  double startTime = distribution.exponential(parameters.get("NB_INTER_ARRIVAL_TIME"));

  // create the first vehicle and schedule an arrival event at the start time
  Vehicle firstV = {};
  firstV.id = uniqueId();
  firstV.origin = Street::Tenth;
  firstV.position = Street::Tenth;
  firstV.turndirection = TurnDirection::GO_THROUGH;
  simulation.schedule(new ArrivalEvent(startTime, firstV));

  double startTimeLeft = distribution.exponential(parameters.get("LEFT_INTER_ARRIVAL_TIME"));
  Vehicle firstLeft = {};
  firstLeft.id = uniqueId();
  firstLeft.origin = Street::Eleventh;
  firstLeft.position = Street::Eleventh;

  simulation.schedule(new ArrivalEventLeft(startTimeLeft, firstLeft));

  // Run the simulation.
  simulation.run();

  // print out statistics for all the vehicles that exited the system
  if (exitedVehicles.size() > 0) {
    std::vector<unsigned> count(10, 0);
    fprintf(stdout, "#ID\tOrigin\tDestination\tEntry Time\tExit Time\tWaiting Time\n");
    for (const Vehicle& v : exitedVehicles) {
      count[static_cast<int>(v.exitTime) / 100] += 1;
      fprintf(stdout, "%d\t%d\t%d\t%f\t%f\t%f\n", v.id, v.origin, v.position, v.entryTime, v.exitTime, v.totalWaiting);
    }
  }
  else {
    fprintf(stderr, "No vehicles crossed the stretch! Something went wrong.");
  }

  return 0;
}
