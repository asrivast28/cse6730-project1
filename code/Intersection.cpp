#include "Intersection.hpp"

#include "TrafficParameters.hpp"

#include <string>

std::vector<bool> Intersection::m_occupied(static_cast<size_t>(Street::Fifteenth));

Intersection::Intersection(
) : m_queue(static_cast<size_t>(Street::Fifteenth)),
  m_groupSize(static_cast<size_t>(Street::Fifteenth))
{
}

void
Intersection::addToQueue(
  const Vehicle& v
)
{
  assert(v.position >= Street::Tenth);
  m_queue[v.position].push(v);
}

unsigned
Intersection::queueSize(
  const Vehicle& v
) const
{
  assert(v.position >= Street::Tenth);
  return m_queue[v.position].size();
}

const Vehicle&
Intersection::viewFrontVehicle(
  const Vehicle& v
) const
{
  assert(v.position >= Street::Tenth);
  assert(m_queue[v.position].size() > 0);
  return m_queue[v.position].front();
}

Vehicle
Intersection::getFrontVehicle(
  const Vehicle& v
)
{
  assert(v.position >= Street::Tenth);
  assert(m_queue[v.position].size() > 0);
  Vehicle frontV(m_queue[v.position].front());
  m_queue[v.position].pop();
  return frontV;
}

void
Intersection::increaseGroupSize(
  const Vehicle& v
)
{
  m_groupSize[v.position] += 1;
}

void
Intersection::decreaseGroupSize(
  const Vehicle& v
)
{
  m_groupSize[v.position] -= 1;
}

unsigned
Intersection::groupSize(
  const Vehicle& v
) const
{
  return m_groupSize[v.position];
}

Intersection::SignalState
Intersection::getSignalState(
  const Vehicle& v,
  const double currentTime,
  const TrafficParameters& parameters
)
{
  assert(v.position >= Street::Tenth);
  std::string intersection;
  switch (v.position) {
    case Street::Tenth:
      intersection = "TENTH";
      break;
    case Street::Eleventh:
      intersection = "ELEVENTH";
      break;
    case Street::Twelfth:
      intersection = "TWELFTH";
      break;
    case Street::Thirteenth:
      intersection = "THIRTEENTH";
      break;
    case Street::Fourteenth:
      intersection = "FOURTEENTH";
      break;
    default:
      throw std::runtime_error("Outside the SUI!");
  }
  double segment = std::remainder(currentTime, parameters.get(intersection + "_INTERSECTION_TOTAL"));
  SignalState state = RED;
  if (v.position == Street::Thirteenth) {
    state = GREEN_THRU;
  }
  else if (std::islessequal(segment, parameters.get(intersection + "_INTERSECTION_GREEN"))) {
    state = GREEN_THRU;
  }
  else if (std::islessequal(segment, parameters.get(intersection + "_INTERSECTION_YELLOW"))) {
    state = YELLOW;
  }
  return state;
}

void
Intersection::setOccupied(
  const Vehicle& v
)
{
  assert(v.position >= Street::Tenth);
  m_occupied[v.position] = true;
}

void
Intersection::setClear(
  const Vehicle& v
)
{
  assert(v.position >= Street::Tenth);
  m_occupied[v.position] = false;
}

bool
Intersection::isClear(
  const Vehicle& v
)
{
  assert(v.position >= Street::Tenth);
  return !m_occupied[v.position];
}
