#include "Intersection.hpp"


std::vector<bool> Intersection::m_occupied(static_cast<size_t>(Street::Fifteenth));

Intersection::Intersection(
) : m_queue(static_cast<size_t>(Street::Fifteenth)),
  m_groupSize(static_cast<size_t>(Street::Fifteenth)),
  m_states(static_cast<size_t>(Street::Fifteenth))
{
}

void
Intersection::addToQueue(
  const Vehicle& v
)
{
  m_queue[v.position].push(v);
}

unsigned
Intersection::queueSize(
  const Vehicle& v
) const
{
  return m_queue[v.position].size();
}

const Vehicle&
Intersection::viewFrontVehicle(
  const Vehicle& v
) const
{
  return m_queue[v.position].front();
}

Vehicle
Intersection::getFrontVehicle(
  const Vehicle& v
)
{
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

void
Intersection::updateSignalStates(
  const double currentTime
)
{
  double segment = std::remainder(currentTime, 87.6);
  SignalState state = RED;
  if (std::islessequal(segment, 34.7)) {
    state = GREEN_THRU;
  }
  else if (std::islessequal(segment, 38.3)) {
    state = YELLOW;
  }
  for (int s = Street::Tenth; s < Street::Fifteenth; ++s) {
    m_states[s] = state;
  }
}

Intersection::SignalState
Intersection::signalState(
  const Vehicle& v
) const
{
  return m_states[v.position];
}

void
Intersection::setOccupied(
  const Vehicle& v
)
{
  m_occupied[v.position] = true;
}

void
Intersection::setClear(
  const Vehicle& v
)
{
  m_occupied[v.position] = false;
}

bool
Intersection::isClear(
  const Vehicle& v
)
{
  return !m_occupied[v.position];
}
