#ifndef INTERSECTION_HPP_
#define INTERSECTION_HPP_

#include <cstdlib>
#include <queue>
#include <vector>


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
  addToQueue(
    const Vehicle&
  );

  unsigned
  queueSize(
    const Vehicle&
  ) const;

  Vehicle
  frontVehicle(
    const Vehicle&
  );

  void
  increaseGroupSize(
    const Vehicle&
  );

  void
  decreaseGroupSize(
    const Vehicle&
  );

  unsigned
  groupSize(
    const Vehicle&
  ) const;

  void
  updateSignalStates(
    const double
  );

  Intersection::SignalState
  signalState(
    const Vehicle&
  ) const;

private:
  std::vector<std::queue<Vehicle> > m_queue;
  std::vector<unsigned> m_groupSize;
  std::vector<SignalState> m_states;
}; // class IntersectionState

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
  m_queue[v.currentPosition].push(v);
}

unsigned
Intersection::queueSize(
  const Vehicle& v
) const
{
  return m_queue[v.currentPosition].size();
}

Vehicle
Intersection::frontVehicle(
  const Vehicle& v
)
{
  Vehicle frontV(m_queue[v.currentPosition].front());
  m_queue[v.currentPosition].pop();
  return frontV;
}

void
Intersection::increaseGroupSize(
  const Vehicle& v
)
{
  m_groupSize[v.currentPosition] += 1;
}

void
Intersection::decreaseGroupSize(
  const Vehicle& v
)
{
  m_groupSize[v.currentPosition] -= 1;
}

unsigned
Intersection::groupSize(
  const Vehicle& v
) const
{
  return m_groupSize[v.currentPosition];
}

void
Intersection::updateSignalStates(
  const double currentTime
)
{
	int segement = currentTime / 10;
	int residual = segement % 2;
  SignalState state = (residual == 0) ? GREEN_THRU : RED;
  for (int s = Street::Tenth; s < Street::Fifteenth; ++s) {
    m_states[s] = state;
  }
}

Intersection::SignalState
Intersection::signalState(
  const Vehicle& v
) const
{
  return m_states[v.currentPosition];
}

#endif // INTERSECTION_HPP_
