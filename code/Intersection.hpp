#ifndef INTERSECTION_HPP_
#define INTERSECTION_HPP_

#include <cstdlib>
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
  updateSignalStates(
    const double
  );

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
Intersection::getSignalState(
  const Vehicle& v
) const
{
  return m_states[v.currentPosition];
}

#endif // INTERSECTION_HPP_
