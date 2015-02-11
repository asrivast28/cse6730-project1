#ifndef SIMULATION_HPP_
#define SIMULATION_HPP_

#include "Event.hpp"

#include <queue>

class Simulation {
public:
  Simulation();

  void
  run();

  void
  schedule(
    Event* const
  );

  double
  currentTime() const;

private:
  std::priority_queue<std::shared_ptr<Event>, std::vector<std::shared_ptr<Event> >, EventComparator> m_fel;
  double m_simtime;
}; // class Simulation

#endif // SIMULATION_HPP_
