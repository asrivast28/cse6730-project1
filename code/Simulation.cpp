#include "Simulation.hpp"


Simulation::Simulation(
) : m_fel(), m_simtime(0)
{
}

// run the current simulation
void
Simulation::run(
)
{
  while (m_fel.size() > 0) {
    const std::shared_ptr<Event> event(m_fel.top());
    m_fel.pop();
    m_simtime = event->timestamp();
    event->process(*this);
  }
}

// schedule the event at time-stamp, and provide a callback to its handler
void
Simulation::schedule(
  Event* const event
)
{
  m_fel.emplace(event);
}

// returns the current simulation time
double
Simulation::currentTime(
) const
{
	return m_simtime;
}
