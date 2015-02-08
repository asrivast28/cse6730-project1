#include "Event.hpp"

#include <queue>

class SimulationEngine {
public:
  SimulationEngine();

  void
  run();

  void
  schedule(
    double,
    const EventData&,
    void (*) (const EventData&)
  );

  double
  currentTime() const;

private:
  std::priority_queue<Event> m_fel;
  double m_simtime;
}; // class SimulationEngine

SimulationEngine::SimulationEngine(
) : m_fel(), m_simtime(0)
{
}

void
SimulationEngine::run(
)
{
  while (m_fel.size() > 0) {
    const Event event(m_fel.top());
    m_fel.pop();
    m_simtime = event.timestamp();
    event.callback();
  }
}

// schedule the event at time-stamp, and provide a callback to its handler
void
SimulationEngine::schedule(
  double timestamp,
  const EventData& eventData,
  void (*callback) (const EventData&)
)
{
  m_fel.emplace(timestamp, callback, eventData);
}

// returns the current simulation time
double
SimulationEngine::currentTime(
) const
{
	return m_simtime;
}
