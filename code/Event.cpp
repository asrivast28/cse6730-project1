#include "Event.hpp"


Event::Event(
  const double timestamp
) : m_timestamp(timestamp)
{
}

Event::Event(
  const Event& event
) : m_timestamp(event.m_timestamp)
{
}

double
Event::timestamp(
) const
{
  return m_timestamp;
}

Event::~Event(
)
{
}
