#include "Event.hpp"


EventData::EventData(
  const Vehicle& vehicle,
  const bool continued
) : m_vehicle(vehicle),
  m_continued(continued)
{
}

const Vehicle&
EventData::vehicle(
) const
{
  return m_vehicle;
}

bool
EventData::continued(
) const
{
  return m_continued;
}

EventData::~EventData(
)
{
}


Event::Event(
  const double timestamp,
  void (*callback)(const EventData&),
  const EventData& eventData
) : m_timestamp(timestamp),
  m_callback(callback),
  m_eventData(eventData)
{
}

Event::Event(
  const Event& event
) : m_timestamp(event.m_timestamp),
  m_callback(event.m_callback),
  m_eventData(event.m_eventData)
{
}

double
Event::timestamp(
) const
{
  return m_timestamp;
}

void
Event::callback(
) const
{
  m_callback(m_eventData);
}

Event::~Event(
)
{
}

bool
operator<(
  const Event& e1,
  const Event& e2
)
{
  return (e1.m_timestamp < e2.m_timestamp);
}
