#include "TrafficOptions.hpp"

#include <sstream>


TrafficOptions::TrafficOptions(
) : m_desc("Traffic simulator options"),
  m_cutoffTime(14400),
  m_randomSeed(std::time(0))
{
  po::options_description desc;
  m_desc.add_options()
    ("help,h", "Print this message.")
    ("seed", po::value<unsigned>(&m_randomSeed), "Random seed value.")
    ("time", po::value<double>(&m_cutoffTime), "Cutoff time (in seconds).")
    ;
}

void
TrafficOptions::parse(
  int argc,
  char** argv
)
{
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, m_desc), vm);
  po::notify(vm);    
  
  if (vm.count("help") > 0) {
    std::stringstream ss;
    ss << m_desc;
    throw po::error(ss.str());
  }
}
