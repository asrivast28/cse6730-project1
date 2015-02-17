#include "TrafficParameters.hpp"

#include <fstream>
#include <sstream>


TrafficParameters::TrafficParameters(
) : m_desc("Traffic simulator options"),
  m_paramMap(),
  m_paramFile(),
  m_randomSeed(std::time(0))
{
  po::options_description desc;
  m_desc.add_options()
    ("help,h", "Print this message.")
    ("parameters", po::value<std::string>(&m_paramFile), "File containing simulation parameters.")
    ("seed", po::value<unsigned>(&m_randomSeed), "Random seed value.")
    ;
}

double
TrafficParameters::get(
  const std::string& key
) const
{
  return m_paramMap.at(key);
}

void
TrafficParameters::parse(
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
  else if (vm.count("parameters") == 0) {
    throw po::error("No parameter file provided! Can't run without a parameter file.");
  }

  std::fstream params(m_paramFile, std::ifstream::in);
  while (params.good()) {
    std::string key;
    double value;
    params >> key >> value;
    m_paramMap[key] = value;
  }
}
