#ifndef TRAFFICPARAMETERS_HPP_
#define TRAFFICPARAMETERS_HPP_

#include <string>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

/**
 * @brief  Utility class for parsing command line arguments. 
 */
class TrafficParameters {
public:
  TrafficParameters();

  void
  parse(
    int,
    char**
  );

  unsigned
  randomSeed() const { return m_randomSeed; }

  double
  cutoffTime() const { return m_cutoffTime; }

  ~TrafficParameters() { }
  
private:
  po::options_description m_desc;
  double m_cutoffTime;
  unsigned m_randomSeed;
}; // class TrafficParameters

#endif // TRAFFICPARAMETERS_HPP_
