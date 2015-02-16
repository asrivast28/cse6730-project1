#ifndef TRAFFICOPTIONS_HPP_
#define TRAFFICOPTIONS_HPP_

#include <string>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

/**
 * @brief  Utility class for parsing command line arguments. 
 */
class TrafficOptions {
public:
  TrafficOptions();

  void
  parse(
    int,
    char**
  );

  unsigned
  randomSeed() const { return m_randomSeed; }

  double
  cutoffTime() const { return m_cutoffTime; }

  ~TrafficOptions() { }
  
private:
  po::options_description m_desc;
  double m_cutoffTime;
  unsigned m_randomSeed;
}; // class TrafficOptions

#endif // TRAFFICOPTIONS_HPP_
