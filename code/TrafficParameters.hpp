#ifndef TRAFFICPARAMETERS_HPP_
#define TRAFFICPARAMETERS_HPP_

#include <string>
#include <unordered_map>

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

  double
  get(
    const std::string&
  ) const;

  unsigned
  randomSeed() const { return m_randomSeed; }

  ~TrafficParameters() { }
  
private:
  po::options_description m_desc;
  std::unordered_map<std::string, double> m_paramMap;
  std::string m_paramFile;
  unsigned m_randomSeed;
}; // class TrafficParameters

#endif // TRAFFICPARAMETERS_HPP_
