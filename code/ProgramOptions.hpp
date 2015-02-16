#ifndef PROGRAMOPTIONS_HPP_
#define PROGRAMOPTIONS_HPP_

#include <string>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

/**
 * @brief  Utility class for parsing command line arguments. 
 */
class ProgramOptions {
public:
  ProgramOptions();

  void
  parse(
    int,
    char**
  );

  unsigned
  randomSeed() const { return m_randomSeed; }

  double
  cutoffTime() const { return m_cutoffTime; }

  ~ProgramOptions() { }
  
private:
  po::options_description m_desc;
  double m_cutoffTime;
  unsigned m_randomSeed;
}; // class ProgramOptions

#endif // PROGRAMOPTIONS_HPP_
