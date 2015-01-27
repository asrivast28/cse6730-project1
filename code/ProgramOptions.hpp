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

  ~ProgramOptions() { }
  
private:
  po::options_description m_desc;
}; // class ProgramOptions

#endif // PROGRAMOPTIONS_HPP_
