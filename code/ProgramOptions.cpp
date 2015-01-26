#include "ProgramOptions.hpp"

#include <sstream>


ProgramOptions::ProgramOptions(
) : m_desc("Traffic simulator options")
{
  po::options_description desc;
  m_desc.add_options()
    ("help,h", "Print this message.")
    ;
}

void
ProgramOptions::parse(
  int argc,
  char** argv
)
{
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, m_desc), vm);
  po::notify(vm);    
  
  //if ((argc == 1) || (vm.count("help") > 0)) {
  if (vm.count("help") > 0) {
    std::stringstream ss;
    ss << m_desc;
    throw po::error(ss.str());
  }
}
