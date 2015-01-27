#include "ProgramOptions.hpp"

#include <iostream>

/**
 * @brief Structure for storing all the information for a vehicle.
 */
typedef struct {
  unsigned id; // ID of the vehicle.
  unsigned originatingZone; // Originating zone of the vehicle.
  unsigned destinationZone; // Destination zone of the vehicle.
  unsigned startTime; // Time at which the vehicle entered the area of interest.
  unsigned endTime; // Time at which the vehicle exited from the area of interest.
  double startWaiting; // Time at which the vehicle started waiting at the previous intersection.
  double endWaiting; // Time at which the vehicle passed the previous intersection. 
} Vehicle;

/**
 * @brief  Main function which is called to start the simulator.  
 *
 * @param argc  Number of arguments passed to the simulator.
 * @param argv  Actual arguments passed to the simulator.
 *
 * @return Returns 0 if successful else returns a positive error code. 
 */
int
main(
  int argc,
  char** argv
)
{
  ProgramOptions options;
  try {
    options.parse(argc, argv);
  }
  catch (po::error& pe) {
    std::cerr << pe.what() << std::endl;
    return 1;
  }

  return 0;
}
