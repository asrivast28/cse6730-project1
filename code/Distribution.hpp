#ifndef DISTRIBUTION_HPP_
#define DISTRIBUTION_HPP_

#include <cmath>
#include <cstdlib>


/*
 * Generate a uniform random number in the range [0,1)
 */
double urand(void) {
	double x;
  do {
    x = std::rand() / static_cast<double>(RAND_MAX);
  } while (std::isgreaterequal(x, 1.0));		// loop until x < 1
	return x;
}

/*
 * Generate a random number from an exponential distribution
 * with a given mean.
 *
 * @param mean the mean of the distribution
 *
 * @return a number draw from exponential distribution
 */
double randexp(double mean) {
	return (-1 * mean) * (log(1.0 - urand()));
}

#endif // DISTRIBUTION_HPP_
