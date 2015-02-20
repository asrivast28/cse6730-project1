#ifndef DISTRIBUTION_HPP_
#define DISTRIBUTION_HPP_

#include <cmath>
#include <cstdlib>
#include <random>

/**
 * @brief Class for generating random number for different distributions.
 */
class Distribution {
public:
  Distribution()
    : m_engine() 
  {
  }

  void
  seed(
    const unsigned number
  )
  {
    m_engine.seed(number);
  }

  double
  urand(
    const double a = 0.0,
    const double b = 1.0
  )
  {
    return std::uniform_real_distribution<double>(a, b)(m_engine);
  }

  double
  normal(
    const double mean
  )
  {
    return std::normal_distribution<double>()(m_engine);
  }

  double
  exponential(
    const double mean 
  )
  {
    return (-1.0 * mean) * (log(1.0 - urand()));
  }

private:
  std::default_random_engine m_engine;
};

#endif // DISTRIBUTION_HPP_
