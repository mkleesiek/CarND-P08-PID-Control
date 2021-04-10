/**
 * @author marco.kleesiek@gmail.com
 */

#pragma once

#include <vector>
#include <numeric>
#include <cassert>

/**
 * Function minimizer using the "Twiddle" algorithm.
 * @tparam ObjectiveT Functor type of the objective function to be evaluated.
 */
template<typename ObjectiveT>
class TwiddleOptimizer {
public:
  /**
   * Constructor.
   * @param objective Reference to the objective function functor.
   */
  TwiddleOptimizer(ObjectiveT& objective);
  /**
   * Destructor.
   */
  virtual ~TwiddleOptimizer() = default;

  /**
   * Find the objective function minimum, using the "Twiddle" algorithm.
   * @param[in,out] params Initial values for the objective function arguments.
   * @param deltas Initial step sizes regarding @p params for probing the function parameter space.
   * @param threshold The threshold defining the break condition for the minimizer.
   * @return Returns true when the sum of parameter deltas has been reduced below the @p threshold.
   *   The final function argument values can be read from @p params.
   */
  bool Minimize(std::vector<double>& params, std::vector<double> deltas, double threshold = 0.001);

private:
  ObjectiveT& m_objective;
};

template<typename ObjectiveT>
TwiddleOptimizer<ObjectiveT>::TwiddleOptimizer(ObjectiveT& objective)
: m_objective{objective}
{ }

template<typename ObjectiveT>
bool TwiddleOptimizer<ObjectiveT>::Minimize(std::vector<double>& params, std::vector<double> deltas, double threshold)
{
  assert(!params.empty() && params.size() == deltas.size()
    && "Number of function arguments must not be zero and must match the number of deltas.");

  double best_error = m_objective(params);

  while (std::accumulate(deltas.begin(), deltas.end(), 0.0) > threshold) {

    for (size_t i = 0; i < params.size(); i++) {

      if (deltas[i] == 0.0)
      {
        continue;
      }

      params[i] += deltas[i];
      double error = m_objective(params);

      // there was some improvement
      if (error < best_error)
      {
        best_error = error;
        deltas[i] *= 1.1;
      }
      // there was no improvement
      else
      {
        // go into the other direction
        params[i] -= 2.0 * deltas[i];
        error = m_objective(params);

        // there was some improvement
        if (error < best_error)
        {
            best_error = error;
            deltas[i] *= 1.1;
        }
        // there was no improvement
        else
        {
            params[i] += deltas[i];
            // since there was no improvement in either direction, the step size might be too big:
            deltas[i] *= 0.9;
        }
      }
    }
  }

  return true;
}
