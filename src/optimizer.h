#pragma once

#include <vector>
#include <numeric>
#include <random>

template<typename ObjectiveT>
class TwiddleOptimizer {
public:
  TwiddleOptimizer(ObjectiveT& objective);
  virtual ~TwiddleOptimizer() = default;

  bool Minimize(std::vector<double>& params, double stepsize = 0.2, double threshold = 0.01);

private:
  ObjectiveT& m_objective;
  std::vector<double> m_params;
  std::vector<double> m_deltas;
  double m_best_error = 0.0;
};

template<typename ObjectiveT>
TwiddleOptimizer<ObjectiveT>::TwiddleOptimizer(ObjectiveT& objective)
: m_objective{objective}
{ }

template<typename ObjectiveT>
bool TwiddleOptimizer<ObjectiveT>::Minimize(std::vector<double>& params, double stepsize, double threshold)
{
  std::vector<double> deltas(params.size());
  std::transform(params.begin(), params.end(), deltas.begin(), [&](double param) {
    return stepsize * param;
  });

  double best_error = m_objective(params);

  // initialize random number generator
  std::default_random_engine generator{std::random_device{}()};
  std::uniform_int_distribution<size_t> param_dist{0, params.size()-1};

  while (std::accumulate(deltas.begin(), deltas.end(), 0.0) > threshold) {

    // randomly choose the next parameter to adjust
    size_t i = param_dist(generator);
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
          deltas[i] *= 1.05;
      }
      // there was no improvement
      else
      {
          params[i] += deltas[i];
          // since there was no improvement in either direction, the stepsize might be too big:
          deltas[i] *= 0.95;
      }
    }
  }

  return true;
}
