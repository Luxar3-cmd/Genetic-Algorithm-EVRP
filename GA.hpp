#ifndef GA_HPP
#define GA_HPP

#include "Preprocess.hpp"
#include "Solution.hpp"
#include "Split.hpp"
#include <random>
#include <string>

using namespace std;

/**
 * @class GA
 * @brief Coordinador del algoritmo genético para EVRP
 */
class GA {
public:
  /**
   * @brief Constructor
   * @param filename Archivo de instancia
   * @param battery_step Paso de discretización de batería
   */
  GA(const string &filename, int battery_step = 1);

  /**
   * @brief Configura parámetros del algoritmo genético
   * @param pop_size Tamaño de población
   * @param generations Número de generaciones
   * @param crossover_rate Probabilidad de cruce
   * @param mutation_rate Probabilidad de mutación
   */
  void set_parameters(int pop_size, int generations, double crossover_rate,
                      double mutation_rate);

  /**
   * @brief Ejecuta el algoritmo genético
   * @return Mejor solución encontrada
   */
  Solution run();

  const Preprocess &get_preprocess() const { return preprocess_; }
  const Split &get_split() const { return split_; }

private:
  // Modules
  Preprocess preprocess_;
  Split split_;

  // Parameters
  int pop_size_;
  int generations_;
  double crossover_rate_;
  double mutation_rate_;

  mt19937 rng_;
};

#endif // GA_HPP
