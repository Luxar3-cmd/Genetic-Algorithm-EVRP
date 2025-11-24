#ifndef POPULATION_HPP
#define POPULATION_HPP

#include "Preprocess.hpp"
#include "Solution.hpp"
#include "Split.hpp"
#include <random>
#include <vector>

using namespace std;

/**
 * @class Population
 * @brief Manejo de población para el algoritmo genético
 */
class Population {
public:
  /**
   * @brief Constructor
   * @param preprocess Módulo de preprocesamiento
   * @param split Módulo de decodificación
   * @param size Tamaño de la población
   * @param fleet_size Tamaño de la flota disponible
   */
  Population(const Preprocess &preprocess, const Split &split, int size,
             int fleet_size);

  /**
   * @brief Inicializa la población con permutaciones aleatorias
   */
  void initialize_random();

  /**
   * @brief Evalúa el fitness de toda la población
   */
  void evaluate();

  /**
   * @brief Añade un cromosoma específico a la población (útil para debug o
   * seeds)
   * @param chromosome Cromosoma a añadir
   */
  void add_chromosome(const vector<int> &chromosome);

  /**
   * @brief Reemplaza el peor individuo si el nuevo es mejor
   * @param chromosome Nuevo cromosoma
   * @param solution Solución decodificada del nuevo cromosoma
   */
  void replace_worst(const vector<int> &chromosome, const Solution &solution);

  // Getters
  const vector<vector<int>> &get_chromosomes() const { return chromosomes_; }
  const vector<Solution> &get_solutions() const { return solutions_; }
  const vector<double> &get_fitness() const { return fitness_; }
  int get_best_index() const;
  int get_worst_index() const;
  /**
   * @brief Obtiene el índice de un individuo aleatorio de la mitad peor de la
   * población
   * @param rng Generador de números aleatorios
   * @return Índice del individuo seleccionado
   */
  int get_random_worst_half_index(mt19937 &rng) const;

  /**
   * @brief Reemplaza el individuo en el índice especificado
   * @param index Índice del individuo a reemplazar
   * @param chromosome Nuevo cromosoma
   * @param solution Solución decodificada del nuevo cromosoma
   */
  void replace_at(int index, const vector<int> &chromosome,
                  const Solution &solution);

  const Solution &get_best_solution() const;

private:
  const Preprocess &preprocess_;
  const Split &split_;
  int size_;
  int fleet_size_;

  vector<vector<int>> chromosomes_;
  vector<Solution> solutions_;
  vector<double> fitness_;

  mt19937 rng_;

  vector<int> generate_random_chromosome();
};

#endif // POPULATION_HPP
