#include "GA.hpp"
#include "Operators.hpp"
#include "Population.hpp"
#include <iomanip>
#include <iostream>

GA::GA(const string &filename, int battery_step)
    : preprocess_(filename, battery_step), split_(preprocess_) {
  rng_.seed(std::random_device{}());

  // Parámetros por defecto
  pop_size_ = 50;
  generations_ = 100;
  crossover_rate_ = 0.8;
  mutation_rate_ = 0.2;
}

void GA::set_parameters(int pop_size, int generations, double crossover_rate,
                        double mutation_rate) {
  pop_size_ = pop_size;
  generations_ = generations;
  crossover_rate_ = crossover_rate;
  mutation_rate_ = mutation_rate;
}

Solution GA::run() {
  cout << "\n=== Algoritmo Genético EVRP ===" << endl;
  cout << "Población: " << pop_size_ << ", Generaciones: " << generations_
       << endl;
  cout << "Crossover: " << crossover_rate_ << ", Mutación: " << mutation_rate_
       << endl;

  // Crear población inicial
  Population population(preprocess_, split_, pop_size_, preprocess_.get_B());
  population.initialize_random();

  cout << "\nPoblación inicial generada" << endl;
  cout << "Mejor costo inicial: " << population.get_best_solution().total_cost
       << endl;

  uniform_real_distribution<double> prob_dist(0.0, 1.0);

  // Evolución
  for (int gen = 0; gen < generations_; gen++) {
    const auto &chromosomes = population.get_chromosomes();
    const auto &fitness = population.get_fitness();

    // Selección y reproducción (Binary Tournament)
    vector<int> parent1 =
        Operators::tournament_selection(chromosomes, fitness, 2, rng_);
    vector<int> parent2 =
        Operators::tournament_selection(chromosomes, fitness, 2, rng_);

    // Crossover
    vector<int> child1, child2;
    if (prob_dist(rng_) < crossover_rate_) {
      auto children = Operators::order_crossover(parent1, parent2, rng_);
      child1 = children.first;
      child2 = children.second;
    } else {
      child1 = parent1;
      child2 = parent2;
    }

    // Mutación (búsqueda local)
    if (prob_dist(rng_) < mutation_rate_) {
      Operators::local_search_mutation(child1, split_, preprocess_.get_B(), 10,
                                       rng_);
    }
    if (prob_dist(rng_) < mutation_rate_) {
      Operators::local_search_mutation(child2, split_, preprocess_.get_B(), 10,
                                       rng_);
    }

    // Evaluar hijos
    Solution sol1 = split_.decode(child1, preprocess_.get_B());
    Solution sol2 = split_.decode(child2, preprocess_.get_B());

    // Reemplazar peores individuos si los hijos son mejores
    if (sol1.is_feasible &&
        sol1.total_cost < fitness[population.get_worst_index()]) {
      population.replace_worst(child1, sol1);
    }
    if (sol2.is_feasible &&
        sol2.total_cost < fitness[population.get_worst_index()]) {
      population.replace_worst(child2, sol2);
    }

    // Mostrar progreso cada 10 generaciones
    if ((gen + 1) % 10 == 0) {
      cout << "Gen " << setw(3) << (gen + 1) << " | Mejor: " << fixed
           << setprecision(2) << population.get_best_solution().total_cost
           << endl;
    }
  }

  cout << "\n=== Evolución completada ===" << endl;
  const Solution &best = population.get_best_solution();
  cout << "Mejor solución encontrada:" << endl;
  cout << "  Costo total: " << best.total_cost << endl;
  cout << "  Vehículos usados: " << best.num_vehicles << " / "
       << preprocess_.get_B() << endl;
  cout << "  Factible: " << (best.is_feasible ? "SÍ" : "NO") << endl;

  if (best.is_feasible) {
    // Detalle de rutas movido a main.cpp
  }

  return best;
}