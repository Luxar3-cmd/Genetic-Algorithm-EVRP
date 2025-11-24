#include "Population.hpp"
#include <algorithm>
#include <numeric>

Population::Population(const Preprocess &preprocess, const Split &split,
                       int size, int fleet_size)
    : preprocess_(preprocess), split_(split), size_(size),
      fleet_size_(fleet_size) {
  rng_.seed(std::random_device{}());
}

void Population::initialize_random() {
  chromosomes_.clear();
  solutions_.clear();
  fitness_.clear();

  for (int i = 0; i < size_; i++) {
    chromosomes_.push_back(generate_random_chromosome());
  }

  evaluate();
}

void Population::evaluate() {
  solutions_.clear();
  fitness_.clear();

  for (const auto &chromosome : chromosomes_) {
    Solution sol = split_.decode(chromosome, fleet_size_);
    solutions_.push_back(sol);

    // Fitness = costo total (minimizar), o penalización si infactible
    if (sol.is_feasible) {
      fitness_.push_back(sol.total_cost);
    } else {
      fitness_.push_back(1e9); // Penalización alta
    }
  }
}

void Population::add_chromosome(const vector<int> &chromosome) {
  chromosomes_.push_back(chromosome);
  Solution sol = split_.decode(chromosome, fleet_size_);
  solutions_.push_back(sol);
  fitness_.push_back(sol.is_feasible ? sol.total_cost : 1e9);
}

void Population::replace_worst(const vector<int> &chromosome,
                               const Solution &solution) {
  int worst_idx = get_worst_index();
  chromosomes_[worst_idx] = chromosome;
  solutions_[worst_idx] = solution;
  fitness_[worst_idx] = solution.is_feasible ? solution.total_cost : 1e9;
}

int Population::get_random_worst_half_index(mt19937 &rng) const {
  // Crear vector de índices
  vector<int> indices(size_);
  iota(indices.begin(), indices.end(), 0);

  // Ordenar índices por fitness (menor costo = mejor)
  // Queremos identificar la mitad peor (costos más altos)
  sort(indices.begin(), indices.end(),
       [&](int a, int b) { return fitness_[a] < fitness_[b]; });

  // La mitad peor está en el rango [size/2, size-1]
  int start_worst_half = size_ / 2;
  uniform_int_distribution<int> dist(start_worst_half, size_ - 1);

  return indices[dist(rng)];
}

void Population::replace_at(int index, const vector<int> &chromosome,
                            const Solution &solution) {
  chromosomes_[index] = chromosome;
  solutions_[index] = solution;
  fitness_[index] = solution.is_feasible ? solution.total_cost : 1e9;
}

int Population::get_best_index() const {
  return min_element(fitness_.begin(), fitness_.end()) - fitness_.begin();
}

int Population::get_worst_index() const {
  return max_element(fitness_.begin(), fitness_.end()) - fitness_.begin();
}

const Solution &Population::get_best_solution() const {
  return solutions_[get_best_index()];
}

vector<int> Population::generate_random_chromosome() {
  int P = preprocess_.get_P();
  vector<int> chromosome(P);
  iota(chromosome.begin(), chromosome.end(), 1); // [1, 2, ..., P]
  shuffle(chromosome.begin(), chromosome.end(), rng_);
  return chromosome;
}
