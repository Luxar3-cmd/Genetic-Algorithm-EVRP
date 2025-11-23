#include "Operators.hpp"
#include "Solution.hpp"
#include "Split.hpp"
#include <algorithm>

vector<int>
Operators::tournament_selection(const vector<vector<int>> &population,
                                const vector<double> &fitness,
                                int tournament_size, mt19937 &rng) {

  uniform_int_distribution<int> dist(0, population.size() - 1);

  int best_idx = dist(rng);
  double best_fitness = fitness[best_idx];

  for (int i = 1; i < tournament_size; i++) {
    int idx = dist(rng);
    if (fitness[idx] < best_fitness) {
      best_idx = idx;
      best_fitness = fitness[idx];
    }
  }

  return population[best_idx];
}

pair<vector<int>, vector<int>>
Operators::order_crossover(const vector<int> &parent1,
                           const vector<int> &parent2, mt19937 &rng) {

  int n = parent1.size();
  uniform_int_distribution<int> dist(0, n - 1);

  int point1 = dist(rng);
  int point2 = dist(rng);
  if (point1 > point2)
    swap(point1, point2);

  vector<int> child1(n, -1), child2(n, -1);

  // Copiar segmento central
  for (int i = point1; i <= point2; i++) {
    child1[i] = parent1[i];
    child2[i] = parent2[i];
  }

  // Llenar el resto manteniendo orden
  auto fill_child = [](vector<int> &child, const vector<int> &parent, int p1,
                       int p2) {
    int n = child.size();
    int pos = (p2 + 1) % n;
    for (int i = (p2 + 1) % n; i != p1; i = (i + 1) % n) {
      int gene = parent[i];
      if (find(child.begin(), child.end(), gene) == child.end()) {
        while (child[pos] != -1)
          pos = (pos + 1) % n;
        child[pos] = gene;
        pos = (pos + 1) % n;
      }
    }
  };

  fill_child(child1, parent2, point1, point2);
  fill_child(child2, parent1, point1, point2);

  return {child1, child2};
}

void Operators::local_search_mutation(vector<int> &chromosome,
                                      const Split &split, int fleet_size,
                                      int max_iterations, mt19937 &rng) {

  int n = chromosome.size();
  if (n < 2)
    return;

  // Evaluar cromosoma actual
  Solution current_sol = split.decode(chromosome, fleet_size);
  double current_cost = current_sol.is_feasible ? current_sol.total_cost : 1e9;

  vector<int> best_chromosome = chromosome;
  double best_cost = current_cost;

  int iterations_without_improvement = 0;
  uniform_int_distribution<int> dist(0, n - 1);
  uniform_int_distribution<int> move_dist(0, 2); // 3 movimientos

  while (iterations_without_improvement < max_iterations) {
    vector<int> neighbor = best_chromosome;

    // Seleccionar movimiento aleatorio
    int move_type = move_dist(rng);
    int i = dist(rng);
    int j = dist(rng);

    switch (move_type) {
    case 0: // Swap
      if (i != j) {
        apply_swap(neighbor, i, j);
      }
      break;

    case 1: // 2-opt
      if (i < j && j - i > 1) {
        apply_2opt(neighbor, i, j);
      }
      break;

    case 2: // Reassign
      if (i != j) {
        apply_reassign(neighbor, i, j);
      }
      break;
    }

    // Evaluar vecino
    Solution neighbor_sol = split.decode(neighbor, fleet_size);
    double neighbor_cost =
        neighbor_sol.is_feasible ? neighbor_sol.total_cost : 1e9;

    // Si encontramos mejora
    if (neighbor_cost < best_cost) {
      best_chromosome = neighbor;
      best_cost = neighbor_cost;
      iterations_without_improvement = 0;
    } else {
      iterations_without_improvement++;
    }
  }

  // Retornar el mejor encontrado (o el original si no hubo mejora)
  chromosome = best_chromosome;
}

void Operators::apply_swap(vector<int> &chromosome, int i, int j) {
  swap(chromosome[i], chromosome[j]);
}

void Operators::apply_2opt(vector<int> &chromosome, int i, int j) {
  // Invertir segmento [i+1, j]
  reverse(chromosome.begin() + i + 1, chromosome.begin() + j + 1);
}

void Operators::apply_reassign(vector<int> &chromosome, int from, int to) {
  // Mover cliente de posición 'from' a posición 'to'
  int client = chromosome[from];
  chromosome.erase(chromosome.begin() + from);

  // Ajustar 'to' si es necesario
  if (from < to)
    to--;

  chromosome.insert(chromosome.begin() + to, client);
}
