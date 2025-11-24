#ifndef OPERATORS_HPP
#define OPERATORS_HPP

#include <random>
#include <vector>

// Forward declarations
class Split;
struct Solution;

using namespace std;

/**
 * @class Operators
 * @brief Operadores genéticos para el algoritmo evolutivo
 */
class Operators {
public:
  /**
   * @brief Selección por torneo binario (tamaño fijo = 2)
   */
  static vector<int> tournament_selection(const vector<vector<int>> &population,
                                          const vector<double> &fitness,
                                          mt19937 &rng);

  /**
   * @brief Order Crossover (OX)
   */
  static pair<vector<int>, vector<int>>
  order_crossover(const vector<int> &parent1, const vector<int> &parent2,
                  mt19937 &rng);

  /**
   * @brief Mutación por búsqueda local con 3 movimientos
   *
   * Intenta mejorar el cromosoma explorando 3 vecindarios:
   * - Swap: Intercambiar dos clientes
   * - 2-opt: Invertir un segmento
   * - Reassign: Mover un cliente a otra posición
   *
   * Si encuentra mejora en max_iterations, retorna el mejor.
   * Si no encuentra mejora, retorna el cromosoma original.
   *
   * @param chromosome Cromosoma a mejorar
   * @param split Decodificador para evaluar
   * @param fleet_size Tamaño de flota
   * @param max_iterations Número máximo de iteraciones sin mejora
   * @param rng Generador de números aleatorios
   */
  static void local_search_mutation(vector<int> &chromosome, const Split &split,
                                    int fleet_size, int max_iterations,
                                    mt19937 &rng);

private:
  // Movimientos de vecindario
  static void apply_swap(vector<int> &chromosome, int i, int j);
  static void apply_2opt(vector<int> &chromosome, int i, int j);
  static void apply_reassign(vector<int> &chromosome, int from, int to);
};

#endif // OPERATORS_HPP
