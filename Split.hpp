#ifndef SPLIT_HPP
#define SPLIT_HPP

#include "Preprocess.hpp"
#include "Solution.hpp"
// #include <cmath>
#include <limits>
#include <vector>

using namespace std;

/**
 * @class Split
 * @brief Decodificador de cromosomas para EVRP con batería residual
 *
 * Implementa el algoritmo split adaptado con DP multidimensional para
 * considerar el estado de batería residual al final de cada ruta.
 *
 * DP Multidimensional:
 * V[i][b] = costo mínimo para atender clientes 0..i-1, terminando con batería b
 *
 * Diferencia con split clásico:
 * - Clásico: V[i] (1D)
 * - Con batería: V[i][b] (2D)
 */
class Split {
public:
  /**
   * @brief Constructor
   * @param preprocess Referencia al módulo de preprocesamiento
   */
  explicit Split(const Preprocess &preprocess);

  /**
   * @brief Decodifica un cromosoma en una solución EVRP
   * @param chromosome Permutación de clientes [1..P]
   * @param fleet_size Número máximo de vehículos disponibles
   * @return Solución con rutas óptimas
   */
  Solution decode(const std::vector<int> &chromosome, int fleet_size) const;

  /**
   * @brief Reconstruye la ruta completa incluyendo estaciones de recarga
   * @param route_clients Secuencia de clientes en la ruta
   * @return Vector con la secuencia completa de nodos (incluyendo D y R)
   */
  std::vector<int>
  get_detailed_path(const std::vector<int> &route_clients) const;

private:
  const Preprocess &preprocess_;

  /**
   * @struct DPState
   * @brief Estado en el DP multidimensional
   */
  struct DPState {
    double cost;      // Costo acumulado
    int pred_pos;     // Posición del predecesor en el cromosoma
    int pred_battery; // Batería del predecesor

    DPState()
        : cost(numeric_limits<double>::infinity()), pred_pos(-1),
          pred_battery(-1) {}
  };

  /**
   * @brief Calcula el costo de una ruta [start..end] considerando batería
   * @param chromosome Cromosoma completo
   * @param start Posición inicial (inclusive)
   * @param end Posición final (inclusive)
   * @param[out] battery_at_end Batería al regresar al depósito
   * @param[out] total_recharges Número de recargas en la ruta
   * @return Costo de la ruta, o INF si es infactible
   */
  double calculate_route_cost(const vector<int> &chromosome, int start, int end,
                              int &battery_at_end, int &total_recharges) const;

  /**
   * @brief Reconstruye la solución desde el DP
   * @param chromosome Cromosoma original
   * @param V Matriz DP [posición][batería]
   * @param n Número de clientes
   * @param fleet_size Tamaño de flota
   * @return Solución reconstruida
   */
  Solution reconstruct_solution(const vector<int> &chromosome,
                                const vector<vector<DPState>> &V, int n,
                                int fleet_size) const;
};

#endif // SPLIT_HPP
