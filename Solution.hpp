#ifndef SOLUTION_HPP
#define SOLUTION_HPP

#include <string>
#include <vector>

using namespace std;

/**
 * @struct Route
 * @brief Representa una ruta individual en la solución EVRP
 */
struct Route {
  vector<int> clients; // Secuencia de node_ids de clientes en esta ruta
  double cost;   // Costo total de la ruta (incluye estaciones automáticamente)
  int load;      // Carga total acumulada en la ruta
  int recharges; // Número de recargas en la ruta

  Route() : cost(0.0), load(0), recharges(0) {}
};

/**
 * @enum FeasibilityReason
 * @brief Razones por las que una solución puede ser infactible
 */
enum class FeasibilityReason {
  Feasible,            // Solución factible
  CapacityExceeded,    // Se excede la capacidad Q de los vehículos
  FleetExceeded,       // Se excede el número máximo B de vehículos
  BatteryInsufficient, // Batería insuficiente
  UnreachableClient,   // Algún cliente es inalcanzable
  Unknown              // Razón desconocida
};

/**
 * @struct Solution
 * @brief Representa una solución completa al EVRP
 */
struct Solution {
  vector<Route> routes; // Conjunto de rutas
  double total_cost;    // Costo total de la solución
  int num_vehicles;     // Número de vehículos utilizados
  bool is_feasible;     // true si respeta todas las restricciones
  FeasibilityReason infeasibility_reason; // Razón de infactibilidad
  string infeasibility_details;           // Detalles adicionales

  Solution()
      : total_cost(0.0), num_vehicles(0), is_feasible(false),
        infeasibility_reason(FeasibilityReason::Unknown),
        infeasibility_details("") {}
};

/**
 * @brief Convierte FeasibilityReason a string para debugging
 */
string feasibility_reason_to_string(FeasibilityReason reason);

#endif // SOLUTION_HPP
