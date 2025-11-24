/**
 * @file Split.cpp
 * @brief Implementación del decodificador Split con DP multidimensional para
 * batería
 *
 * Este módulo implementa el algoritmo split adaptado para EVRP con restricción
 * de batería residual, usando programación dinámica bidimensional V[i][b].
 */

#include "Split.hpp"
#include <cmath>
#include <limits>
#include <queue>

using namespace std;

Split::Split(const Preprocess &preprocess) : preprocess_(preprocess) {}

Solution Split::decode(const vector<int> &chromosome, int fleet_size) const {
  int n = chromosome.size();
  // const double INF = numeric_limits<double>::infinity();
  int B_levels = preprocess_.get_B_levels();

  // DP Multidimensional: V[posición][batería]
  vector<vector<DPState>> V(n + 1, vector<DPState>(B_levels));

  // Estado inicial: posición 0, batería llena (índice B_levels-1)
  int b_full = B_levels - 1;
  V[0][b_full].cost = 0.0;
  V[0][b_full].pred_pos = -1;
  V[0][b_full].pred_battery = -1;

  // DP: construir rutas
  for (int i = 0; i < n; i++) {
    // Para cada nivel de batería en posición i
    for (int b_start = 0; b_start < B_levels; b_start++) {
      if (!isfinite(V[i][b_start].cost))
        continue; // Estado inalcanzable

      int load = 0; // Carga acumulada

      // Intentar extender ruta desde i hasta j
      for (int j = i; j < n; j++) {
        int client_id = chromosome[j];
        load += preprocess_.get_demand()[client_id - 1];

        // Verificar capacidad, si la carga supera Q, no podemos extender la
        // ruta.
        if (load > preprocess_.get_Q())
          break;

        // Calcular costo de ruta [i..j]
        int battery_at_end;
        int recharges;
        double route_cost =
            calculate_route_cost(chromosome, i, j, battery_at_end, recharges);

        // Si la ruta es infactible (battery_at_end < 0)
        if (!isfinite(route_cost) || battery_at_end < 0) {
          break;
        } // Así no se actualiza el DP

        // Actualizar DP
        double new_cost = V[i][b_start].cost + route_cost;
        if (new_cost < V[j + 1][battery_at_end].cost) {
          V[j + 1][battery_at_end].cost = new_cost;
          V[j + 1][battery_at_end].pred_pos = i;
          V[j + 1][battery_at_end].pred_battery = b_start;
        }
      }
    }
  }

  // Reconstruir solución
  return reconstruct_solution(chromosome, V, n, fleet_size);
}

double Split::calculate_route_cost(const vector<int> &chromosome, int start,
                                   int end, int &battery_at_end,
                                   int &total_recharges) const {

  const double INF = numeric_limits<double>::infinity();
  const auto &W = preprocess_.get_W();
  const auto &Rcnt = preprocess_.get_Rcnt();
  const auto &FinalBattery = preprocess_.get_FinalBattery();

  const auto &W_safe = preprocess_.get_W_safe();
  const auto &Rcnt_safe = preprocess_.get_Rcnt_safe();
  const auto &FinalBattery_safe = preprocess_.get_FinalBattery_safe();

  int B_levels = preprocess_.get_B_levels();
  int b_full = B_levels - 1; // Índice de batería llena

  // Estructura para búsqueda
  struct State {
    int index; // Índice en el cromosoma (start..end)
    int battery;
    double cost;
    int recharges;

    bool operator>(const State &other) const { return cost > other.cost; }
  };

  // Usar una cola de prioridad para encontrar el camino de costo mínimo
  // que sea factible.
  // Dado que el número de clientes en una ruta es pequeño (limitado por Q),
  // esto es eficiente.
  priority_queue<State, vector<State>, greater<State>> pq;

  // Inicialización
  // Salimos del depósito (0) hacia el primer cliente
  int first_client = chromosome[start];
  int u_depot = 0;
  int u_first = first_client;

  // Probar ambas estrategias para el primer tramo
  // 1. Min Cost
  if (isfinite(W[u_depot][u_first][b_full])) {
    int bat = FinalBattery[u_depot][u_first][b_full];
    if (bat >= 0) {
      pq.push({start, bat, W[u_depot][u_first][b_full],
               Rcnt[u_depot][u_first][b_full]});
    }
  }
  // 2. Max Battery (Safe)
  if (isfinite(W_safe[u_depot][u_first][b_full])) {
    int bat = FinalBattery_safe[u_depot][u_first][b_full];
    if (bat >= 0) {
      // Evitar duplicados si son iguales
      if (abs(W_safe[u_depot][u_first][b_full] - W[u_depot][u_first][b_full]) >
              1e-6 ||
          bat != FinalBattery[u_depot][u_first][b_full]) {
        pq.push({start, bat, W_safe[u_depot][u_first][b_full],
                 Rcnt_safe[u_depot][u_first][b_full]});
      }
    }
  }

  // Mejor costo encontrado para llegar al final
  double best_total_cost = INF;
  int best_total_recharges = 0;
  bool found_solution = false;

  // Visited array para evitar ciclos o estados redundantes
  // State space: index (start..end) x battery (0..B).
  // Podemos usar una matriz de mejores costos.
  vector<vector<double>> min_cost_to(end - start + 2,
                                     vector<double>(B_levels, INF));

  while (!pq.empty()) {
    State current = pq.top();
    pq.pop();

    int idx = current.index; // Índice del cliente actual en chromosome

    // Si el costo es peor que el mejor encontrado hasta ahora para este estado,
    // skip (Ajuste: index relativo 0..len)
    int rel_idx = idx - start;
    if (current.cost >= min_cost_to[rel_idx][current.battery])
      continue;
    min_cost_to[rel_idx][current.battery] = current.cost;

    // Si ya encontramos una solución completa mejor, podar
    if (found_solution && current.cost >= best_total_cost)
      continue;

    // Si llegamos al último cliente, intentar volver al depósito
    if (idx == end) {
      int u_last = chromosome[end];

      // Intentar volver con Min Cost
      if (isfinite(W[u_last][u_depot][current.battery])) {
        int final_bat = FinalBattery[u_last][u_depot][current.battery];
        if (final_bat >= 0) {
          double total = current.cost + W[u_last][u_depot][current.battery];
          if (total < best_total_cost) {
            best_total_cost = total;
            best_total_recharges =
                current.recharges + Rcnt[u_last][u_depot][current.battery];
            found_solution = true;
          }
        }
      }

      // Intentar volver con Max Battery (Safe) - aunque aquí solo importa
      // llegar
      if (isfinite(W_safe[u_last][u_depot][current.battery])) {
        int final_bat = FinalBattery_safe[u_last][u_depot][current.battery];
        if (final_bat >= 0) {
          double total =
              current.cost + W_safe[u_last][u_depot][current.battery];
          if (total < best_total_cost) {
            best_total_cost = total;
            best_total_recharges =
                current.recharges + Rcnt_safe[u_last][u_depot][current.battery];
            found_solution = true;
          }
        }
      }
      continue;
    }

    // Extender al siguiente cliente
    int u_curr = chromosome[idx];
    int u_next = chromosome[idx + 1];

    // Opción 1: Min Cost
    if (isfinite(W[u_curr][u_next][current.battery])) {
      int next_bat = FinalBattery[u_curr][u_next][current.battery];
      if (next_bat >= 0) {
        pq.push({idx + 1, next_bat,
                 current.cost + W[u_curr][u_next][current.battery],
                 current.recharges + Rcnt[u_curr][u_next][current.battery]});
      }
    }

    // Opción 2: Max Battery
    if (isfinite(W_safe[u_curr][u_next][current.battery])) {
      int next_bat = FinalBattery_safe[u_curr][u_next][current.battery];
      if (next_bat >= 0) {
        // Evitar duplicados
        if (abs(W_safe[u_curr][u_next][current.battery] -
                W[u_curr][u_next][current.battery]) > 1e-6 ||
            next_bat != FinalBattery[u_curr][u_next][current.battery]) {
          pq.push(
              {idx + 1, next_bat,
               current.cost + W_safe[u_curr][u_next][current.battery],
               current.recharges + Rcnt_safe[u_curr][u_next][current.battery]});
        }
      }
    }
  }

  if (!found_solution) {
    battery_at_end = -1;
    return INF;
  }

  battery_at_end = b_full;
  total_recharges = best_total_recharges;
  return best_total_cost;
}

Solution Split::reconstruct_solution(const vector<int> &chromosome,
                                     const vector<vector<DPState>> &V, int n,
                                     int fleet_size) const {

  Solution solution;
  const double INF = numeric_limits<double>::infinity();
  int B_levels = preprocess_.get_B_levels();

  // Buscar el mejor estado final (posición n, cualquier batería)
  double best_cost = INF;
  int best_battery = -1;

  for (int b = 0; b < B_levels; b++) {
    if (V[n][b].cost < best_cost) {
      best_cost = V[n][b].cost;
      best_battery = b;
    }
  }

  // Si no hay solución factible
  if (!isfinite(best_cost) || best_battery < 0) {
    solution.is_feasible = false;
    solution.infeasibility_reason = FeasibilityReason::UnreachableClient;
    solution.infeasibility_details =
        "No se pudo construir ninguna solución factible";
    return solution;
  }

  // Reconstruir rutas en reversa
  vector<Route> routes_reversed;
  int pos = n;
  int bat = best_battery;

  while (pos > 0) {
    int pred_pos = V[pos][bat].pred_pos;
    int pred_bat = V[pos][bat].pred_battery;

    if (pred_pos < 0)
      break; // Llegamos al inicio

    // Construir ruta [pred_pos..pos-1]
    Route route;
    for (int i = pred_pos; i < pos; i++) {
      route.clients.push_back(chromosome[i]);
      route.load += preprocess_.get_demand()[chromosome[i] - 1];
    }

    // Calcular costo de la ruta
    int battery_end, recharges;
    route.cost = calculate_route_cost(chromosome, pred_pos, pos - 1,
                                      battery_end, recharges);
    route.recharges = recharges;

    routes_reversed.push_back(route);

    pos = pred_pos;
    bat = pred_bat;
  }

  // Invertir rutas para orden correcto
  solution.routes.assign(routes_reversed.rbegin(), routes_reversed.rend());
  solution.num_vehicles = solution.routes.size();
  solution.total_cost = best_cost;

  // Verificar restricción de flota
  if (solution.num_vehicles > fleet_size) {
    solution.is_feasible = false;
    solution.infeasibility_reason = FeasibilityReason::FleetExceeded;
    solution.infeasibility_details =
        "Se necesitan " + to_string(solution.num_vehicles) +
        " vehículos pero solo hay " + to_string(fleet_size);
  } else {
    solution.is_feasible = true;
    solution.infeasibility_reason = FeasibilityReason::Feasible;
  }

  return solution;
}

vector<int> Split::get_detailed_path(const vector<int> &route_clients) const {
  vector<int> full_path;
  if (route_clients.empty())
    return full_path;

  const auto &W = preprocess_.get_W();
  const auto &FinalBattery = preprocess_.get_FinalBattery();
  const auto &PathUV = preprocess_.get_PathUV();

  const auto &W_safe = preprocess_.get_W_safe();
  const auto &FinalBattery_safe = preprocess_.get_FinalBattery_safe();
  const auto &PathUV_safe = preprocess_.get_PathUV_safe();

  int B_levels = preprocess_.get_B_levels();
  int M = preprocess_.get_U().size();
  int b_full = B_levels - 1;

  // Estructura para búsqueda (similar a calculate_route_cost pero guardando el
  // path)
  struct State {
    int index; // Índice en route_clients
    int battery;
    double cost;
    vector<int> path_so_far; // Path acumulado

    bool operator>(const State &other) const { return cost > other.cost; }
  };

  priority_queue<State, vector<State>, greater<State>> pq;

  // El primer nodo es 0 (Depósito)
  full_path.push_back(0);

  // Estado inicial: en route_clients[0] (Depósito) con batería full
  // Queremos llegar a route_clients[1]

  pq.push({0, b_full, 0.0, {}});

  double best_cost = numeric_limits<double>::infinity();
  vector<int> best_path;

  // Matriz de visitados para evitar ciclos infinitos en estados (idx, battery)
  vector<vector<double>> min_cost_to(
      route_clients.size(),
      vector<double>(B_levels, numeric_limits<double>::infinity()));

  while (!pq.empty()) {
    State current = pq.top();
    pq.pop();

    if (current.cost >= min_cost_to[current.index][current.battery])
      continue;
    min_cost_to[current.index][current.battery] = current.cost;

    // Si llegamos al final (último nodo, que es D)
    if (current.index == (int)route_clients.size() - 1) {
      if (current.cost < best_cost) {
        best_cost = current.cost;
        best_path = current.path_so_far;
      }
      // En Dijkstra el primero que saca es el óptimo.
      best_path = current.path_so_far;
      break;
    }

    int u_curr = route_clients[current.index];
    int u_next = route_clients[current.index + 1];

    // Intentar Min Cost
    if (isfinite(W[u_curr][u_next][current.battery])) {
      int next_bat = FinalBattery[u_curr][u_next][current.battery];
      if (next_bat >= 0) {
        vector<int> segment =
            PathUV[u_curr * M * B_levels + u_next * B_levels + current.battery];

        vector<int> new_path = current.path_so_far;
        // Añadir segmento (saltando el primero porque ya estamos ahí)
        for (size_t k = 1; k < segment.size(); k++) {
          new_path.push_back(segment[k]);
        }

        pq.push({current.index + 1, next_bat,
                 current.cost + W[u_curr][u_next][current.battery], new_path});
      }
    }

    // Intentar Max Battery
    if (isfinite(W_safe[u_curr][u_next][current.battery])) {
      int next_bat = FinalBattery_safe[u_curr][u_next][current.battery];
      if (next_bat >= 0) {
        if (abs(W_safe[u_curr][u_next][current.battery] -
                W[u_curr][u_next][current.battery]) > 1e-6 ||
            next_bat != FinalBattery[u_curr][u_next][current.battery]) {

          vector<int> segment =
              PathUV_safe[u_curr * M * B_levels + u_next * B_levels +
                          current.battery];
          vector<int> new_path = current.path_so_far;
          for (size_t k = 1; k < segment.size(); k++) {
            new_path.push_back(segment[k]);
          }

          pq.push({current.index + 1, next_bat,
                   current.cost + W_safe[u_curr][u_next][current.battery],
                   new_path});
        }
      }
    }
  }

  full_path.insert(full_path.end(), best_path.begin(), best_path.end());

  return full_path;
}
