/**
 * @file Preprocess.cpp
 * @brief Implementación del preprocesamiento EVRP con W 3D (batería residual)
 *
 * Este archivo implementa el preprocesamiento del grafo EVRP para generar
 * matrices W[i][j][b], Rcnt[i][j][b] y PathUV que consideran el estado de
 * batería residual.
 */

#include "Preprocess.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>

using namespace std;

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

/**
 * @brief Calcula la matriz de distancias euclidianas entre todos los nodos
 */
static vector<vector<double>>
calculate_distances(const vector<pair<int, int>> &coords) {
  int N = coords.size();
  vector<vector<double>> matriz(N, vector<double>(N, 0.0));

  for (int i = 0; i < N; i++) {
    for (int j = i; j < N; j++) {
      double dx = coords[j].first - coords[i].first;
      double dy = coords[j].second - coords[i].second;
      double d = sqrt(dx * dx + dy * dy);

      matriz[i][j] = d;
      matriz[j][i] = d;
    }
  }
  return matriz;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Preprocess::Preprocess(const string &filename, int battery_step)
    : battery_step(battery_step) {
  // Leer archivo de instancia
  ifstream file(filename);
  if (!file)
    throw runtime_error("No se pudo abrir " + filename);

  // LÍNEA 1: B Q Bmax
  file >> B >> Q >> B_max;

  // LÍNEA 2: Ckm Crec
  file >> C_km >> C_rec;

  // LÍNEA 3: P (número de clientes)
  file >> P;

  // LÍNEA 4: S (número de estaciones)
  file >> S;

  // Validar parámetros
  if (B <= 0)
    throw runtime_error("B (vehículos) debe ser > 0");
  if (Q <= 0)
    throw runtime_error("Q (capacidad) debe ser > 0");
  if (B_max <= 0)
    throw runtime_error("B_max (autonomía) debe ser > 0");
  if (C_km < 0)
    throw runtime_error("C_km debe ser >= 0");
  if (C_rec < 0)
    throw runtime_error("C_rec debe ser >= 0");
  if (P <= 0)
    throw runtime_error("P (clientes) debe ser > 0");
  if (S < 0)
    throw runtime_error("S (estaciones) debe ser >= 0");

  // Calcular total de nodos: 1 depósito + P clientes + S estaciones
  N = 1 + P + S;

  // LÍNEA 5: Leer demandas de los P clientes
  demand.resize(P);
  for (int i = 0; i < P; i++) {
    file >> demand[i];
    if (demand[i] < 0)
      throw runtime_error("Demanda debe ser >= 0");
  }

  // LÍNEA 6: N (total de nodos) - validación
  int N_file;
  file >> N_file;
  if (N_file != N) {
    throw runtime_error("N en archivo (" + to_string(N_file) +
                        ") no coincide con 1+P+S = " + to_string(N));
  }

  // LÍNEAS 7 a 7+N-1: Leer coordenadas de todos los nodos
  // Orden: depósito (nodo 0), luego P clientes (nodos 1..P), luego S estaciones
  // (nodos P+1..N-1)
  coords.resize(N);
  for (int i = 0; i < N; i++) {
    file >> coords[i].first >> coords[i].second;
  }

  // Calcular tamaño de U y niveles de batería
  M = P + 1;                             // Depósito + Clientes
  B_levels = (B_max / battery_step) + 1; // Niveles discretos de batería

  // FASE 0: Calcular distancias euclidianas
  dist = calculate_distances(coords);

  // FASES DE PREPROCESAMIENTO
  build_types();      // FASE 1: Clasificar nodos y construir U
  build_graph();      // FASE 2: Construir grafo extendido
  preprocess_paths(); // FASE 3: Calcular W, Rcnt, PathUV
}

// ============================================================================
// FASE 1: CLASIFICAR NODOS Y CONSTRUIR U
// ============================================================================

void Preprocess::build_types() {
  // Inicializar todos como clientes
  type.assign(N, NodeType::Client);

  // Asignar tipos específicos
  type[0] = NodeType::Depot; // Nodo 0 es el depósito

  for (int j = P + 1; j < P + 1 + S; j++) {
    type[j] = NodeType::Station; // Nodos P+1..N-1 son estaciones
  }

  // Construir conjunto U = {Depósito} ∪ {Clientes}
  U.clear();
  U.push_back(0); // Depósito en U[0]
  for (int i = 1; i <= P; i++) {
    U.push_back(i); // Clientes en U[1..P]
  }
  // |U| = P + 1 = M

  // Construir U_inv para mapeo inverso
  U_inv.assign(N, -1);
  for (size_t i = 0; i < U.size(); i++) {
    U_inv[U[i]] = i;
  }
}

// ============================================================================
// FASE 2: CONSTRUIR GRAFO EXTENDIDO
// ============================================================================

void Preprocess::build_graph() {
  // Inicializar lista de adyacencia vacía
  adj.assign(N, {});

  // Construir aristas considerando restricción de autonomía B_max
  for (int a = 0; a < N; a++) {
    for (int b = 0; b < N; b++) {
      if (a == b)
        continue; // No hay auto-loops

      double d = dist[a][b];

      // Solo crear arista si está dentro de la autonomía
      if (d <= B_max) {
        // Peso: costo por kilómetro + posible costo de recarga
        double w = C_km * d + (type[b] == NodeType::Station ? C_rec : 0.0);

        adj[a].push_back(Edge{b, w, type[b] == NodeType::Station});
      }
    }
  }
}

// ============================================================================
// FASE 3: PREPROCESAR CAMINOS ENTRE NODOS EN U
// ============================================================================

/**
 * @struct State
 * @brief Estado para Dijkstra lexicográfico con batería
 */
struct State {
  double dist; // Costo acumulado
  int rec;     // Recargas realizadas
  int v;       // Nodo actual
  int battery; // Batería actual (discretizada)

  // Operador invertido para min-heap
  bool operator<(const State &o) const {
    if (dist != o.dist)
      return dist > o.dist;
    return rec > o.rec;
  }
};

void Preprocess::dijkstra_lex(int start_id, int b_start,
                              vector<vector<double>> &distG,
                              vector<vector<int>> &recG,
                              vector<vector<pair<int, int>>> &prev) {
  const double INF = numeric_limits<double>::infinity();

  // Redimensionar y resetear matrices
  distG.assign(N, vector<double>(B_levels, INF));
  recG.assign(N, vector<int>(B_levels, __INT_MAX__));
  prev.assign(N, vector<pair<int, int>>(B_levels, {-1, -1}));

  // Inicializar nodo origen con batería inicial
  priority_queue<State> pq;
  distG[start_id][b_start] = 0.0;
  recG[start_id][b_start] = 0;
  pq.push(State{0.0, 0, start_id, b_start});

  // Algoritmo de Dijkstra
  while (!pq.empty()) {
    State state = pq.top();
    pq.pop();

    int u = state.v;
    int b = state.battery;

    // Estado desactualizado
    if (state.dist > distG[u][b] ||
        (state.dist == distG[u][b] && state.rec > recG[u][b])) {
      continue;
    }

    // Explorar vecinos
    for (const auto &e : adj[u]) {
      int v = e.to;

      // Calcular consumo de batería (discretización con battery_step)
      double distance_uv = dist[u][v];
      int db = (int)round(distance_uv / battery_step);

      // Verificar si hay suficiente batería
      if (db > b) {
        continue; // No se puede alcanzar v
      }

      // Calcular batería después del movimiento
      int b_after = b - db;

      // Calcular nuevo costo y recargas
      double new_dist = state.dist + e.w;
      int new_rec = state.rec;
      int b_next;

      // Lógica de recarga según tipo de nodo
      if (type[v] == NodeType::Station) {
        // ESTACIÓN: recarga batería, cobra C_rec, incrementa contador
        b_next = B_levels - 1; // Batería llena (índice máximo)
        new_dist += C_rec;
        new_rec += 1;
      } else if (v == 0) {
        // DEPÓSITO: recarga batería SIN cobrar ni incrementar contador
        // (el depósito marca fin de ruta, la recarga es conceptualmente
        // opcional)
        b_next = B_levels - 1; // Batería llena
        // NO se cobra: new_dist += 0
        // NO se cuenta: new_rec += 0
      } else {
        // CLIENTE: no recarga, batería residual
        b_next = b_after;
      }

      // Verificar si este camino es mejor
      bool better = false;
      if (new_dist < distG[v][b_next]) {
        better = true;
      } else if (new_dist == distG[v][b_next] && new_rec < recG[v][b_next]) {
        better = true;
      }

      // Actualizar si encontramos un camino mejor
      if (better) {
        distG[v][b_next] = new_dist;
        recG[v][b_next] = new_rec;
        prev[v][b_next] = {u, b};
        pq.push(State{new_dist, new_rec, v, b_next});
      }
    }
  }
}

vector<int>
Preprocess::reconstruct_path(int start, int target,
                             const vector<vector<pair<int, int>>> &prev,
                             int best_battery) const {
  // Si origen y destino son el mismo
  if (start == target)
    return {start};

  // Reconstruir camino en reversa
  vector<int> rev;
  int v = target;
  int b = best_battery;

  while (!(v == start && b >= 0)) {
    rev.push_back(v);
    auto p = prev[v][b];
    if (p.first == -1) {
      // No hay camino válido
      return {};
    }
    v = p.first;
    b = p.second;
  }

  rev.push_back(start);
  reverse(rev.begin(), rev.end());
  return rev;
}

void Preprocess::preprocess_paths() {
  // Inicializar matrices 3D
  W.assign(
      M, vector<vector<double>>(
             M, vector<double>(B_levels, numeric_limits<double>::infinity())));
  Rcnt.assign(M, vector<vector<int>>(M, vector<int>(B_levels, __INT_MAX__)));
  FinalBattery.assign(M, vector<vector<int>>(M, vector<int>(B_levels, -1)));

  W_safe.assign(
      M, vector<vector<double>>(
             M, vector<double>(B_levels, numeric_limits<double>::infinity())));
  Rcnt_safe.assign(M,
                   vector<vector<int>>(M, vector<int>(B_levels, __INT_MAX__)));
  FinalBattery_safe.assign(M,
                           vector<vector<int>>(M, vector<int>(B_levels, -1)));

  PathUV.assign(M * M * B_levels, {});
  PathUV_safe.assign(M * M * B_levels, {});

  // Vectores temporales para dijkstra
  vector<vector<double>> distG;
  vector<vector<int>> recG;
  vector<vector<pair<int, int>>> prev;
  // Matriz temporal para guardar batería final de cada nodo en Dijkstra
  // No necesitamos guardarla globalmente, la podemos deducir o extraer de prev,
  // pero Dijkstra ya calcula la batería al llegar a cada nodo.
  // Espera, Dijkstra calcula costo minimo.
  // Necesitamos saber con qué batería llegamos al destino 'target' en el camino
  // óptimo. El estado en Dijkstra es (dist, rec, v, battery). Pero nosotros
  // corremos Dijkstra desde 'start' con 'b_start'. Al final miramos
  // distG[target][b]. El 'b' que minimiza el costo es la batería con la que
  // llegamos.

  // Para cada nodo origen en U y cada nivel de batería inicial
  for (int iu = 0; iu < M; iu++) {
    int start = U[iu];

    // Para cada nivel de batería inicial posible
    for (int b_start = 0; b_start < B_levels; b_start++) {
      // Ejecutar Dijkstra desde start con batería b_start
      dijkstra_lex(start, b_start, distG, recG, prev);

      // Para cada nodo destino en U
      for (int jv = 0; jv < M; jv++) {
        int target = U[jv];
        if (start == target) {
          W[iu][jv][b_start] = 0.0;
          Rcnt[iu][jv][b_start] = 0;
          FinalBattery[iu][jv][b_start] = b_start;

          W_safe[iu][jv][b_start] = 0.0;
          Rcnt_safe[iu][jv][b_start] = 0;
          FinalBattery_safe[iu][jv][b_start] = b_start;
          continue;
        }

        // Buscar el mejor estado de llegada a target
        // Estrategia 1: Minimizar Costo (Original)
        double best_dist = numeric_limits<double>::infinity();
        int best_rec = __INT_MAX__;
        int best_battery = -1;

        // Estrategia 2: Maximizar Batería Final (Safe)
        double safe_dist = numeric_limits<double>::infinity();
        int safe_rec = __INT_MAX__;
        int safe_battery = -1;

        for (int b = 0; b < B_levels; b++) {
          double d = distG[target][b];
          int r = recG[target][b];

          if (!isfinite(d))
            continue;

          // Actualizar Min Cost
          if (d < best_dist) {
            best_dist = d;
            best_rec = r;
            best_battery = b;
          } else if (d == best_dist && r < best_rec) {
            best_rec = r;
            best_battery = b;
          } else if (d == best_dist && r == best_rec && b > best_battery) {
            best_battery = b; // Tie-break: más batería
          }

          // Actualizar Max Battery (Safe)
          // Priorizamos batería, pero solo si el costo no es infinito
          if (b > safe_battery) {
            safe_battery = b;
            safe_dist = d;
            safe_rec = r;
          } else if (b == safe_battery && d < safe_dist) {
            safe_dist = d;
            safe_rec = r;
          }
        }

        // Guardar resultados Min Cost
        if (!isfinite(best_dist) || best_battery == -1) {
          // No alcanzable
          W[iu][jv][b_start] = numeric_limits<double>::infinity();
          Rcnt[iu][jv][b_start] = __INT_MAX__;
          FinalBattery[iu][jv][b_start] = -1;
          PathUV[iu * M * B_levels + jv * B_levels + b_start].clear();
        } else {
          W[iu][jv][b_start] = best_dist;
          Rcnt[iu][jv][b_start] = best_rec;
          FinalBattery[iu][jv][b_start] = best_battery;
          PathUV[iu * M * B_levels + jv * B_levels + b_start] =
              reconstruct_path(start, target, prev, best_battery);
        }

        // Guardar resultados Max Battery
        if (!isfinite(safe_dist) || safe_battery == -1) {
          W_safe[iu][jv][b_start] = numeric_limits<double>::infinity();
          Rcnt_safe[iu][jv][b_start] = __INT_MAX__;
          FinalBattery_safe[iu][jv][b_start] = -1;
          PathUV_safe[iu * M * B_levels + jv * B_levels + b_start].clear();
        } else {
          W_safe[iu][jv][b_start] = safe_dist;
          Rcnt_safe[iu][jv][b_start] = safe_rec;
          FinalBattery_safe[iu][jv][b_start] = safe_battery;
          PathUV_safe[iu * M * B_levels + jv * B_levels + b_start] =
              reconstruct_path(start, target, prev, safe_battery);
        }
      }
    }
  }
}

// ============================================================================
// HELPER
// ============================================================================

int Preprocess::node_to_U_index(int node_id) const {
  if (node_id < 0 || node_id >= N)
    return -1;
  return U_inv[node_id];
}

// ============================================================================
// MÉTODOS DE DEBUG
// ============================================================================

void Preprocess::print_node_info() const {
  cout << "\n=== INFORMACIÓN DE NODOS ===" << endl;
  cout << "Total de nodos: " << N << endl;
  cout << "Clientes: " << P << endl;
  cout << "Estaciones: " << S << endl;
  cout << "M (|U|): " << M << endl;
  cout << "B_max: " << B_max << endl;
  cout << "Discretización de batería: " << battery_step << endl;
  cout << "Niveles de batería: " << B_levels << endl;
  cout << "\nTipos de nodos:" << endl;
  for (int i = 0; i < N; i++) {
    cout << "  Nodo " << i << ": ";
    if (type[i] == NodeType::Depot)
      cout << "Depósito";
    else if (type[i] == NodeType::Client)
      cout << "Cliente";
    else
      cout << "Estación";
    cout << " (coords: " << coords[i].first << "," << coords[i].second << ")";
    if (type[i] == NodeType::Client && i > 0) {
      cout << " [demanda: " << demand[i - 1] << "]";
    }
    cout << endl;
  }
}

void Preprocess::print_W_matrix() const {
  cout << "\n=== MATRIZ W (muestra para batería llena) ===" << endl;
  cout
      << "W[i][j][B_levels-1] = costo mínimo de U[i] a U[j] con batería llena\n"
      << endl;

  int b_full = B_levels - 1; // Índice de batería llena

  cout << setw(8) << "";
  for (int j = 0; j < M; j++) {
    cout << setw(10) << ("U[" + to_string(j) + "]=" + to_string(U[j]));
  }
  cout << endl;

  for (int i = 0; i < M; i++) {
    cout << setw(8) << ("U[" + to_string(i) + "]=" + to_string(U[i]));
    for (int j = 0; j < M; j++) {
      double val = W[i][j][b_full];
      if (isfinite(val)) {
        cout << setw(10) << fixed << setprecision(2) << val;
      } else {
        cout << setw(10) << "INF";
      }
    }
    cout << endl;
  }
}

void Preprocess::print_Rcnt_matrix() const {
  cout << "\n=== MATRIZ Rcnt (muestra para batería llena) ===" << endl;
  cout << "Rcnt[i][j][B_levels-1] = número de recargas de U[i] a U[j]\n"
       << endl;

  int b_full = B_levels - 1;

  cout << setw(8) << "";
  for (int j = 0; j < M; j++) {
    cout << setw(8) << ("U[" + to_string(j) + "]");
  }
  cout << endl;

  for (int i = 0; i < M; i++) {
    cout << setw(8) << ("U[" + to_string(i) + "]");
    for (int j = 0; j < M; j++) {
      int val = Rcnt[i][j][b_full];
      if (val != __INT_MAX__) {
        cout << setw(8) << val;
      } else {
        cout << setw(8) << "INF";
      }
    }
    cout << endl;
  }
}

void Preprocess::print_paths() const {
  cout << "\n=== CAMINOS PREPROCESADOS (muestra para batería llena) ==="
       << endl;

  int b_full = B_levels - 1;

  for (int i = 0; i < min(3, M); i++) {
    for (int j = 0; j < min(3, M); j++) {
      if (i == j)
        continue;

      const auto &path = PathUV[i * M * B_levels + j * B_levels + b_full];
      cout << "U[" << i << "]=" << U[i] << " → U[" << j << "]=" << U[j] << ": ";

      if (path.empty()) {
        cout << "NO ALCANZABLE";
      } else {
        for (size_t k = 0; k < path.size(); k++) {
          if (k > 0)
            cout << " → ";
          cout << path[k];
          if (type[path[k]] == NodeType::Station)
            cout << "(R)";
        }
      }
      cout << endl;
    }
  }
}
