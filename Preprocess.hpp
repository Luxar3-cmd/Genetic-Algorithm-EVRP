#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <string>
#include <utility>
#include <vector>

using namespace std;

/**
 * @struct Edge
 * @brief Arista en el grafo extendido con restricción de autonomía
 */
struct Edge {
  int to;         // Nodo destino
  double w;       // Peso: C_km * dist + (es_estación ? C_rec : 0)
  bool isStation; // True si destino es estación de recarga
};

/**
 * @enum NodeType
 * @brief Tipos de nodos en el problema EVRP
 */
enum class NodeType {
  Depot,  // Depósito: punto de inicio y fin de rutas
  Client, // Cliente: nodo obligatorio con demanda
  Station // Estación de recarga: nodo opcional para recargar batería
};

/**
 * @class Preprocess
 * @brief Preprocesa el grafo EVRP para generar matrices W, Rcnt, PathUV
 *
 * MARCO CONCEPTUAL - W 3D con Batería Residual:

 * ================================================
 
 * W[i][j][b] = Costo mínimo para ir de U[i] a U[j] cuando se entra a U[i] con
 * batería b.
 *
 * Física del problema:
 * - La batería se consume al viajar: battery_after = battery_before -
 * round(dist[u][v])
 * - La batería NO se regenera en clientes
 * - La batería se recarga al 100% (B_max) solo en estaciones o depósito
 *
 * Discretización:
 * - Batería se discretiza en pasos: b ∈ {0, battery_step, 2*battery_step, ...,
 * B_max}
 * - Consumo: round(distancia euclidiana / battery_step)
 * - Niveles de batería: B_levels = (B_max / battery_step) + 1
 *
 * Dimensiones de matrices:
 * - W[i][j][b_idx]: i,j ∈ [0..M-1], b_idx ∈ [0..B_levels-1], donde M = P+1
 * - Rcnt[i][j][b_idx]: número de recargas en el camino óptimo
 * - PathUV: almacenamiento aplanado de caminos completos
 */
class Preprocess {
public:
  /**
   * @brief Constructor: lee instancia y ejecuta preprocesamiento completo
   * @param filename Ruta al archivo de instancia del problema
   * @param battery_step Discretización de batería (default=1 para máxima
   * precisión) Valores mayores = más rápido pero menos preciso Ejemplo:
   * battery_step=10 reduce niveles de 201 a 21
   */
  Preprocess(const string &filename, int battery_step = 1);

  // ==================== Getters ====================
  const vector<vector<vector<double>>> &get_W() const { return W; }
  const vector<vector<vector<int>>> &get_Rcnt() const { return Rcnt; }
  const vector<vector<vector<int>>> &get_FinalBattery() const {
    return FinalBattery;
  }

  const vector<vector<vector<double>>> &get_W_safe() const { return W_safe; }
  const vector<vector<vector<int>>> &get_Rcnt_safe() const { return Rcnt_safe; }
  const vector<vector<vector<int>>> &get_FinalBattery_safe() const {
    return FinalBattery_safe;
  }

  const vector<vector<int>> &get_PathUV() const { return PathUV; }
  const vector<vector<int>> &get_PathUV_safe() const { return PathUV_safe; }
  const vector<int> &get_U() const { return U; }
  const vector<int> &get_demand() const { return demand; }
  const vector<vector<double>> &get_dist() const { return dist; }
  int get_M() const { return M; }
  int get_P() const { return P; }
  int get_S() const { return S; }
  int get_B() const { return B; }
  int get_Q() const { return Q; }
  int get_B_max() const { return B_max; }
  int get_C_km() const { return C_km; }
  int get_C_rec() const { return C_rec; }
  int get_battery_step() const { return battery_step; }
  int get_B_levels() const { return B_levels; }

  // ==================== Métodos de Debug ====================
  void print_W_matrix() const;
  void print_Rcnt_matrix() const;
  void print_paths() const;
  void print_node_info() const;

  /**
   * @brief Convertir node_id a índice en U
   * @param node_id ID del nodo en el grafo completo
   * @return Índice en U, o -1 si no está en U
   */
  int node_to_U_index(int node_id) const;

private:
  // ==================== Parámetros del Problema ====================
  int B;     // Cantidad de vehículos disponibles
  int Q;     // Capacidad de carga de cada vehículo
  int B_max; // Autonomía máxima con batería completa (unidades de distancia)
  int C_km;  // Costo por kilómetro recorrido
  int C_rec; // Costo fijo por cada recarga
  int P;     // Número de clientes
  int S;     // Número de estaciones de recarga
  int N;     // Número total de nodos: N = 1 + P + S
  int M;     // Tamaño de U: M = P + 1 (depósito + clientes)

  // ==================== Parámetros de Discretización ====================
  int battery_step; // Paso de discretización de batería
  int B_levels;     // Número de niveles discretos: (B_max / battery_step) + 1

  // ==================== Datos del Problema ====================
  vector<int> demand;            // demand[i]: demanda del cliente i+1
  vector<pair<int, int>> coords; // coords[i]: coordenadas (x,y) del nodo i
  vector<vector<double>> dist;   // dist[i][j]: distancia euclidiana entre i y j

  // ==================== Grafo Extendido ====================
  vector<NodeType> type;    // Tipo de cada nodo
  vector<vector<Edge>> adj; // Lista de adyacencia del grafo extendido
  vector<int> U;            // Nodos en U (depósito + clientes)
  vector<int> U_inv;        // Mapeo inverso: node_id → índice en U

  // ==================== Matrices Preprocesadas (3D) ====================
  // W[i][j][b] = costo mínimo para ir de U[i] a U[j] entrando a U[i] con
  // batería b
  vector<vector<vector<double>>> W;

  // Rcnt[i][j][b] = número de recargas en el camino óptimo
  vector<vector<vector<int>>> Rcnt;

  // FinalBattery[i][j][b] = batería residual al llegar a U[j]
  vector<vector<vector<int>>> FinalBattery;

  // Matrices "Safe" (Maximizan batería residual en lugar de minimizar costo)
  vector<vector<vector<double>>> W_safe;
  vector<vector<vector<int>>> Rcnt_safe;
  vector<vector<vector<int>>> FinalBattery_safe;

  // PathUV[i*M*B_levels + j*B_levels + b] = camino reconstruido (vector de
  // nodos)
  vector<vector<int>> PathUV;
  vector<vector<int>> PathUV_safe;

  // ==================== Métodos de Preprocesamiento ====================

  /**
   * @brief Clasificar nodos por tipo y construir conjunto U
   */
  void build_types();

  /**
   * @brief Construir grafo extendido con restricción de autonomía B_max
   */
  void build_graph();

  /**
   * @brief Preprocesar caminos óptimos entre todos los pares en U
   */
  void preprocess_paths();

  /**
   * @brief Dijkstra lexicográfico con rastreo de batería
   * @param start_id Nodo origen
   * @param b_start Batería inicial al salir de start_id
   * @param[out] distG distG[v][b] = costo desde start_id a v con batería b
   * @param[out] recG recG[v][b] = recargas desde start_id a v con batería b
   * @param[out] prev prev[v][b] = (nodo_anterior, batería_anterior)
   */
  void dijkstra_lex(int start_id, int b_start, vector<vector<double>> &distG,
                    vector<vector<int>> &recG,
                    vector<vector<pair<int, int>>> &prev);

  /**
   * @brief Reconstruir camino desde start a target usando prev
   * @param start Nodo origen
   * @param target Nodo destino
   * @param prev Vector de predecesores [nodo][batería]
   * @param best_battery Batería óptima al llegar a target
   * @return Camino completo [start, ..., target] o vacío si no existe
   */
  vector<int> reconstruct_path(int start, int target,
                               const vector<vector<pair<int, int>>> &prev,
                               int best_battery) const;

  /**
   * @brief Convertir node_id a índice en U
   * @param node_id ID del nodo en el grafo completo
   * @return Índice en U, o -1 si no está en U
   */
};

#endif // PREPROCESS_HPP
