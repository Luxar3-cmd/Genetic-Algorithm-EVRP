// ga_H.h
// Implementación de Algoritmo Genético para Electric Vehicle Routing Problem (EVRP)
// Utiliza representación TSP-like con grafo preprocesado para manejar estaciones de recarga
#ifndef ga_H
#define ga_H

#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>
#include <numeric>
#include <random>
#include <set>

using namespace std; 

/**
 * @struct Edge
 * @brief Representa una arista en el grafo extendido
 * 
 * Contiene el nodo destino, el costo de la arista (distancia + posible costo de recarga),
 * y un flag que indica si el destino es una estación de recarga.
 */
struct Edge {
    int to;             // Nodo destino de la arista
    double w;           // Peso: C_km * dist + (destino es estación ? C_rec : 0)
    bool isStation;     // True si el nodo destino es una estación de recarga
};

/**
 * @enum NodeType
 * @brief Clasificación de los nodos en el problema EVRP
 */
enum class NodeType { 
    Depot,      // Depósito: punto de inicio y fin de las rutas
    Client,     // Cliente: nodo que debe ser visitado (tiene demanda)
    Station     // Estación de recarga: nodo opcional para recargar batería al 100%
};

/**
 * @struct Route
 * @brief Representa una ruta individual en la solución EVRP
 */
struct Route {
    vector<int> clients;    // Secuencia de node_ids de clientes en esta ruta
    double cost;            // Costo total de la ruta (incluye estaciones automáticamente)
    int load;               // Carga total acumulada en la ruta
    
    Route() : cost(0.0), load(0) {}
};

/**
 * @enum FeasibilityReason
 * @brief Razones por las que una solución puede ser infactible
 */
enum class FeasibilityReason {
    Feasible,           // Solución factible
    CapacityExceeded,   // Se excede la capacidad Q de los vehículos
    FleetExceeded,      // Se excede el número máximo B de vehículos
    BatteryInsufficient, // Batería insuficiente (no hay estaciones cercanas)
    UnreachableClient,  // Algún cliente es inalcanzable
    Unknown             // Razón desconocida
};

/**
 * @struct Solution
 * @brief Representa una solución completa al EVRP
 * 
 * Una solución es el resultado de aplicar split a un cromosoma.
 * Contiene el conjunto de rutas que visitan todos los clientes.
 */
struct Solution {
    vector<Route> routes;   // Conjunto de rutas (cada ruta usa un vehículo)
    double total_cost;      // Costo total de la solución
    int num_vehicles;       // Número de vehículos utilizados
    bool is_feasible;       // true si respeta Q (capacidad), B (flota) y batería
    FeasibilityReason infeasibility_reason; // Razón de infactibilidad si is_feasible = false
    string infeasibility_details; // Detalles adicionales sobre la infactibilidad
    
    Solution() : total_cost(0.0), num_vehicles(0), is_feasible(false), 
                 infeasibility_reason(FeasibilityReason::Unknown), infeasibility_details("") {}
};

// --- Función externa ---
/**
 * @brief Calcula la matriz de distancias euclidianas entre todos los nodos
 * @param coordenadas Vector de coordenadas (x,y) de todos los nodos
 * @param P Número de clientes
 * @param S Número de estaciones de recarga
 * @return Matriz NxN con distancias euclidianas, donde N = 1 + P + S
 */
vector<vector<double>> matrix_distances(const vector<pair<int,int>> &coordenadas, int P, int S); 

/**
 * @class evolutionaryAlgo
 * @brief Clase principal para resolver EVRP mediante algoritmo genético con representación TSP-like
 * 
 * Esta clase preprocesa el grafo completo (Depósito + Clientes + Estaciones) para generar
 * un grafo contraído que solo incluye nodos obligatorios (Depósito + Clientes), donde los
 * costos entre ellos ya contemplan las estaciones de recarga necesarias y la restricción
 * de autonomía (B_max). Esto permite usar la función split estándar de VRP sin modificaciones.
 */
class evolutionaryAlgo {
    public: 
        // Configuración del algoritmo genético
        int population_size;      // Tamaño de la población 
        int generations;          // Número de generaciones a ejecutar
        double crossover_rate;    // Tasa de cruce entre individuos
        double mutation_rate;     // Tasa de mutación de genes
        int local_search_iterations; // K_max: máximo de iteraciones de búsqueda local en mutación
        vector<vector<int>> population; // Población de soluciones

        vector<double> fitness; // Fitness[i] = aptitud del individuo i
        int best_fitness_index; // Índice del mejor individuo en la población
        Solution best_solution;   // Mejor solución encontrada
        
        // Población de padres seleccionados para cruce
        vector<vector<int>> parents;  // Padres seleccionados mediante torneos
        
        /**
         * @brief Ejecuta el algoritmo genético completo
         * 
         * Loop principal del algoritmo genético que:
         * 1. Inicializa la población
         * 2. Evalúa la población inicial
         * 3. Por cada generación:
         *    - Selecciona padres
         *    - Aplica cruce
         *    - Aplica mutación
         *    - Evalúa la nueva población
         * 4. Al finalizar, imprime la mejor solución en el formato especificado
         */
        void run();
        
        /**
         * @brief Constructor: lee la instancia y preprocesa el grafo
         * @param filename Ruta al archivo de instancia del problema
         */
        evolutionaryAlgo(const string& filename);
        
        /**
         * @brief Inicializa los parámetros del algoritmo genético
         * @param pop_size Tamaño de la población
         * @param gens Número de generaciones
         * @param cross_rate Tasa de cruce
         * @param mut_rate Tasa de mutación
         * @param ls_iterations Número máximo de iteraciones de búsqueda local en mutación (K_max)
         */
        void initialize_parameters(int pop_size, int gens, double cross_rate, double mut_rate, int ls_iterations = 10);
        
        /**
         * @brief Imprime la matriz de distancias euclidianas entre todos los nodos
         */
        void print_distance_matrix() const;
        
        /**
         * @brief Imprime la matriz W del grafo preprocesado (costos entre nodos en U)
         */
        void print_preprocessed_matrix() const;
        
        /**
         * @brief Imprime la matriz Rcnt (número de recargas entre nodos en U)
         */
        void print_recharge_count_matrix() const;
        
        /**
         * @brief Imprime información sobre los nodos (tipos, coordenadas, etc.)
         */
        void print_node_info() const;
        
        /**
         * @brief Imprime los caminos completos entre todos los pares de nodos en U
         */
        void print_paths() const;
        
    private:
        /**
         * @brief Imprime la solución en el formato de salida especificado
         * 
         * Formato:
         * - Costo total
         * - Para cada vehículo: ruta completa (con estaciones), distancia, recargas
         * - Leyenda explicativa
         * 
         * @param solution Solución a imprimir
         */
        void print_solution(const Solution& solution) const;
        
        /**
         * @brief Expande una ruta usando PathUV para mostrar el camino completo con estaciones
         * 
         * @param route Ruta a expandir
         * @return Vector con la secuencia completa de nodos: [D, ..., C, ..., R, ..., D]
         */
        vector<int> expand_route(const Route& route) const;
        
        /**
         * @brief Calcula la distancia total real de un camino expandido
         * 
         * @param path Camino completo de nodos
         * @return Distancia total euclidiana
         */
        double calculate_route_distance(const vector<int>& path) const;
        
        /**
         * @brief Cuenta el número de recargas en un camino expandido
         * 
         * @param path Camino completo de nodos
         * @return Número de estaciones de recarga visitadas
         */
        int count_recharges(const vector<int>& path) const;
        
        /**
         * @brief Convierte un node_id a su índice en el conjunto U
         * 
         * @param node_id ID del nodo
         * @return Índice en U, o -1 si no está en U
         */
        int node_to_U_index(int node_id) const;


        // ========== Parámetros del Problema ==========
        int B;          // Cantidad de vehículos disponibles
        int Q;          // Capacidad de carga de cada vehículo (unidades de demanda)
        int B_max;      // Autonomía máxima con batería completa (unidades de distancia)
        int C_km;       // Costo monetario por kilómetro recorrido
        int C_rec;      // Costo fijo por cada recarga en una estación
        
        int P;          // Total de clientes a visitar
        int S;          // Total de estaciones de recarga disponibles
        int N;          // Total de nodos: N = 1 (depósito) + P (clientes) + S (estaciones)

        // ========== Datos del Problema ==========
        vector<int> demand;             // demand[i]: demanda del cliente con node_id = i+1
                                        // demand[0] = demanda del cliente 1 (nodo 1)
                                        // demand[1] = demanda del cliente 2 (nodo 2), etc.
                                        // Para acceder: demand[node_id - 1] donde node_id ∈ [1,P]
        vector<pair<int,int>> coords;   // coords[i]: coordenadas (x,y) del nodo i

        // ========== Matrices Base ==========
        vector<vector<double>> dist;    // dist[i][j]: distancia euclidiana entre nodos i y j

        // ========== Grafo Extendido ==========
        // Grafo que considera la restricción de autonomía B_max
        vector<NodeType> type;          // type[i]: tipo del nodo i (Depot, Client o Station)
        vector<vector<Edge>> adj;       // adj[u]: lista de aristas salientes desde el nodo u
                                        // Solo existe arista u→v si dist[u][v] ≤ B_max

        // ========== Conjunto de Nodos Obligatorios ==========
        vector<int> U;                  // U = {Depósito} ∪ {Clientes}
                                        // U[0] = 0 (depósito), U[1..P] = clientes
                                        // Tamaño: M = P + 1
        vector<int> U_inv;              // U_inv[node_id] = índice de node_id en U
                                        // U_inv[0] = 0, U_inv[1] = 1, ..., U_inv[P] = P
                                        // U_inv[j] = -1 para estaciones (j > P)
        // ========== Salidas del Preprocesamiento ==========
        // Estas matrices permiten que split funcione sin conocer las estaciones
        vector<vector<double>> W;       // W[i][j]: costo mínimo de U[i] → U[j] 
                                        // Incluye automáticamente estaciones intermedias
                                        // Dimensión: M × M donde M = P + 1
        
        vector<vector<int>> Rcnt;       // Rcnt[i][j]: número de recargas en el camino mínimo U[i] → U[j]
                                        // Dimensión: M × M
        
        vector<vector<int>> PathUV;     // PathUV[i*M + j]: camino completo de U[i] → U[j]
                                        // Incluye nodos origen, intermedios (solo estaciones) y destino
                                        // Formato: [U[i], estación1, estación2, ..., U[j]]
                                        // Vector 1D aplanado: tamaño M*M, acceso con índice i*M + j
        
    private:
        /**
         * @brief Obtiene la demanda de un cliente dado su node_id
         * @param node_id ID del nodo cliente (debe estar en rango [1, P])
         * @return Demanda del cliente
         * @throws runtime_error si node_id no corresponde a un cliente
         */
        int get_client_demand(int node_id) const;
        
        /**
         * @brief Inicializa la población de soluciones aleatoriamente
         */
        void initialize_population();
        
        /**
         * @brief Aplica el algoritmo split a un cromosoma para obtener la mejor solución VRP
         * 
         * Split construye un grafo auxiliar H donde cada arco (i,j) representa una ruta
         * que atiende los clientes chromosome[i..j-1]. Usa programación dinámica para
         * encontrar la partición óptima en rutas.
         * 
         * La función está dividida en dos partes:
         * 1. Programación dinámica: calcula V[] (costos mínimos) y Pred[] (predecesores)
         * 2. Reconstrucción: llama a reconstruct_solution_from_split() para decodificar
         *    la solución desde Pred[]
         * 
         * @param chromosome Permutación de node_ids de clientes [1..P]
         * @return Solución con rutas óptimas y su costo
         */
        Solution split(const vector<int>& chromosome) const;
        
        // ========== Funciones del Algoritmo Genético ==========
        
        /**
         * @brief Realiza la selección de padres usando Binary Tournament Selection
         * 
         * Selecciona padres para el cruce mediante torneos binarios. En cada torneo,
         * se eligen dos individuos aleatorios y se selecciona el mejor (menor fitness).
         * Esto ayuda a evitar clones ya que siempre se seleccionan individuos diferentes.
         */
        void selection();
        
        /**
         * @brief Realiza el cruce de padres usando Order Crossover (OX)
         * 
         * Order Crossover preserva el orden relativo de los elementos de un padre
         * mientras mantiene las posiciones absolutas del otro padre.
         */
        void crossover();
        
        /**
         * @brief Aplica mutación tipo búsqueda local a los individuos de la población
         * 
         * La mutación aplica búsqueda local usando tres tipos de movimientos:
         * - 2-opt: Invierte un segmento de la permutación
         * - Relocate: Mueve un cliente de posición i a j
         * - Swap: Intercambia dos clientes
         * 
         * Se aplica a cada individuo según mutation_rate. Para cada individuo seleccionado:
         * 1. Realiza hasta K_max iteraciones de búsqueda local
         * 2. En cada iteración, selecciona un movimiento aleatorio
         * 3. Aplica el movimiento y evalúa el fitness
         * 4. Si mejora, acepta el movimiento (first improvement)
         * 5. Si no mejora en K_max iteraciones, mantiene el individuo original
         */
        void mutation();
        
    private:
        /**
         * @brief Aplica búsqueda local a un individuo específico
         * 
         * @param individual Índice del individuo en la población
         * @return true si se encontró una mejora, false en caso contrario
         */
        bool local_search_mutation(size_t individual);
        
        /**
         * @brief Aplica movimiento 2-opt a una permutación
         * 
         * Invierte el segmento entre las posiciones i y j.
         * 
         * @param chromosome Permutación a modificar (se modifica in-place)
         * @param i Posición inicial del segmento
         * @param j Posición final del segmento
         */
        void apply_2opt(vector<int>& chromosome, int i, int j) const;
        
        /**
         * @brief Aplica movimiento Relocate a una permutación
         * 
         * Mueve el elemento en posición i a la posición j.
         * 
         * @param chromosome Permutación a modificar (se modifica in-place)
         * @param i Posición origen
         * @param j Posición destino
         */
        void apply_relocate(vector<int>& chromosome, int i, int j) const;
        
        /**
         * @brief Aplica movimiento Swap a una permutación
         * 
         * Intercambia los elementos en las posiciones i y j.
         * 
         * @param chromosome Permutación a modificar (se modifica in-place)
         * @param i Primera posición
         * @param j Segunda posición
         */
        void apply_swap(vector<int>& chromosome, int i, int j) const;
        
    private: 
        /**
         * @brief Clasifica cada nodo según su tipo y construye el conjunto U
         * 
         * Asigna:
         * - type[0] = Depot
         * - type[1..P] = Client
         * - type[P+1..N-1] = Station
         * 
         * Construye U = {0, 1, 2, ..., P} (depósito + clientes)
         */
        void build_types();
        
        /**
         * @brief Construye el grafo extendido considerando la restricción de autonomía
         * 
         * Crea arista a→b solo si dist[a][b] ≤ B_max
         * Peso de arista: w = C_km * dist[a][b] + (b es estación ? C_rec : 0)
         * 
         * Asume: al llegar a una estación, se recarga la batería al 100%
         */
        void build_graph();
        
        /**
         * @brief Preprocesa los caminos óptimos entre todos los pares de nodos en U
         * 
         * Para cada par (i,j) en U × U:
         * - Ejecuta dijkstra_lex desde U[i]
         * - Calcula W[i][j], Rcnt[i][j], PathUV[i][j]
         * - Verifica que todos los clientes sean alcanzables desde el depósito
         */
        void preprocess_paths();

        /**
         * @brief Dijkstra con prioridad lexicográfica: (distancia, recargas)
         * 
         * Encuentra caminos mínimos desde start_id a todos los demás nodos considerando:
         * 1. Minimizar distancia/costo total
         * 2. En empate, minimizar número de recargas
         * 
         * @param start_id Nodo origen
         * @param distG[out] distG[v] = costo mínimo desde start_id a v
         * @param recG[out] recG[v] = número de recargas en ese camino
         * @param prev[out] prev[v] = predecesor de v en el árbol de caminos mínimos
         */
        void dijkstra_lex(int start_id, vector<double>& distG, vector<int>& recG, vector<int>& prev);
        
        /**
         * @brief Reconstruye el camino desde start_id hasta target_id usando el vector prev
         * 
         * @param start_id Nodo origen
         * @param target_id Nodo destino
         * @param prev Vector de predecesores (salida de dijkstra_lex)
         * @return Vector con el camino [start_id, ..., target_id], vacío si no hay camino
         */
        vector<int> reconstruct_path(int start_id, int target_id, const vector<int>& prev) const;
        
        /**
         * @brief Reconstruye la solución desde el vector de predecesores del algoritmo split
         * 
         * Toma el resultado de la programación dinámica (vector Pred) y reconstruye
         * las rutas correspondientes, calculando cargas y costos de cada ruta.
         * 
         * @param chromosome Permutación de clientes del cromosoma
         * @param V Vector de costos mínimos de la programación dinámica
         * @param Pred Vector de predecesores de la programación dinámica
         * @param n Número de clientes en el cromosoma
         * @return Solución reconstruida con todas las rutas
         */
        Solution reconstruct_solution_from_split(const vector<int>& chromosome, 
                                                  const vector<double>& V,
                                                  const vector<int>& Pred, 
                                                  int n) const;
        
        /**
         * @brief Evalúa la aptitud de cada individuo en la población, utilizando la función split
         * 
         * Función interna del backend que calcula el fitness de todos los individuos.
         * No debe ser llamada directamente desde fuera de la clase.
         */
        void evaluate_population();
        

        
};

#endif /* ga_H */

