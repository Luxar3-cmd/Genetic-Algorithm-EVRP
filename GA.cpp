#include "GA.h"
#include <iomanip>

using namespace std;

/**
 * FASE 3: PREPROCESAMIENTO DE CAMINOS ÓPTIMOS
 * ==============================================
 * Esta función colapsa el grafo completo (N nodos) en un grafo contraído (M nodos)
 * donde M = P+1 = |U| = tamaño del conjunto de nodos obligatorios.
 * 
 * Objetivo: Calcular el costo óptimo entre cada par de nodos en U (depósito + clientes),
 * considerando automáticamente las estaciones de recarga necesarias.
 * 
 * Resultado:
 * - W[i][j]: Costo mínimo para ir de U[i] a U[j] (puede incluir estaciones intermedias)
 * - Rcnt[i][j]: Número de recargas realizadas en ese camino
 * - PathUV[i*M+j]: Camino completo [U[i], ..., estaciones, ..., U[j]]
 */
void evolutionaryAlgo::preprocess_paths() {
    int M = (int)U.size(); // M = P+1 (depósito + clientes)
    
    // Inicializar matrices de salida con valores infinitos
    W.assign(M, vector<double>(M, numeric_limits<double>::infinity()));
    Rcnt.assign(M, vector<int>(M, __INT_MAX__));
    PathUV.assign(M*M, {}); // Almacenamiento aplanado: índice = iu*M + jv
    
    // Vectores temporales para dijkstra
    vector<double> distG;
    vector<int> recG, prev; 
    
    // Para cada nodo origen en U
    for (int iu = 0; iu < M; ++iu) {
        int start = U[iu]; // ID real del nodo en el grafo completo
        
        // Ejecutar Dijkstra desde este nodo hacia todos los demás
        dijkstra_lex(start, distG, recG, prev); 
        
        // Extraer información solo para nodos en U
        for (int jv = 0; jv < M; ++jv) {
            if (iu == jv) continue; // Saltar diagonal (mismo nodo)
            
            int target = U[jv]; // ID real del nodo destino
            double d = distG[target];
            
            // Si no es alcanzable, marcar como infinito
            if (!isfinite(d)) {
                W[iu][jv] = numeric_limits<double>::infinity();
                Rcnt[iu][jv] = __INT_MAX__;
                PathUV[iu*M + jv].clear();
                continue;
            }
            
            // Guardar costo, recargas y camino completo
            W[iu][jv] = d; 
            Rcnt[iu][jv] = recG[target];
            PathUV[iu*M + jv] = reconstruct_path(start, target, prev);
        }
    }
    
    // VALIDACIÓN: Verificar que todos los clientes sean alcanzables desde el depósito
    // y que se pueda regresar del cliente al depósito
    for (int i = 1; i < M; i++) {
        if (!isfinite(W[0][i]) || !isfinite(W[i][0])) {
            throw runtime_error("Cliente inalcanzable con B_max y estaciones dadas");
        }
    }
}

/**
 * RECONSTRUCCIÓN DE CAMINO
 * =========================
 * Reconstruye el camino completo desde start_id hasta target_id usando el
 * vector de predecesores generado por dijkstra_lex.
 * 
 * @param start_id Nodo origen del camino
 * @param target_id Nodo destino del camino
 * @param prev Vector de predecesores donde prev[v] = nodo anterior a v en el camino mínimo
 * @return Vector con todos los nodos del camino: [start_id, n1, n2, ..., target_id]
 *         Incluye estaciones de recarga intermedias si las hay.
 *         Retorna vector vacío si no existe camino.
 */
vector<int> evolutionaryAlgo::reconstruct_path(int start_id, int target_id, const vector<int>& prev) const {
    // Caso base: si origen y destino son el mismo
    if (start_id == target_id) return {start_id}; 
    
    // Si el target no tiene predecesor, no hay camino
    if (prev[target_id] == -1) return {};
    
    // Reconstruir camino en reversa desde target hasta start
    vector<int> rev; 
    rev.reserve(N); 
    int cur = target_id; 
    
    while (cur != -1) {
        rev.push_back(cur);
        if (cur == start_id) break; 
        cur = prev[cur]; 
    }
    
    // Verificar que efectivamente llegamos al start
    if (rev.back() != start_id) return {}; // No se alcanzó el inicio
    
    // Invertir para tener orden correcto: start → target
    reverse(rev.begin(), rev.end());
    return rev; // [start_id, ..., estaciones, ..., target_id]
}

/**
 * @struct State
 * @brief Estado para el algoritmo de Dijkstra lexicográfico
 * 
 * Representa un estado en la búsqueda de camino mínimo con dos criterios:
 * 1. Distancia/costo acumulado (prioridad principal)
 * 2. Número de recargas (criterio de desempate)
 * 
 * El operador< está invertido para convertir el max-heap de priority_queue
 * en un min-heap (necesario para Dijkstra).
 */
struct State {
    double dist;    // Distancia/costo acumulado desde el origen
    int rec;        // Número de recargas realizadas en el camino
    int v;          // Nodo actual
    
    /**
     * @brief Operador de comparación invertido para min-heap
     * 
     * Orden lexicográfico: primero por distancia, luego por recargas.
     * Se invierte (> en lugar de <) porque priority_queue es max-heap por defecto.
     */
    bool operator<(const State& o) const {
        if (dist != o.dist) return dist > o.dist; // Invertido: menor distancia = mayor prioridad
        return rec > o.rec; // Invertido: menos recargas = mayor prioridad
    }
};

/**
 * DIJKSTRA LEXICOGRÁFICO
 * =======================
 * Algoritmo de Dijkstra modificado con orden lexicográfico: (distancia, recargas).
 * 
 * Encuentra los caminos de costo mínimo desde start_id a todos los demás nodos,
 * considerando primero la minimización de distancia/costo, y en caso de empate,
 * la minimización del número de recargas.
 * 
 * Restricción de batería: Solo puede moverse por aristas que ya fueron filtradas
 * por build_graph() (dist ≤ B_max).
 * 
 * Asunción: Al llegar a una estación, la batería se recarga al 100%.
 * 
 * @param start_id Nodo origen
 * @param[out] distG distG[v] = costo mínimo desde start_id hasta v
 * @param[out] recG recG[v] = número de recargas en el camino mínimo
 * @param[out] prev prev[v] = predecesor de v en el árbol de caminos mínimos
 */
void evolutionaryAlgo::dijkstra_lex(int start_id, vector<double>& distG, vector<int>& recG, vector<int>& prev) {
    const double INF = numeric_limits<double>::infinity();
    
    // Inicializar distancias y recargas a infinito
    distG.assign(N, INF); 
    recG.assign(N, __INT_MAX__);
    prev.assign(N, -1);
    
    // Inicializar nodo origen
    priority_queue<State> pq; 
    distG[start_id] = 0.0; 
    recG[start_id] = 0; 
    pq.push(State{0.0, 0, start_id}); 
    
    // Algoritmo de Dijkstra
    while (!pq.empty()) {
        State s = pq.top(); 
        pq.pop(); 
        int u = s.v; 
        
        // Si este estado está desactualizado, ignorarlo
        if (s.dist > distG[u] || (s.dist == distG[u] && s.rec > recG[u])) continue; 
        
        // Explorar vecinos
        for (const auto& e : adj[u]) {
            int v = e.to; 
            double new_dist = distG[u] + e.w; 
            
            // Incrementar contador de recargas si llegamos a una estación
            int new_rec = recG[u] + (e.isStation ? 1 : 0); 
            
            // Verificar si este camino es mejor (criterio lexicográfico)
            bool better = false; 
            if (new_dist < distG[v]) {
                better = true; // Mejor distancia
            } else if (new_dist == distG[v] && new_rec < recG[v]) {
                better = true; // Igual distancia pero menos recargas
            }
            
            // Si encontramos un camino mejor, actualizar
            if (better) {
                distG[v] = new_dist; 
                recG[v] = new_rec; 
                prev[v] = u; 
                pq.push(State{new_dist, new_rec, v}); 
            }
        }
    }
}

/**
 * FASE 1: CLASIFICACIÓN DE NODOS
 * ================================
 * Asigna un tipo (Depot, Client, Station) a cada nodo según su índice.
 * 
 * Estructura de índices del problema:
 * - Nodo 0: Depósito
 * - Nodos 1..P: Clientes
 * - Nodos P+1..N-1: Estaciones de recarga
 * 
 * También construye el conjunto U de nodos obligatorios (depósito + clientes)
 * que serán los únicos nodos considerados por el algoritmo genético.
 */
void evolutionaryAlgo::build_types() {
    // Inicializar todos como clientes (luego se corrigen depósito y estaciones)
    type.assign(N, NodeType::Client);
    
    // Asignar tipos específicos
    type[0] = NodeType::Depot; // Nodo 0 siempre es el depósito
    
    for (int i = 1; i <= P; i++) {
        type[i] = NodeType::Client; // Nodos 1..P son clientes (redundante pero explícito)
    }
    
    for (int j = P+1; j < P+1+S; j++) {
        type[j] = NodeType::Station; // Nodos P+1..N-1 son estaciones
    }
    
    // Construir conjunto U = {Depósito} ∪ {Clientes}
    // Este conjunto contiene solo los nodos que el GA debe visitar
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

/**
 * FASE 2: CONSTRUCCIÓN DEL GRAFO EXTENDIDO
 * ==========================================
 * Construye la lista de adyacencia del grafo considerando la restricción de autonomía.
 * 
 * Restricciones:
 * 1. Solo existe arista a→b si dist[a][b] ≤ B_max (el vehículo puede llegar con una carga)
 * 2. RESTRICCIÓN CRÍTICA: Solo las estaciones pueden ser nodos intermedios
 *    - Desde Depot/Cliente: puede ir a cualquier nodo (depot, cliente, estación)
 *    - Desde Estación: solo puede ir a Depot/Cliente (para cerrar el camino)
 *    Esto garantiza que los clientes nunca sean nodos de tránsito en caminos preprocesados.
 * 
 * Cálculo de peso:
 * - w = C_km × dist[a][b] + C_rec (si b es estación)
 * - w = C_km × dist[a][b] + 0 (si b es cliente o depósito)
 * 
 * Asunción: Al llegar a una estación, se realiza una recarga completa al 100%.
 * El costo C_rec se cobra al LLEGAR a la estación.
 */
void evolutionaryAlgo::build_graph() {
    // Inicializar lista de adyacencia vacía para cada nodo
    adj.assign(N, {}); 
    
    // Construir aristas entre pares de nodos según reglas de conectividad
    for (int a = 0; a < N; a++) {
        for (int b = 0; b < N; b++) {
            if (a == b) continue; // No hay auto-loops
            
            double d = dist[a][b]; // Distancia euclidiana
            
            // Solo crear arista si está dentro de la autonomía del vehículo
            if (d <= B_max) {
                // Determinar si esta arista es válida según el tipo de nodos
                bool valid_edge = false;
                
                if (type[a] == NodeType::Depot || type[a] == NodeType::Client) {
                    // Desde Depot o Cliente: puede ir a cualquier nodo
                    valid_edge = true;
                } else if (type[a] == NodeType::Station) {
                    // Desde Estación: solo puede ir a Depot o Cliente
                    // (no puede ir a otra estación, evitando cadenas de estaciones)
                    if (type[b] == NodeType::Depot || type[b] == NodeType::Client) {
                        valid_edge = true;
                    }
                }
                
                if (valid_edge) {
                    // Calcular peso: costo por kilómetro + posible costo de recarga
                    double w = C_km * d + (type[b] == NodeType::Station ? C_rec : 0.0); 
                    
                    // Agregar arista a la lista de adyacencia
                    adj[a].push_back(Edge{b, w, type[b] == NodeType::Station});
                }
            }
            // Si d > B_max o arista no válida, no se crea la arista
        }
    }
}

/**
 * CONSTRUCTOR
 * ===========
 * Lee la instancia del problema desde un archivo y ejecuta el preprocesamiento completo.
 * 
 * Formato del archivo (valores separados por whitespace, puede estar en múltiples líneas):
 * 1. B Q B_max C_km C_rec P S
 * 2. demand[0] demand[1] ... demand[P-1]  (demandas de clientes 1..P)
 * 3. N
 * 4. x_0 y_0  (coordenadas nodo 0: depósito)
 *    x_1 y_1  (coordenadas nodo 1: cliente 1)
 *    ...
 *    x_P y_P  (coordenadas nodo P: cliente P)
 *    x_{P+1} y_{P+1}  (coordenadas nodo P+1: estación 1)
 *    ...
 *    x_{N-1} y_{N-1}  (coordenadas nodo N-1: estación S)
 * 
 * Estructura de nodos:
 * - Nodo 0: Depósito (sin demanda)
 * - Nodos 1..P: Clientes (con demanda)
 * - Nodos P+1..N-1: Estaciones de recarga (sin demanda)
 * - N debe ser igual a 1 + P + S
 * 
 * Proceso:
 * 1. Leer y validar parámetros
 * 2. Calcular matriz de distancias euclidianas
 * 3. Clasificar nodos y construir conjunto U
 * 4. Construir grafo extendido con restricción B_max
 * 5. Preprocesar caminos óptimos entre nodos en U
 */
evolutionaryAlgo::evolutionaryAlgo(const string& filename) {
    ifstream file(filename); 
    if (!file) throw runtime_error("No se pudo abrir " + filename);
    
    // Leer parámetros del problema
    file >> B >> Q >> B_max >> C_km >> C_rec >> P >> S;
    
    // Validar parámetros básicos
    if (B <= 0) throw runtime_error("B (vehículos) debe ser > 0");
    if (Q <= 0) throw runtime_error("Q (capacidad) debe ser > 0");
    if (B_max <= 0) throw runtime_error("B_max (autonomía) debe ser > 0");
    if (C_km < 0) throw runtime_error("C_km (costo/km) debe ser >= 0");
    if (C_rec < 0) throw runtime_error("C_rec (costo recarga) debe ser >= 0");
    if (P <= 0) throw runtime_error("P (clientes) debe ser > 0");
    if (S < 0) throw runtime_error("S (estaciones) debe ser >= 0");
    
    // Leer demandas de los clientes
    // NOTA: demand[i] corresponde al cliente con node_id = i+1
    demand.resize(P); 
    for (int i = 0; i < P; i++) {
        file >> demand[i];
        if (demand[i] <= 0) {
            throw runtime_error("Demanda del cliente " + to_string(i+1) + " debe ser > 0");
        }
        if (demand[i] > Q) {
            throw runtime_error("Demanda del cliente " + to_string(i+1) + 
                              " excede capacidad del vehículo");
        }
    }
    
    // Leer número total de nodos y validar
    file >> N;
    int expected_N = 1 + P + S;
    if (N != expected_N) {
        throw runtime_error("N=" + to_string(N) + " pero debería ser 1+P+S=" + 
                          to_string(expected_N));
    }
    
    // Leer coordenadas de todos los nodos
    coords.resize(N);
    for (auto& p : coords) {
        file >> p.first >> p.second; 
    }
    
    // FASE 0: Calcular matriz de distancias euclidianas
    dist = matrix_distances(coords, P, S);
    
    // PREPROCESAMIENTO DEL GRAFO (3 fases)
    build_types();          // FASE 1: Clasificar nodos y construir U
    build_graph();          // FASE 2: Construir grafo extendido con restricción B_max
    preprocess_paths();     // FASE 3: Calcular caminos óptimos entre nodos en U
    
    // Al terminar, W, Rcnt y PathUV están listos para ser usados por split
}

/**
 * HELPER: OBTENER DEMANDA DE UN CLIENTE
 * ======================================
 * Función helper para acceder a la demanda de un cliente de forma segura.
 * 
 * @param node_id ID del nodo (debe ser un cliente: node_id ∈ [1, P])
 * @return Demanda del cliente
 * 
 * Nota: Internamente convierte node_id a índice del vector demand.
 *       demand[node_id - 1] = demanda del cliente con node_id
 */
int evolutionaryAlgo::get_client_demand(int node_id) const {
    // Validar que es un cliente válido
    if (node_id < 1 || node_id > P) {
        throw runtime_error("node_id=" + to_string(node_id) + 
                          " no es un cliente válido (debe estar en [1," + to_string(P) + "])");
    }
    
    // Validar tipo de nodo
    if (type[node_id] != NodeType::Client) {
        throw runtime_error("node_id=" + to_string(node_id) + " no es un cliente");
    }
    
    return demand[node_id - 1];
}



/**
 * FASE 0: CÁLCULO DE MATRIZ DE DISTANCIAS
 * =========================================
 * Calcula la distancia euclidiana entre todos los pares de nodos.
 * 
 * @param coords Vector de coordenadas (x,y) de cada nodo
 * @param P Número de clientes
 * @param S Número de estaciones de recarga
 * @return Matriz simétrica N×N donde dist[i][j] = distancia euclidiana entre i y j
 *         N = 1 (depósito) + P (clientes) + S (estaciones)
 */
vector<vector<double>> matrix_distances(const vector<pair<int,int>>& coords, int P, int S) {
    int N = 1 + P + S; // Total de nodos
    vector<vector<double>> matriz(N, vector<double>(N, 0.0)); 
    
    // Calcular distancias solo para el triángulo superior (matriz simétrica)
    for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
            // Distancia euclidiana: d = √[(x_j - x_i)² + (y_j - y_i)²]
            double dx = coords[j].first - coords[i].first;
            double dy = coords[j].second - coords[i].second;
            double d = sqrt(dx * dx + dy * dy);
            
            // Matriz simétrica
            matriz[i][j] = d;
            matriz[j][i] = d;
        }
    }
    return matriz; 
}

void evolutionaryAlgo::initialize_parameters(int pop_size, int gens, double cross_rate, double mut_rate, int ls_iterations) {
    // Configuración del algoritmo genético por defecto
    population_size = pop_size;      // Tamaño de la población 
    generations = gens;          // Número de generaciones a ejecutar
    crossover_rate = cross_rate;       // Tasa de cruce entre individuos
    mutation_rate = mut_rate;        // Tasa de mutación de genes
    local_search_iterations = ls_iterations;  // Iteraciones máximas de búsqueda local
}

void evolutionaryAlgo::initialize_population() {
    // Calcular el número máximo de permutaciones posibles
    // Para P clientes, hay P! pemutaciones posibles
    auto factorial = [](int n) -> unsigned long long {
        unsigned long long result = 1;
        for (int i = 2; i <= n; i++) {
            result *= i;
            if (result > 1e9) return 1e9; // Limitar para evitar overflow
        }
        return result;
    };

    unsigned long long max_permutations = factorial(P);

    // Ajustar el tamaño de la población si excede el número de permutaciones
    if (population_size > max_permutations) {
        cout << "Advertencia: El tamaño de la población (" << population_size 
             << ") excede el número máximo de permutaciones posibles (" 
             << max_permutations << "). Ajustando tamaño de población a " 
             << max_permutations << "." << endl;
        population_size = max_permutations;
    }

    // Inicializar generador de números aleatorios
    random_device rd; 
    mt19937 gen(rd());

    // Crear vector inicial de clientes [1, 2, ..., P]
    vector<int> base_individual(P);
    iota(base_individual.begin(), base_individual.end(), 1); 

    // Reservar espacio en la población 
    population.clear();
    population.reserve(population_size);

    // Generar individuos con permutaciones aleatorias únicas
    int intentos_fallidos = 0; 
    const int MAX_INTENTOS_FALLIDOS = population_size * 10; 

    while (population.size() < population_size)  {
        // Generar permutación aleatoria usando shuffle
        shuffle(base_individual.begin(), base_individual.end(), gen);

        // Verificar unicidad
        if (find(population.begin(), population.end(), base_individual) == population.end()) {
            population.push_back(base_individual);
            intentos_fallidos = 0; // Reiniciar contador
        } else {
            intentos_fallidos++;

            // Caso borde: Si llevamos muchos intentos fallidos, 
            // estamos cerca del límite de permutaciones
            if (intentos_fallidos >= MAX_INTENTOS_FALLIDOS) {
                cerr << "Advertencia: Después de " << MAX_INTENTOS_FALLIDOS 
                     << " intentos, solo se generaron " << population.size() 
                     << " individuos únicos de " << population_size << " solicitados." << endl;
                cerr << "    Procediendo con población de tamaño " << population.size() << endl;
                population_size = population.size(); // Ajustar al tamaño real
                break;
                    }
                }
    }
    
    // Logging deshabilitado

}

/**
 * FUNCIÓN SPLIT - Algoritmo de Partición Óptima
 * ==============================================
 * Implementa el algoritmo split del paper de Prins para VRP, adaptado a EVRP.
 * 
 * Dado un cromosoma (permutación de clientes), construye un grafo auxiliar H donde:
 * - Nodos: 0 a P (P = número de clientes)
 * - Arco (i,j): representa una ruta que atiende chromosome[i], chromosome[i+1], ..., chromosome[j-1]
 * - Peso del arco: costo total de esa ruta (depósito → clientes → depósito)
 * 
 * Usa programación dinámica (Bellman) para encontrar el camino de costo mínimo de 0 a P,
 * lo cual corresponde a la partición óptima en rutas.
 * 
 * La función está dividida en dos partes:
 * 1. Programación dinámica: calcula V[] (costos) y Pred[] (predecesores)
 * 2. Reconstrucción: llama a reconstruct_solution_from_split() para decodificar la solución
 * 
 * @param chromosome Permutación de node_ids de clientes [1..P]
 * @return Solución con rutas óptimas, costo total y factibilidad
 */
Solution evolutionaryAlgo::split(const vector<int>& chromosome) const {
    int n = chromosome.size();  // Número de clientes en el cromosoma
    const double INF = numeric_limits<double>::infinity();
    
    // Arrays para programación dinámica
    vector<double> V(n + 1, INF);  // V[i] = costo mínimo para atender clientes 0..i-1
    vector<int> Pred(n + 1, -1);   // Pred[i] = predecesor de i en el camino óptimo
    
    V[0] = 0.0;  // Costo base: no hay clientes atendidos
    
    // Para cada posición inicial i
    for (int i = 0; i < n; i++) {
        if (!isfinite(V[i])) continue;  // Si i no es alcanzable, saltar
        
        int load = 0;      // Carga acumulada en la ruta actual
        double cost = 0.0; // Costo acumulado de la ruta actual
        int j = i;         // Posición final de la ruta
        
        // Intentar extender la ruta desde i hasta j
        while (j < n) {
            // Agregar cliente chromosome[j] a la ruta
            int client_id = chromosome[j];
            load += get_client_demand(client_id);
            
            // Verificar restricción de capacidad
            if (load > Q) break;  // No se puede agregar más clientes
            
            // Calcular costo de la ruta [i..j]
            if (j == i) {
                // Primera cliente en la ruta: Depósito → client → Depósito
                double cost_depot_to_client = W[0][client_id];
                double cost_client_to_depot = W[client_id][0];
                
                if (!isfinite(cost_depot_to_client) || !isfinite(cost_client_to_depot)) {
                    break;  // Cliente no alcanzable desde depósito
                }
                
                cost = cost_depot_to_client + cost_client_to_depot;
            } else {
                // Agregar cliente a ruta existente
                int prev_client = chromosome[j - 1];
                double cost_between = W[prev_client][client_id];
                double cost_to_depot = W[client_id][0];
                double cost_prev_to_depot = W[prev_client][0];
                
                if (!isfinite(cost_between) || !isfinite(cost_to_depot)) {
                    break;  // Arco no factible
                }
                
                // Actualizar costo: quitar arco previo→depósito, agregar previo→actual→depósito
                cost = cost - cost_prev_to_depot + cost_between + cost_to_depot;
            }
            
            // Actualizar programación dinámica si encontramos mejor camino
            if (V[i] + cost < V[j + 1]) {
                V[j + 1] = V[i] + cost;
                Pred[j + 1] = i;
            }
            
            j++;  // Intentar agregar siguiente cliente
        }
    }
    
    // Decodificar solución desde Pred[] usando función separada
    return reconstruct_solution_from_split(chromosome, V, Pred, n);
}

/**
 * RECONSTRUCCIÓN DE SOLUCIÓN DESDE SPLIT
 * =======================================
 * Toma los resultados de la programación dinámica (V y Pred) y reconstruye
 * la solución completa con todas las rutas, calculando cargas y costos.
 * 
 * @param chromosome Permutación de clientes del cromosoma
 * @param V Vector de costos mínimos de la programación dinámica
 * @param Pred Vector de predecesores de la programación dinámica
 * @param n Número de clientes en el cromosoma
 * @return Solución reconstruida con todas las rutas y su factibilidad
 */
Solution evolutionaryAlgo::reconstruct_solution_from_split(const vector<int>& chromosome,
                                                           const vector<double>& V,
                                                           const vector<int>& Pred,
                                                           int n) const {
    const double INF = numeric_limits<double>::infinity();
    Solution sol;
    
    // Verificar si se encontró solución factible
    if (!isfinite(V[n])) {
        sol.is_feasible = false;
        sol.total_cost = INF;
        return sol;
    }
    
    sol.total_cost = V[n];
    sol.is_feasible = true;
    
    // Reconstruir rutas siguiendo predecesores (desde n hasta 0)
    int end = n;
    while (end > 0) {
        int start = Pred[end];
        if (start == -1) {
            // Error: no se puede reconstruir la solución
            sol.is_feasible = false;
            sol.total_cost = INF;
            return sol;
        }
        
        // Crear ruta desde start hasta end-1
        Route route;
        int route_load = 0;
        
        // Agregar clientes de esta ruta
        for (int k = start; k < end; k++) {
            int client_id = chromosome[k];
            route.clients.push_back(client_id);
            route_load += get_client_demand(client_id);
        }
        
        route.load = route_load;
        
        // Calcular costo de esta ruta usando matriz W
        if (route.clients.size() > 0) {
            double route_cost = W[0][route.clients[0]];  // Depósito → primer cliente
            
            // Costos entre clientes consecutivos
            for (size_t k = 0; k < route.clients.size() - 1; k++) {
                route_cost += W[route.clients[k]][route.clients[k + 1]];
            }
            
            // Último cliente → depósito
            route_cost += W[route.clients.back()][0];
            
            route.cost = route_cost;
        } else {
            // Ruta vacía (no debería ocurrir)
            route.cost = 0.0;
        }
        
        // Agregar ruta al inicio (estamos reconstruyendo en reversa)
        sol.routes.insert(sol.routes.begin(), route);
        
        end = start;
    }
    
    sol.num_vehicles = sol.routes.size();
    
    // Verificar restricción de flota
    if (sol.num_vehicles > B) {
        sol.is_feasible = false;
        sol.total_cost = INF;  // Penalización: solución no factible
    }
    
    return sol;
}

/**
 * EVALUACIÓN DE POBLACIÓN
 * ========================
 * Evalúa la aptitud (fitness) de cada individuo en la población usando la función split.
 * 
 * Para cada cromosoma en la población:
 * 1. Aplica split() para obtener la solución óptima
 * 2. Guarda el costo total como fitness (menor es mejor)
 * 3. Si la solución es factible y mejor que la mejor actual, la actualiza
 * 
 * El fitness de un individuo infactible se marca como infinito.
 * 
 * Actualiza:
 * - fitness[i]: costo total de population[i]
 * - best_solution: mejor solución encontrada
 * - best_fitness_index: índice del mejor individuo
 */
void evolutionaryAlgo::evaluate_population() {
    // Redimensionar vector de fitness
    fitness.resize(population.size());
    
    // Inicializar mejor solución con valores por defecto
    double best_cost = numeric_limits<double>::infinity();
    best_fitness_index = -1;
    best_solution.is_feasible = false;
    best_solution.total_cost = numeric_limits<double>::infinity();
    
    // Evaluar cada individuo en la población
    for (size_t i = 0; i < population.size(); i++) {
        // Aplicar split al cromosoma
        Solution sol = split(population[i]);
        
        // Guardar fitness (costo total)
        fitness[i] = sol.total_cost;
        
        // Actualizar mejor solución si es factible y mejor
        if (sol.is_feasible && sol.total_cost < best_cost) {
            best_cost = sol.total_cost;
            best_fitness_index = i;
            best_solution = sol;
        }
    }
    
    // Logging: mostrar estadísticas de la evaluación
    int num_feasible = 0;
    double avg_fitness = 0.0;
    for (size_t i = 0; i < fitness.size(); i++) {
        if (isfinite(fitness[i])) {
            num_feasible++;
            avg_fitness += fitness[i];
        }
    }
    
    // Logging deshabilitado para reducir ruido en la salida
    // (solo se imprime la solución final en run())
}

/**
 * BINARY TOURNAMENT SELECTION
 * ============================
 * Selecciona padres mediante torneos binarios.
 * 
 * En cada torneo:
 * 1. Selecciona dos individuos aleatorios de la población
 * 2. Compara sus fitness (menor es mejor)
 * 3. Selecciona el mejor como padre
 * 
 * Esto ayuda a evitar clones ya que:
 * - Siempre se seleccionan individuos diferentes en cada torneo
 * - Se favorece la diversidad al seleccionar aleatoriamente
 * - El mejor individuo tiene mayor probabilidad de ser seleccionado
 * 
 * Genera una población de padres del mismo tamaño que la población original.
 */
void evolutionaryAlgo::selection() {
    // Inicializar generador de números aleatorios
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, population.size() - 1);
    
    // Limpiar y redimensionar población de padres
    parents.clear();
    parents.reserve(population_size);
    
    // Realizar torneos binarios para seleccionar padres
    for (int i = 0; i < population_size; i++) {
        // Seleccionar dos individuos aleatorios diferentes
        int idx1 = dist(gen);
        int idx2 = dist(gen);
        
        // Asegurar que sean diferentes
        while (idx2 == idx1 && population.size() > 1) {
            idx2 = dist(gen);
        }
        
        // Comparar fitness y seleccionar el mejor (menor fitness = mejor)
        int winner_idx;
        if (fitness[idx1] < fitness[idx2]) {
            winner_idx = idx1;
        } else if (fitness[idx2] < fitness[idx1]) {
            winner_idx = idx2;
        } else {
            // En caso de empate, seleccionar aleatoriamente
            winner_idx = (gen() % 2 == 0) ? idx1 : idx2;
        }
        
        // Agregar el ganador como padre
        parents.push_back(population[winner_idx]);
    }
}

/**
 * ORDER CROSSOVER (OX)
 * ====================
 * Implementa el operador de cruce Order Crossover para permutaciones.
 * 
 * Order Crossover preserva:
 * - El orden relativo de los elementos de un padre
 * - Las posiciones absolutas del otro padre
 * 
 * Algoritmo:
 * 1. Seleccionar un segmento aleatorio del padre 1
 * 2. Copiar ese segmento al hijo en las mismas posiciones
 * 3. Completar el resto del hijo con elementos del padre 2 en el orden
 *    en que aparecen, omitiendo los ya copiados del padre 1
 * 
 * Ejemplo:
 * Padre 1: [1, 2, 3, 4, 5, 6]
 * Padre 2: [4, 5, 6, 1, 2, 3]
 * Segmento: [2, 4] (posiciones 1-2)
 * Hijo: [_, 2, 4, _, _, _]
 * Completar con Padre 2: [5, 2, 4, 6, 1, 3]
 */
void evolutionaryAlgo::crossover() {
    // Inicializar generador de números aleatorios
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> prob_dist(0.0, 1.0);
    // Nueva población de hijos
    vector<vector<int>> offspring;
    offspring.reserve(parents.size());
    
    // Realizar cruces por pares de padres
    for (size_t i = 0; i < parents.size(); i += 2) {
        // Si hay un padre sin pareja, copiarlo directamente
        if (i + 1 >= parents.size()) {
            offspring.push_back(parents[i]);
            continue;
        }
        
        // Decidir si se hace cruce según crossover_rate
        if (prob_dist(gen) > crossover_rate) {
            // No cruzar: copiar padres directamente
            offspring.push_back(parents[i]);
            offspring.push_back(parents[i + 1]);
            continue;
        }
        
        // Realizar cruce OX
        const vector<int>& parent1 = parents[i];
        const vector<int>& parent2 = parents[i + 1];
        
        // Seleccionar dos puntos de corte aleatorios
        uniform_int_distribution<int> pos_dist(0, P - 1);
        int cut1 = pos_dist(gen);
        int cut2 = pos_dist(gen);
        
        // Asegurar que cut1 <= cut2
        if (cut1 > cut2) {
            swap(cut1, cut2);
        }
        
        // Crear dos hijos mediante OX
        vector<int> child1(P, -1);
        vector<int> child2(P, -1);
        
        // Hijo 1: segmento de parent1, resto de parent2
        // Copiar segmento de parent1
        set<int> segment1;
        for (int j = cut1; j <= cut2; j++) {
            child1[j] = parent1[j];
            segment1.insert(parent1[j]);
        }
        
        // Completar hijo 1 con elementos de parent2
        int idx = 0;
        for (int j = 0; j < P; j++) {
            if (child1[j] == -1) {
                // Buscar siguiente elemento de parent2 que no esté en segment1
                while (idx < P && segment1.count(parent2[idx]) > 0) {
                    idx++;
                }
                if (idx < P) {
                    child1[j] = parent2[idx];
                    idx++;
                }
            }
        }
        
        // Hijo 2: segmento de parent2, resto de parent1
        set<int> segment2;
        for (int j = cut1; j <= cut2; j++) {
            child2[j] = parent2[j];
            segment2.insert(parent2[j]);
        }
        
        // Completar hijo 2 con elementos de parent1
        idx = 0;
        for (int j = 0; j < P; j++) {
            if (child2[j] == -1) {
                // Buscar siguiente elemento de parent1 que no esté en segment2
                while (idx < P && segment2.count(parent1[idx]) > 0) {
                    idx++;
                }
                if (idx < P) {
                    child2[j] = parent1[idx];
                    idx++;
                }
            }
        }
        
        // Agregar hijos a la nueva población
        offspring.push_back(child1);
        offspring.push_back(child2);
    }
    
    // Reemplazar población actual con hijos
    population = offspring;
    
    // Ajustar tamaño si es necesario (por redondeo en cruces o padres sin pareja)
    if (population.size() != population_size) {
        if (population.size() > population_size) {
            // Eliminar exceso (mantener los primeros)
            population.resize(population_size);
        } else {
            // Completar con padres aleatorios si faltan
            while (population.size() < population_size && !parents.empty()) {
                uniform_int_distribution<size_t> parent_dist(0, parents.size() - 1);
                population.push_back(parents[parent_dist(gen)]);
            }
        }
    }
}

/**
 * MUTACIÓN - BÚSQUEDA LOCAL
 * ==========================
 * Aplica mutación tipo búsqueda local a los individuos de la población.
 * 
 * La mutación utiliza tres tipos de movimientos:
 * 1. 2-opt: Invierte un segmento de la permutación
 * 2. Relocate: Mueve un cliente de posición i a j
 * 3. Swap: Intercambia dos clientes
 * 
 * Se aplica a cada individuo según mutation_rate. Para cada individuo seleccionado:
 * - Realiza hasta K_max iteraciones de búsqueda local
 * - En cada iteración, selecciona un movimiento aleatorio
 * - Aplica el movimiento y evalúa el fitness
 * - Si mejora estrictamente, acepta el movimiento (first improvement)
 * - Si no mejora en K_max iteraciones, mantiene el individuo original
 */
void evolutionaryAlgo::mutation() {
    // Inicializar generador de números aleatorios
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> prob_dist(0.0, 1.0);
    
    // Aplicar mutación a cada individuo según mutation_rate
    for (size_t i = 0; i < population.size(); i++) {
        // Decidir si se aplica mutación a este individuo
        if (prob_dist(gen) <= mutation_rate) {
            // Aplicar búsqueda local
            local_search_mutation(i);
        }
    }
}

/**
 * BÚSQUEDA LOCAL PARA UN INDIVIDUO
 * ==================================
 * Aplica búsqueda local a un individuo específico usando movimientos aleatorios.
 * 
 * Estrategia: First Improvement
 * - En cada iteración, selecciona un movimiento aleatorio (2-opt, relocate, swap)
 * - Aplica el movimiento y evalúa el fitness
 * - Si mejora estrictamente y es factible, acepta y termina
 * - Si no mejora, continúa hasta K_max iteraciones
 * - Si no encuentra mejora, mantiene el individuo original
 * 
 * @param individual Índice del individuo en la población
 * @return true si se encontró una mejora, false en caso contrario
 */
bool evolutionaryAlgo::local_search_mutation(size_t individual) {
    // Inicializar generador de números aleatorios
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> move_dist(0, 2);  // 0=2-opt, 1=relocate, 2=swap
    uniform_int_distribution<int> pos_dist(0, P - 1);
    
    // Obtener individuo actual y su fitness
    vector<int> current_chromosome = population[individual];
    double current_fitness = fitness[individual];
    
    // Si el fitness es infinito (infactible), no aplicar búsqueda local
    if (!isfinite(current_fitness)) {
        return false;
    }
    
    // Realizar hasta K_max iteraciones
    for (int k = 0; k < local_search_iterations; k++) {
        // Seleccionar movimiento aleatorio
        int move_type = move_dist(gen);
        
        // Crear copia para probar el movimiento
        vector<int> test_chromosome = current_chromosome;
        
        // Aplicar movimiento según el tipo
        if (move_type == 0) {
            // 2-opt: seleccionar dos posiciones y invertir el segmento
            int i = pos_dist(gen);
            int j = pos_dist(gen);
            if (i > j) swap(i, j);
            if (i < j) {  // Solo aplicar si i != j
                apply_2opt(test_chromosome, i, j);
            } else {
                continue;  // Saltar si i == j
            }
        } else if (move_type == 1) {
            // Relocate: mover elemento de i a j
            int i = pos_dist(gen);
            int j = pos_dist(gen);
            if (i != j) {
                apply_relocate(test_chromosome, i, j);
            } else {
                continue;  // Saltar si i == j
            }
        } else {
            // Swap: intercambiar elementos en i y j
            int i = pos_dist(gen);
            int j = pos_dist(gen);
            if (i != j) {
                apply_swap(test_chromosome, i, j);
            } else {
                continue;  // Saltar si i == j
            }
        }
        
        // Evaluar el nuevo cromosoma
        Solution test_sol = split(test_chromosome);
        double test_fitness = test_sol.total_cost;
        
        // Si mejora estrictamente (menor fitness es mejor) y es factible, aceptar
        if (test_sol.is_feasible && test_fitness < current_fitness) {
            // Aceptar la mejora
            population[individual] = test_chromosome;
            fitness[individual] = test_fitness;
            
            // Actualizar mejor solución si es necesario
            if (best_fitness_index < 0 || test_fitness < best_solution.total_cost) {
                best_solution = test_sol;
                best_fitness_index = individual;
            }
            
            return true;  // First improvement: salir después de encontrar mejora
        }
    }
    
    // No se encontró mejora: mantener individuo original (ya está en population[individual])
    return false;
}

/**
 * MOVIMIENTO 2-OPT
 * ================
 * Invierte el segmento entre las posiciones i y j (inclusive).
 * 
 * Ejemplo: [1, 2, 3, 4, 5] con i=1, j=3 → [1, 4, 3, 2, 5]
 * 
 * @param chromosome Permutación a modificar (se modifica in-place)
 * @param i Posición inicial del segmento
 * @param j Posición final del segmento
 */
void evolutionaryAlgo::apply_2opt(vector<int>& chromosome, int i, int j) const {
    while (i < j) {
        swap(chromosome[i], chromosome[j]);
        i++;
        j--;
    }
}

/**
 * MOVIMIENTO RELOCATE
 * ===================
 * Mueve el elemento en posición i a la posición j.
 * 
 * Si i < j: desplaza elementos hacia la izquierda
 * Si i > j: desplaza elementos hacia la derecha
 * 
 * Ejemplo: [1, 2, 3, 4, 5] con i=1, j=3 → [1, 3, 4, 2, 5]
 *          [1, 2, 3, 4, 5] con i=3, j=1 → [1, 4, 2, 3, 5]
 * 
 * @param chromosome Permutación a modificar (se modifica in-place)
 * @param i Posición origen
 * @param j Posición destino
 */
void evolutionaryAlgo::apply_relocate(vector<int>& chromosome, int i, int j) const {
    if (i == j) return;
    
    int value = chromosome[i];
    
    if (i < j) {
        // Desplazar elementos hacia la izquierda
        for (int k = i; k < j; k++) {
            chromosome[k] = chromosome[k + 1];
        }
    } else {
        // Desplazar elementos hacia la derecha
        for (int k = i; k > j; k--) {
            chromosome[k] = chromosome[k - 1];
        }
    }
    
    chromosome[j] = value;
}

/**
 * MOVIMIENTO SWAP
 * ===============
 * Intercambia los elementos en las posiciones i y j.
 * 
 * Ejemplo: [1, 2, 3, 4, 5] con i=1, j=3 → [1, 4, 3, 2, 5]
 * 
 * @param chromosome Permutación a modificar (se modifica in-place)
 * @param i Primera posición
 * @param j Segunda posición
 */
void evolutionaryAlgo::apply_swap(vector<int>& chromosome, int i, int j) const {
    if (i != j) {
        swap(chromosome[i], chromosome[j]);
    }
}

/**
 * EJECUCIÓN DEL ALGORITMO GENÉTICO
 * ==================================
 * Loop principal del algoritmo genético.
 * 
 * Proceso:
 * 1. Inicializa la población
 * 2. Evalúa la población inicial
 * 3. Por cada generación:
 *    - Selecciona padres (Binary Tournament)
 *    - Aplica cruce (Order Crossover)
 *    - Aplica mutación (Búsqueda Local)
 *    - Evalúa la nueva población
 * 4. Al finalizar, imprime la mejor solución
 */
void evolutionaryAlgo::run() {
    // Inicializar población
    initialize_population();
    
    // Evaluar población inicial
    evaluate_population();
    
    // Loop principal de generaciones
    for (int gen = 0; gen < generations; gen++) {
        // Selección de padres
        selection();
        
        // Cruce
        crossover();
        
        // Mutación
        mutation();
        
        // Evaluar nueva población
        evaluate_population();
        
        // Sin logging durante la ejecución (solo se imprime la solución final)
    }
    
    // Imprimir mejor solución en el formato especificado
    if (best_fitness_index >= 0 && best_solution.is_feasible) {
        print_solution(best_solution);
    } else {
        cout << "No se encontró solución factible." << endl;
    }
}

/**
 * IMPRIMIR SOLUCIÓN EN FORMATO ESPECIFICADO
 * ==========================================
 * Imprime la solución en el formato requerido:
 * - Costo total
 * - Para cada vehículo: ruta completa, distancia, recargas
 * - Leyenda explicativa
 */
void evolutionaryAlgo::print_solution(const Solution& solution) const {
    // Imprimir costo total inicial (con 1 decimal según el ejemplo)
    cout << fixed << setprecision(1) << solution.total_cost << endl;
    cout << endl;
    
    // Imprimir información de cada vehículo
    for (size_t i = 0; i < solution.routes.size(); i++) {
        const Route& route = solution.routes[i];
        
        // Expandir ruta para obtener camino completo con estaciones
        vector<int> full_path = expand_route(route);
        
        // Calcular distancia real
        double distance = calculate_route_distance(full_path);
        
        // Contar recargas
        int recharges = count_recharges(full_path);
        
        // Imprimir información del vehículo
        cout << "Vehiculo " << (i + 1) << ":" << endl;
        
        // Imprimir ruta
        cout << "Ruta: ";
        for (size_t j = 0; j < full_path.size(); j++) {
            int node_id = full_path[j];
            if (node_id == 0) {
                cout << "D";
            } else if (type[node_id] == NodeType::Client) {
                cout << "C" << node_id;
            } else if (type[node_id] == NodeType::Station) {
                // Las estaciones están en node_id desde P+1 hasta N-1
                // Necesitamos numerarlas desde 1: R1, R2, R3, ...
                int station_number = node_id - P;
                cout << "R" << station_number;
            }
            if (j < full_path.size() - 1) {
                cout << " -> ";
            }
        }
        cout << endl;
        
        // Imprimir distancia (sin decimales, como entero)
        cout << "Distancia: " << (int)round(distance) << endl;
        
        // Imprimir recargas
        cout << "Recargas: " << recharges << endl;
        cout << endl;
    }
    
    // Imprimir costo total final (sin decimales, como entero)
    cout << "Costo total: " << (int)round(solution.total_cost) << endl;
    cout << endl;
    
    // Imprimir leyenda
    cout << "Donde:" << endl;
    cout << "■ D indica el depósito principal (inicio y fin de la ruta)." << endl;
    cout << "■ Ci representa un cliente atendido." << endl;
    cout << "■ Rj representa una estación de recarga visitada." << endl;
    cout << "■ Distancia corresponde a la distancia total recorrida por el vehículo." << endl;
    cout << "■ Recargas indica cuántas veces el vehículo realizó una recarga." << endl;
    cout << "■ Costo total es el valor global de la solución considerando los costos por distancia y por recarga." << endl;
}

/**
 * EXPANDIR RUTA
 * =============
 * Expande una ruta usando PathUV para incluir estaciones intermedias.
 * 
 * @param route Ruta con solo clientes
 * @return Camino completo: [D, ..., C1, ..., R, ..., C2, ..., D]
 */
vector<int> evolutionaryAlgo::expand_route(const Route& route) const {
    vector<int> full_path;
    
    if (route.clients.empty()) {
        // Ruta vacía: solo depósito
        return {0};
    }
    
    int M = U.size();
    
    // Agregar camino desde depósito (0) al primer cliente
    int first_client = route.clients[0];
    int u_idx_depot = 0;  // Depósito está en U[0]
    int u_idx_first = node_to_U_index(first_client);
    
    if (u_idx_first >= 0) {
        vector<int> path_depot_to_first = PathUV[u_idx_depot * M + u_idx_first];
        // Agregar todos los nodos del camino (incluye D y primer cliente)
        for (int node_id : path_depot_to_first) {
            full_path.push_back(node_id);
        }
    }
    
    // Agregar caminos entre clientes consecutivos
    for (size_t i = 0; i + 1 < route.clients.size(); i++) {
        int current_client = route.clients[i];
        int next_client = route.clients[i + 1];
        int u_idx_current = node_to_U_index(current_client);
        int u_idx_next = node_to_U_index(next_client);
        
        if (u_idx_current >= 0 && u_idx_next >= 0) {
            vector<int> path_between = PathUV[u_idx_current * M + u_idx_next];
            // Agregar todos los nodos excepto el primero (ya agregado como último del camino anterior)
            for (size_t j = 1; j < path_between.size(); j++) {
                full_path.push_back(path_between[j]);
            }
        }
    }
    
    // Agregar camino desde último cliente al depósito
    int last_client = route.clients.back();
    int u_idx_last = node_to_U_index(last_client);
    
    if (u_idx_last >= 0) {
        vector<int> path_last_to_depot = PathUV[u_idx_last * M + u_idx_depot];
        // Agregar todos los nodos excepto el primero (ya agregado como último cliente)
        for (size_t i = 1; i < path_last_to_depot.size(); i++) {
            full_path.push_back(path_last_to_depot[i]);
        }
    }
    
    return full_path;
}

/**
 * CALCULAR DISTANCIA DE RUTA
 * ===========================
 * Calcula la distancia total real sumando distancias euclidianas entre nodos consecutivos.
 * 
 * @param path Camino completo de nodos
 * @return Distancia total
 */
double evolutionaryAlgo::calculate_route_distance(const vector<int>& path) const {
    if (path.size() < 2) return 0.0;
    
    double total_distance = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        total_distance += dist[path[i]][path[i + 1]];
    }
    return total_distance;
}

/**
 * CONTAR RECARGAS
 * ===============
 * Cuenta el número de estaciones de recarga en el camino.
 * 
 * @param path Camino completo de nodos
 * @return Número de estaciones visitadas
 */
int evolutionaryAlgo::count_recharges(const vector<int>& path) const {
    int count = 0;
    for (int node_id : path) {
        if (type[node_id] == NodeType::Station) {
            count++;
        }
    }
    return count;
}

/**
 * CONVERTIR NODE_ID A ÍNDICE EN U
 * ================================
 * Convierte un node_id a su índice en el conjunto U.
 * 
 * @param node_id ID del nodo
 * @return Índice en U, o -1 si no está en U
 */
int evolutionaryAlgo::node_to_U_index(int node_id) const {
    for (size_t i = 0; i < U.size(); i++) {
        if (U[i] == node_id) {
            return i;
        }
    }
    return -1;  // No está en U (no debería ocurrir para clientes)
}