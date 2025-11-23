/**
 * @file main.cpp
 * @brief Programa principal para resolver EVRP con algoritmo genético
 *
 * Uso:
 *   ./bin/evrp_solver <archivo_instancia> [battery_step] [pop_size]
 * [generations]
 *
 * Ejemplos:
 *   ./bin/evrp_solver instancias/instancia1.txt
 *   ./bin/evrp_solver instancias/instancia1.txt 1 50 100
 *   ./bin/evrp_solver instancias/instancia1.txt 10 30 50
 */

#include "GA.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace std::chrono;

/**
 * @brief Imprime instrucciones de uso del programa
 * @param program_name Nombre del ejecutable
 */
void print_usage(const char *program_name) {
  cout << "Uso: " << program_name
       << " <instancia> [battery_step] [pop_size] [generations]" << endl;
  cout << endl;
  cout << "Parámetros:" << endl;
  cout << "  instancia     : Ruta al archivo de instancia (requerido)" << endl;
  cout << "  battery_step  : Discretización de batería (default: 1)" << endl;
  cout << "                  Valores menores = más preciso, más lento" << endl;
  cout << "                  Ejemplos: 0.1 (ultra-preciso), 1 (balance), 10 "
          "(rápido)"
       << endl;
  cout << "  pop_size      : Tamaño de población (default: 50)" << endl;
  cout << "  generations   : Número de generaciones (default: 100)" << endl;
  cout << endl;
  cout << "Ejemplos:" << endl;
  cout << "  " << program_name << " instancias/instancia1.txt" << endl;
  cout << "  " << program_name << " instancias/instancia1.txt 1 50 100" << endl;
  cout << "  " << program_name << " instancias/instancia1.txt 10 30 50" << endl;
}

/**
 * @brief Función principal
 */
int main(int argc, char **argv) {
  try {
    // Verificar argumentos
    if (argc < 2) {
      print_usage(argv[0]);
      return 1;
    }

    // Parámetros
    string instance_file = argv[1];
    double battery_step = (argc > 2) ? atof(argv[2]) : 1.0;
    int pop_size = (argc > 3) ? atoi(argv[3]) : 50;
    int generations = (argc > 4) ? atoi(argv[4]) : 100;

    // Validaciones
    if (battery_step <= 0) {
      cerr << "Error: battery_step debe ser > 0" << endl;
      return 1;
    }
    if (pop_size <= 0) {
      cerr << "Error: pop_size debe ser > 0" << endl;
      return 1;
    }
    if (generations <= 0) {
      cerr << "Error: generations debe ser > 0" << endl;
      return 1;
    }

    cout << "╔═══════════════════════════════════════════════════════════╗"
         << endl;
    cout << "║          EVRP Solver - Algoritmo Genético                ║"
         << endl;
    cout << "╚═══════════════════════════════════════════════════════════╝"
         << endl;
    cout << endl;
    cout << "Archivo de instancia: " << instance_file << endl;
    cout << "Discretización:       " << battery_step << endl;
    cout << "Tamaño población:     " << pop_size << endl;
    cout << "Generaciones:         " << generations << endl;
    cout << endl;

    // Iniciar cronómetro
    auto start_time = high_resolution_clock::now();

    // Crear GA
    cout << "Inicializando preprocesamiento..." << endl;
    GA ga(instance_file, battery_step);

    auto preprocess_time = high_resolution_clock::now();
    auto preprocess_duration =
        duration_cast<milliseconds>(preprocess_time - start_time);
    cout << "Preprocesamiento completado en " << preprocess_duration.count()
         << " ms" << endl;

    // Configurar parámetros
    ga.set_parameters(
        /* pop_size */ pop_size,
        /* generations */ generations,
        /* crossover_rate */ 0.8,
        /* mutation_rate */ 0.2);

    // Ejecutar GA
    Solution solution = ga.run();

    // Finalizar cronómetro
    auto end_time_ga =
        high_resolution_clock::now(); // Renamed to avoid conflict
    auto ga_duration =
        duration_cast<milliseconds>(end_time_ga - preprocess_time);

    // Reporte Final
    cout << "\n╔═══════════════════════════════════════════════════════════╗"
         << endl;
    cout << "║                    REPORTE FINAL                          ║"
         << endl;
    cout << "╚═══════════════════════════════════════════════════════════╝"
         << endl;
    cout << "\nSolución: " << (solution.is_feasible ? "FACTIBLE" : "INFACTIBLE")
         << endl;
    if (!solution.is_feasible) {
      cout << "Razón: "
           << feasibility_reason_to_string(solution.infeasibility_reason)
           << endl;
      cout << "Detalles: " << solution.infeasibility_details << endl;
    }
    // Pre-calcular rutas detalladas y contar sub-rutas (trips)
    vector<vector<int>> all_detailed_paths;
    int total_trips = 0;
    int P = ga.get_preprocess().get_P(); // Número de clientes

    for (const auto &route : solution.routes) {
      // Construir secuencia de nodos (Depot -> Clientes -> Depot)
      vector<int> route_nodes;
      route_nodes.push_back(0);
      route_nodes.insert(route_nodes.end(), route.clients.begin(),
                         route.clients.end());
      route_nodes.push_back(0);

      // Obtener ruta detallada
      vector<int> detailed_path = ga.get_split().get_detailed_path(route_nodes);
      all_detailed_paths.push_back(detailed_path);

      // Contar trips: número de veces que aparece el depósito (0) menos 1
      // Ejemplo: D -> C1 -> D (2 ceros -> 1 trip)
      // Ejemplo: D -> C1 -> D -> C2 -> D (3 ceros -> 2 trips)
      int depot_count = 0;
      for (int node : detailed_path) {
        if (node == 0)
          depot_count++;
      }
      total_trips += (depot_count - 1);
    }

    cout << "\nCosto total:          " << fixed << setprecision(2)
         << solution.total_cost << endl;
    cout << "Vehículos usados:     " << solution.routes.size() << endl;
    cout << "Número de rutas:      " << total_trips << endl;

    cout << "\nDetalle de rutas:" << endl;

    for (size_t i = 0; i < solution.routes.size(); i++) {
      const auto &route = solution.routes[i];
      const auto &detailed_path = all_detailed_paths[i];

      cout << "  Vehiculo " << i + 1 << ":" << endl;
      cout << "  Ruta: ";

      for (size_t j = 0; j < detailed_path.size(); j++) {
        int node = detailed_path[j];
        if (node == 0) {
          cout << "D";
        } else if (node <= P) {
          cout << "C" << node;
        } else {
          cout << "R" << (node - P);
        }

        if (j < detailed_path.size() - 1) {
          cout << " -> ";
        }
      }
      cout << endl;
      cout << "  Distancia: " << fixed << setprecision(2) << route.cost << endl;
      cout << "  Recargas: " << route.recharges << endl;
      cout << endl;
    }

    // Finalizar cronómetro para el tiempo total de ejecución (incluyendo
    // reporte)
    auto end_time_total = high_resolution_clock::now();
    auto total_duration =
        duration_cast<milliseconds>(end_time_total - start_time);

    cout << "Tiempo de ejecución:" << endl;
    cout << "  Preprocesamiento:   " << preprocess_duration.count() << " ms"
         << endl;
    cout << "  Algoritmo genético: " << ga_duration.count() << " ms" << endl;
    cout << "  TIEMPO TOTAL:       " << total_duration.count() << " ms" << endl;
    cout << "                      (" << fixed << setprecision(2)
         << total_duration.count() / 1000.0 << " segundos)" << endl;

    return solution.is_feasible ? 0 : 1;
  } catch (const exception &e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
  }
}
