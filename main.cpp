/**
 * Programa Principal - Algoritmo Genético para EVRP
 * ==================================================
 * 
 * Uso: ./evrp_solver <archivo_instancia>
 * 
 * Ejemplo: ./evrp_solver instancias/instancia4.txt
 */
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include "GA.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    // Verificar argumentos de línea de comandos
    if (argc != 2) {
        cerr << "Uso: " << argv[0] << " <archivo_instancia>" << endl;
        cerr << "Ejemplo: " << argv[0] << " instancias/instancia4.txt" << endl;
        return 1;
    }
    
    string filename = argv[1];
    
    try {
        // Iniciar medición de tiempo
        auto start_time = high_resolution_clock::now();
        
        // Crear instancia del algoritmo genético
        // Esto carga la instancia y ejecuta el preprocesamiento
        auto preprocess_start = high_resolution_clock::now();
        evolutionaryAlgo EVRP(filename);
        auto preprocess_end = high_resolution_clock::now();
        auto preprocess_duration = duration_cast<microseconds>(preprocess_end - preprocess_start);
        
        // Configurar parámetros del algoritmo genético
        // Parámetros: población, generaciones, tasa_cruce, tasa_mutación, iteraciones_búsqueda_local
        EVRP.initialize_parameters(30, 20, 0.8, 0.2, 20);
        
        // Ejecutar algoritmo genético
        // Esto ejecuta el GA y imprime la solución final
        auto ga_start = high_resolution_clock::now();
        EVRP.run();
        auto ga_end = high_resolution_clock::now();
        auto ga_duration = duration_cast<microseconds>(ga_end - ga_start);
        
        // Finalizar medición de tiempo total
        auto end_time = high_resolution_clock::now();
        auto total_duration = duration_cast<microseconds>(end_time - start_time);
        
        // Imprimir tiempos de ejecución
        cout << "\n=== TIEMPOS DE EJECUCIÓN ===" << endl;
        cout << fixed << setprecision(6);
        
        double preprocess_sec = preprocess_duration.count() / 1000000.0;
        double ga_sec = ga_duration.count() / 1000000.0;
        double total_sec = total_duration.count() / 1000000.0;
        
        cout << "Preprocesamiento: " << preprocess_sec << " segundos";
        if (preprocess_sec < 0.001) {
            cout << " (" << preprocess_duration.count() << " μs)";
        } else if (preprocess_sec < 1.0) {
            cout << " (" << (preprocess_duration.count() / 1000.0) << " ms)";
        }
        cout << endl;
        
        cout << "Algoritmo Genético: " << ga_sec << " segundos";
        if (ga_sec < 0.001) {
            cout << " (" << ga_duration.count() << " μs)";
        } else if (ga_sec < 1.0) {
            cout << " (" << (ga_duration.count() / 1000.0) << " ms)";
        }
        cout << endl;
        
        cout << "Tiempo total: " << total_sec << " segundos";
        if (total_sec < 0.001) {
            cout << " (" << total_duration.count() << " μs)";
        } else if (total_sec < 1.0) {
            cout << " (" << (total_duration.count() / 1000.0) << " ms)";
        }
        cout << endl;
        
        // Información adicional sobre parámetros y estadísticas
        cout << "\n=== PARÁMETROS DE EJECUCIÓN ===" << endl;
        cout << "Población: " << EVRP.population_size << endl;
        cout << "Generaciones: " << EVRP.generations << endl;
        cout << "Tasa de cruce: " << EVRP.crossover_rate << endl;
        cout << "Tasa de mutación: " << EVRP.mutation_rate << endl;
        cout << "Iteraciones búsqueda local: " << EVRP.local_search_iterations << endl;
        
        // Calcular estadísticas de rendimiento
        long long total_evaluations = EVRP.population_size * (EVRP.generations + 1);  // +1 por evaluación inicial
        double time_per_evaluation = ga_sec / total_evaluations;
        cout << "\n=== ESTADÍSTICAS DE RENDIMIENTO ===" << endl;
        cout << "Total de evaluaciones: " << total_evaluations << endl;
        cout << "Tiempo por evaluación: " << scientific << setprecision(3) << time_per_evaluation 
             << " segundos" << fixed << endl;
        cout << "Evaluaciones por segundo: " << (1.0 / time_per_evaluation) << endl;
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}