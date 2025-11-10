/**
 * Programa para mostrar matrices de distancias y preprocesamiento
 * ===============================================================
 * 
 * Uso: ./show_matrices <archivo_instancia>
 * 
 * Ejemplo: ./show_matrices instancias/instancia2.txt
 */
#include <iostream>
#include <string>
#include "GA.h"

using namespace std;

int main(int argc, char* argv[]) {
    // Verificar argumentos de línea de comandos
    if (argc != 2) {
        cerr << "Uso: " << argv[0] << " <archivo_instancia>" << endl;
        cerr << "Ejemplo: " << argv[0] << " instancias/instancia2.txt" << endl;
        return 1;
    }
    
    string filename = argv[1];
    
    try {
        // Crear instancia del algoritmo genético
        // Esto carga la instancia y ejecuta el preprocesamiento
        evolutionaryAlgo EVRP(filename);
        
        // Imprimir información de nodos
        EVRP.print_node_info();
        
        // Imprimir matriz de distancias euclidianas
        EVRP.print_distance_matrix();
        
        // Imprimir matriz preprocesada W
        EVRP.print_preprocessed_matrix();
        
        // Imprimir matriz de recargas
        EVRP.print_recharge_count_matrix();
        
        // Imprimir caminos completos
        EVRP.print_paths();
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}

