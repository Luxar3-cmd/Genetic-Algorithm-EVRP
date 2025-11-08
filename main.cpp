# include <iostream>
# include "GA.h"
using namespace std;

int main() {
    // Leí los datos
    evolutionaryAlgo EVRP("instancias/instancia1.txt"); 
    // Configurar el algoritmo con los parámetros
    EVRP.initialize_parameters(30, 40, 0.8, 0.02); // Población, generaciones, cruce, mutación
    // Ejecutarlo
    EVRP.run();
    // Entregar resultados

    return 0; 
}