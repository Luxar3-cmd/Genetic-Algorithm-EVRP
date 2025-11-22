#ifndef GENETICO_H
#define GENETICO_H

#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

using namespace std;

/**
 * Esta función la distancia euclidiana entre todos los nodos.
 * Retorna una matriz de distancias euclidianas.
 */
vector<vector<double>> matrixDistances (const vector<pair<int,int>>& xy, int nClients, int nStations); 

class AE {
    private:
    // Parámetros del algoritmo evolutivo.
    int popSize, generations, localSearchIter;
    double crossRate, mutRate;
    
    // Datos de la instancia
    int cantVehiculos, capVehiculos, autonoMax, Ckm, Crec, nClients, nStations, totalNodes;
    vector<int> demand; // Demanda de cada cliente
    vector<pair<int,int>> coords; // Coordenadas de cada nodo.
    vector<vector<double>> euclidianDistances; // Matriz de distancias euclidianas.
    
    public:
    
    /**
     * CONSTRUCTOR DE LA CLASE 
     * 
     * Esta función debe inicializar lo que lee de las instancias
     */
    AE(const string& filename);
    
    /**
     * Función que inicializa los parámetros del 
     * algoritmo evolutivo.
     */
    void initParameters (int popS, int gens, double crossR, double mutR, int localSearchI);
    
};

#endif // GENETICO_H