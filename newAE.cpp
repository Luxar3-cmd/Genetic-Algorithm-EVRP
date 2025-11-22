#include "NuevaVersion.hpp"

using namespace std;

vector<vector<double>> matrixDistances(const vector<pair<int,int>>& xy, int nClients, int nStations) {
    int totalNodes  = 1 + nClients + nStations;
    vector<vector<double>> matrix(totalNodes, vector<double>(totalNodes,0.0));

    // Calculo de distancias
    for (int i = 0; i < totalNodes; i++) {
        for (int j = i; j < totalNodes; j++) {
            double dx =  xy[j].first - xy[i].first;
            double dy = xy[j].second - xy[i].second;
            double distance = sqrt(dx * dx + dy * dy);

            matrix[i][j] = distance;
            // Matriz simétrica
            matrix[j][i] = distance;
            
        }
    }
    /*
    // TODO: DEBUG
    cout << "Imprimiendo matriz" << endl;
    for (auto& row: matrix) {
        for (auto& num: row) {
            cout << to_string(num) + " "; 
        }
        cout << endl;
    }
    */
    return matrix;

}

// =====================================================================
// === Setup inicial de valores y parámetros del algoritmo evolutivo ===
// =====================================================================

AE::AE(const string& filename) {
    ifstream archivo(filename);
    // Caso en que no se puede abrir
    if (!archivo) throw runtime_error("No se pudo abrir " + filename);

    // Lectura de parámetros del problema
    archivo >> cantVehiculos >> capVehiculos >> autonoMax >> Ckm >> Crec >> nClients >> nStations;

    /*
    // TODO: DEBUG
    cout << "-- Valores leídos --" << endl;
    */
    
    // Lectura de las demandas de los clientes
    demand.resize(nClients);
    for (int i = 0; i < nClients; i++) {
        archivo >> demand[i];
        if (demand[i] > capVehiculos) throw runtime_error("Demanda del cliente " + to_string(i+1) + " excede capacidad del vehículo");

    }
    
    /* 
    // TODO: DEBUG
    cout << "-- Demandas leídas --" << endl;
    */
    
    archivo >> totalNodes;
    // Lectura de las coordenadas
    coords.resize(totalNodes);
    for (auto& p: coords) archivo >> p.first >> p.second;

    /*
    // TODO: DEBUG
    cout << "-- Coordenadas leídas --" << endl;
    */

    // Calculo de distancias euclidianas
    euclidianDistances = matrixDistances(coords, nClients, nStations);

    /*
    // TODO: DEBUG
    cout << "-- Distancias calculadas --" << endl;
    */
    
    // Lógica siguiente


}