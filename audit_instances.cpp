#include "Preprocess.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

void audit_instance(const string &filename) {
  cout << "\n╔═══════════════════════════════════════════════════════════╗"
       << endl;
  cout << "║  AUDITORÍA DE INSTANCIA: " << setw(30) << left << filename
       << "         ║" << endl;
  cout << "╚═══════════════════════════════════════════════════════════╝\n"
       << endl;

  try {
    Preprocess preprocess(filename);

    cout << "=== PARÁMETROS ===" << endl;
    cout << "Clientes (P): " << preprocess.get_P() << endl;
    cout << "Estaciones (S): " << preprocess.get_S() << endl;
    cout << "Vehículos (B): " << preprocess.get_B() << endl;
    cout << "Capacidad (Q): " << preprocess.get_Q() << endl;
    cout << "Autonomía (B_max): " << preprocess.get_B_max() << endl;
    cout << "Costo/km (C_km): " << preprocess.get_C_km() << endl;
    cout << "Costo recarga (C_rec): " << preprocess.get_C_rec() << endl;

    // Verificar alcanzabilidad de clientes desde depósito
    cout << "\n=== ALCANZABILIDAD DESDE DEPÓSITO (con batería llena) ==="
         << endl;
    int b_full = preprocess.get_B_levels() - 1;
    const auto &W = preprocess.get_W();
    const auto &Rcnt = preprocess.get_Rcnt();
    int unreachable_count = 0;

    for (int i = 1; i <= preprocess.get_P(); i++) {
      int u_idx = preprocess.node_to_U_index(i);
      double cost = W[0][u_idx][b_full];
      int recharges = Rcnt[0][u_idx][b_full];

      if (!isfinite(cost)) {
        cout << "❌ Cliente " << i << ": INALCANZABLE" << endl;
        unreachable_count++;
      } else {
        cout << "✓ Cliente " << i << ": alcanzable (costo=" << fixed
             << setprecision(2) << cost << ", recargas=" << recharges << ")"
             << endl;
      }
    }

    // Verificar retorno al depósito
    cout << "\n=== RETORNO AL DEPÓSITO (desde cada cliente) ===" << endl;
    int no_return_count = 0;

    for (int i = 1; i <= preprocess.get_P(); i++) {
      int u_idx = preprocess.node_to_U_index(i);
      double cost = W[u_idx][0][b_full];
      int recharges = Rcnt[u_idx][0][b_full];

      if (!isfinite(cost)) {
        cout << "❌ Cliente " << i << " → Depósito: IMPOSIBLE" << endl;
        no_return_count++;
      } else {
        cout << "✓ Cliente " << i << " → Depósito: posible (costo=" << fixed
             << setprecision(2) << cost << ", recargas=" << recharges << ")"
             << endl;
      }
    }

    // Verificar conectividad entre clientes
    cout << "\n=== CONECTIVIDAD ENTRE CLIENTES ===" << endl;
    int disconnected_pairs = 0;

    for (int i = 1; i <= preprocess.get_P(); i++) {
      for (int j = 1; j <= preprocess.get_P(); j++) {
        if (i == j)
          continue;

        int u_i = preprocess.node_to_U_index(i);
        int u_j = preprocess.node_to_U_index(j);
        double cost = W[u_i][u_j][b_full];

        if (!isfinite(cost)) {
          cout << "❌ Cliente " << i << " → Cliente " << j << ": desconectados"
               << endl;
          disconnected_pairs++;
        }
      }
    }

    if (disconnected_pairs == 0) {
      cout << "✓ Todos los pares de clientes están conectados" << endl;
    } else {
      cout << "⚠️  " << disconnected_pairs << " pares de clientes desconectados"
           << endl;
    }

    // Diagnóstico final
    cout << "\n=== DIAGNÓSTICO ===" << endl;
    if (unreachable_count > 0) {
      cout << "❌ INSTANCIA INFACTIBLE: " << unreachable_count
           << " cliente(s) inalcanzable(s) desde depósito" << endl;
    } else if (no_return_count > 0) {
      cout << "❌ INSTANCIA INFACTIBLE: " << no_return_count
           << " cliente(s) no pueden regresar al depósito" << endl;
    } else if (disconnected_pairs > 0) {
      cout << "⚠️  POSIBLEMENTE INFACTIBLE: Algunos clientes no están "
              "conectados entre sí"
           << endl;
      cout << "    (Podría ser factible con rutas individuales)" << endl;
    } else {
      cout << "✅ INSTANCIA PARECE FACTIBLE: Todos los clientes son alcanzables"
           << endl;
      cout << "    Si el GA no encuentra solución, podría ser por:" << endl;
      cout << "    - Restricción de capacidad Q muy baja" << endl;
      cout << "    - Flota B insuficiente" << endl;
      cout << "    - Necesidad de múltiples recargas por ruta" << endl;
    }

  } catch (const exception &e) {
    cout << "ERROR: " << e.what() << endl;
  }
}

int main(int argc, char **argv) {
  vector<string> instances;

  if (argc > 1) {
    // Usar instancias especificadas
    for (int i = 1; i < argc; i++) {
      instances.push_back(argv[i]);
    }
  } else {
    // Auditar instancias problemáticas
    instances = {"instancias/instancia4.txt", "instancias/instancia6.txt",
                 "instancias/instancia7.txt", "instancias/instancia8.txt",
                 "instancias/instancia9.txt"};
  }

  for (const auto &inst : instances) {
    audit_instance(inst);
  }

  return 0;
}
