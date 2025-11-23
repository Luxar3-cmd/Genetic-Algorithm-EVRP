#include "Preprocess.hpp"
#include <iostream>

using namespace std;

int main() {
  try {
    cout << "=== Debug Instancia 7 ===" << endl;

    Preprocess preprocess("instancias/instancia7.txt");

    preprocess.print_node_info();
    preprocess.print_W_matrix();
    preprocess.print_Rcnt_matrix();
    preprocess.print_paths();

    return 0;
  } catch (const exception &e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
  }
}
