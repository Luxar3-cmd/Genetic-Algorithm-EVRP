# Makefile para EVRP Solver - Versión Modular con Batería
# Electric Vehicle Routing Problem con batería residual

# Compilador y flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Wno-sign-compare
LDFLAGS = -lm

# Directorios
SRC_DIR = .
OBJ_DIR = obj
BIN_DIR = bin

# Módulos del sistema modular
MODULES = Preprocess Solution Split Population Operators GA
MODULE_OBJECTS = $(MODULES:%=$(OBJ_DIR)/%.o)

# Ejecutable principal
EVRP_SOLVER = $(BIN_DIR)/evrp_solver

# Regla por defecto
all: $(EVRP_SOLVER)

# Crear directorios
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Compilar archivos objeto
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ejecutable principal
$(EVRP_SOLVER): $(MODULE_OBJECTS) main.cpp | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(MODULE_OBJECTS) main.cpp -o $(EVRP_SOLVER) $(LDFLAGS)

# Limpiar
clean:
	rm -f $(OBJ_DIR)/*.o $(EVRP_SOLVER)

# Limpiar todo
clean-all:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Recompilar
rebuild: clean all

# Ejecutar con instancia por defecto
run: $(EVRP_SOLVER)
	./$(EVRP_SOLVER) instancias/instancia1.txt

# Ejecutar con discretización gruesa (más rápido)
run-fast: $(EVRP_SOLVER)
	./$(EVRP_SOLVER) instancias/instancia1.txt 10

# Ejecutar con población y generaciones custom
run-long: $(EVRP_SOLVER)
	./$(EVRP_SOLVER) instancias/instancia1.txt 1 100 200

# Ayuda
help:
	@echo "Makefile - EVRP Solver Modular"
	@echo ""
	@echo "Arquitectura modular:"
	@echo "  - Preprocess: Preprocesamiento con W[i][j][b]"
	@echo "  - Solution: Estructuras de datos"
	@echo "  - Split: Decodificador con DP multidimensional"
	@echo "  - Population: Manejo de población"
	@echo "  - Operators: Operadores genéticos (búsqueda local)"
	@echo "  - GA: Coordinador del algoritmo"
	@echo ""
	@echo "Comandos:"
	@echo "  make              - Compila todo"
	@echo "  make run          - Ejecuta con parámetros default"
	@echo "  make run-fast     - Ejecuta con discretización=10"
	@echo "  make run-long     - Ejecuta 100 pop, 200 gen"
	@echo "  make clean        - Limpia compilados"
	@echo "  make rebuild      - Recompila todo"
	@echo ""
	@echo "Uso manual:"
	@echo "  ./bin/evrp_solver <instancia> [battery_step] [pop_size] [generations]"
	@echo ""
	@echo "Ejemplos:"
	@echo "  ./bin/evrp_solver instancias/instancia1.txt"
	@echo "  ./bin/evrp_solver instancias/instancia1.txt 1 50 100"
	@echo "  ./bin/evrp_solver instancias/instancia1.txt 0.1 30 50"

.PHONY: all clean clean-all rebuild run run-fast run-long help
