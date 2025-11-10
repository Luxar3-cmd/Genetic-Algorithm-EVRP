# Makefile para Algoritmo Genético EVRP
# Electric Vehicle Routing Problem con representación TSP-like

# Compilador y flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Wno-sign-compare
LDFLAGS = -lm

# Directorios
SRC_DIR = .
OBJ_DIR = obj
BIN_DIR = bin

# Archivos fuente
SOURCES = GA.cpp main.cpp
OBJECTS = $(SOURCES:%.cpp=$(OBJ_DIR)/%.o)
TARGET = $(BIN_DIR)/evrp_solver

# Programa para mostrar matrices
SHOW_MATRICES_SRC = show_matrices.cpp
SHOW_MATRICES_OBJ = $(OBJ_DIR)/show_matrices.o
SHOW_MATRICES_TARGET = $(BIN_DIR)/show_matrices

# Regla por defecto
all: $(TARGET) $(SHOW_MATRICES_TARGET)

# Crear directorios si no existen
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Compilar archivos objeto
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Enlazar ejecutable
$(TARGET): $(OBJECTS) | $(BIN_DIR)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compilar objeto para show_matrices
$(SHOW_MATRICES_OBJ): $(SHOW_MATRICES_SRC) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Enlazar show_matrices (necesita GA.o)
$(SHOW_MATRICES_TARGET): $(SHOW_MATRICES_OBJ) $(OBJ_DIR)/GA.o | $(BIN_DIR)
	$(CXX) $(SHOW_MATRICES_OBJ) $(OBJ_DIR)/GA.o -o $(SHOW_MATRICES_TARGET) $(LDFLAGS)

# Limpiar archivos compilados
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Limpiar y recompilar
rebuild: clean all

# Ejecutar con instancia por defecto
run: $(TARGET)
	./$(TARGET) instancias/instancia1.txt

# Ayuda
help:
	@echo "Makefile para Algoritmo Genético EVRP"
	@echo ""
	@echo "Comandos disponibles:"
	@echo "  make          - Compila el proyecto"
	@echo "  make clean    - Limpia archivos compilados"
	@echo "  make rebuild  - Limpia y recompila"
	@echo "  make run      - Ejecuta con instancia1.txt"
	@echo "  make help     - Muestra esta ayuda"
	@echo ""
	@echo "Ejecutar manualmente:"
	@echo "  ./bin/evrp_solver <archivo_instancia>"
	@echo "  ./bin/show_matrices <archivo_instancia>"

.PHONY: all clean rebuild run help

