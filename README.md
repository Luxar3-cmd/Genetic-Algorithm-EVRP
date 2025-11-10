# 🚗 Algoritmo Genético para Electric Vehicle Routing Problem (EVRP)

Solución al problema de enrutamiento de vehículos eléctricos utilizando un algoritmo genético con representación TSP-like y grafo preprocesado.

## 📋 Descripción del Proyecto

Este proyecto implementa un algoritmo genético para resolver el **Electric Vehicle Routing Problem (EVRP)**, una variante del problema clásico de enrutamiento de vehículos (VRP) que considera:

- **Vehículos eléctricos** con autonomía limitada (`B_max`)
- **Estaciones de recarga** distribuidas en el territorio
- **Restricción de capacidad** de los vehículos (`Q`)
- **Límite de flota** de vehículos disponibles (`B`)
- **Costos** por distancia recorrida y por recarga

### 🎯 Características Principales

- **Representación TSP-like**: Los cromosomas son permutaciones de clientes
- **Grafo preprocesado**: Las estaciones de recarga se manejan mediante preprocesamiento
- **Función Split**: Particiona permutaciones en rutas óptimas usando programación dinámica
- **Búsqueda Local**: Mutación con tres tipos de movimientos (2-opt, relocate, swap)
- **Binary Tournament Selection**: Selección de padres mediante torneos binarios
- **Order Crossover (OX)**: Cruce que preserva orden relativo

## 🏗️ Arquitectura

### Fase de Preprocesamiento

El algoritmo preprocesa el grafo completo para "ocultar" las estaciones de recarga:

1. **FASE 0**: Calcula matriz de distancias euclidianas
2. **FASE 1**: Clasifica nodos (Depósito, Clientes, Estaciones)
3. **FASE 2**: Construye grafo extendido con restricción de autonomía
4. **FASE 3**: Calcula caminos óptimos entre nodos obligatorios (Depósito + Clientes)

**Resultado**: Matrices `W`, `Rcnt` y `PathUV` que permiten usar la función `split` estándar de VRP.

### Algoritmo Genético

1. **Inicialización**: Genera población de permutaciones únicas de clientes
2. **Evaluación**: Aplica `split` a cada cromosoma para calcular fitness
3. **Selección**: Binary Tournament Selection
4. **Cruce**: Order Crossover (OX)
5. **Mutación**: Búsqueda local con movimientos aleatorios
6. **Terminación**: Después de `generations` iteraciones

## 📦 Requisitos

- **Compilador**: g++ con soporte para C++17
- **Sistema Operativo**: Linux (probado en Fedora)
- **Librerías**: Standard C++ Library (sin dependencias externas)

## 🚀 Compilación

### Usando Makefile (Recomendado)

```bash
# Compilar el proyecto
make

# Limpiar archivos compilados
make clean

# Recompilar desde cero
make rebuild

# Ejecutar con instancia por defecto
make run
```

### Compilación Manual

```bash
g++ -std=c++17 -Wall -Wextra -O2 -Wno-sign-compare -o evrp_solver main.cpp GA.cpp -lm
```

## 💻 Uso

### Ejecución Básica

```bash
./bin/evrp_solver <archivo_instancia>
```

### Ejemplo

```bash
./bin/evrp_solver instancias/instancia4.txt
```

### Formato de Entrada

El archivo de instancia debe seguir el siguiente formato:

```
B Q B_max C_km C_rec P S
d1 d2 ... dP
N
x0 y0
x1 y1
...
x_{N-1} y_{N-1}
```

Donde:
- `B`: Número de vehículos disponibles
- `Q`: Capacidad de carga de cada vehículo
- `B_max`: Autonomía máxima (distancia con batería completa)
- `C_km`: Costo por kilómetro recorrido
- `C_rec`: Costo fijo por recarga
- `P`: Número de clientes
- `S`: Número de estaciones de recarga
- `d1, d2, ..., dP`: Demandas de los clientes
- `N`: Total de nodos (1 + P + S)
- `xi yi`: Coordenadas del nodo i

**Orden de nodos:**
- Nodo 0: Depósito
- Nodos 1..P: Clientes
- Nodos P+1..N-1: Estaciones de recarga

### Formato de Salida

El programa imprime la solución en el siguiente formato:

```
83.2

Vehiculo 1:
Ruta: D -> C1 -> R1 -> C2 -> D
Distancia: 30
Recargas: 1

Vehiculo 2:
Ruta: D -> C3 -> R1 -> C4 -> D
Distancia: 41
Recargas: 1

Costo total: 83

Donde:
■ D indica el depósito principal (inicio y fin de la ruta).
■ Ci representa un cliente atendido.
■ Rj representa una estación de recarga visitada.
■ Distancia corresponde a la distancia total recorrida por el vehículo.
■ Recargas indica cuántas veces el vehículo realizó una recarga.
■ Costo total es el valor global de la solución considerando los costos por distancia y por recarga.

=== TIEMPOS DE EJECUCIÓN ===
Preprocesamiento: 0.000027 segundos (27 μs)
Algoritmo Genético: 0.009008 segundos (9.008000 ms)
Tiempo total: 0.009035 segundos (9.035000 ms)

=== PARÁMETROS DE EJECUCIÓN ===
Población: 24
Generaciones: 100
Tasa de cruce: 0.800000
Tasa de mutación: 0.300000
Iteraciones búsqueda local: 20

=== ESTADÍSTICAS DE RENDIMIENTO ===
Total de evaluaciones: 2424
Tiempo por evaluación: 3.716e-06 segundos
Evaluaciones por segundo: 269094.139
```

## ⚙️ Configuración del Algoritmo

Los parámetros del algoritmo genético se configuran en `main.cpp`:

```cpp
EVRP.initialize_parameters(
    pop_size,              // Tamaño de la población (default: 50)
    generations,           // Número de generaciones (default: 100)
    crossover_rate,        // Tasa de cruce [0,1] (default: 0.8)
    mutation_rate,         // Tasa de mutación [0,1] (default: 0.3)
    local_search_iterations // Iteraciones de búsqueda local (K_max) (default: 20)
);
```

### Parámetros Recomendados

- **Población**: 30-100 individuos
- **Generaciones**: 50-200
- **Tasa de cruce**: 0.7-0.9
- **Tasa de mutación**: 0.1-0.5
- **Iteraciones búsqueda local**: 10-50

## 📁 Estructura del Proyecto

```
codebase/
├── GA.h                 # Header principal con clase evolutionaryAlgo
├── GA.cpp               # Implementación del algoritmo genético
├── main.cpp             # Programa principal
├── Makefile             # Sistema de compilación
├── README.md            # Este archivo
├── instancias/          # Directorio con instancias del problema
│   ├── instancia1.txt
│   ├── instancia2.txt
│   ├── instancia3.txt
│   ├── instancia4.txt
│   ├── instancia5.txt
│   ├── instancia6.txt
│   ├── instancia7.txt
│   ├── instancia8.txt
│   ├── instancia9.txt
│   └── instancia40.txt  # Instancia grande (30 clientes)
└── instancias.zip       # Backup de instancias
```

## 🔬 Algoritmos Implementados

### 1. Preprocesamiento del Grafo

- **Dijkstra Lexicográfico**: Minimiza distancia, luego recargas
- **Grafo Contraído**: Reduce de N nodos a M = P+1 nodos
- **Matrices W, Rcnt, PathUV**: Almacenan caminos óptimos preprocesados

### 2. Función Split

- **Programación Dinámica**: Particiona permutaciones en rutas óptimas
- **Restricciones**: Capacidad (Q) y flota (B)
- **Complejidad**: O(P²) donde P es el número de clientes

### 3. Operadores Genéticos

- **Inicialización**: Permutaciones únicas de clientes
- **Selección**: Binary Tournament Selection
- **Cruce**: Order Crossover (OX)
- **Mutación**: Búsqueda local con 2-opt, relocate, swap

## 🧪 Ejemplos de Ejecución

```bash
# Compilar
make

# Ejecutar con instancia pequeña
./bin/evrp_solver instancias/instancia2.txt

# Ejecutar con instancia mediana
./bin/evrp_solver instancias/instancia4.txt

# Ejecutar con instancia grande
./bin/evrp_solver instancias/instancia40.txt
```

### Salida de Tiempos y Estadísticas

El programa incluye automáticamente medición de tiempos de ejecución:
- **Preprocesamiento**: Tiempo para cargar y preprocesar el grafo
- **Algoritmo Genético**: Tiempo de ejecución del GA completo
- **Tiempo total**: Tiempo completo desde inicio hasta fin
- **Parámetros de ejecución**: Población, generaciones, tasas, etc.
- **Estadísticas de rendimiento**: Total de evaluaciones, tiempo por evaluación y evaluaciones por segundo

Los tiempos se muestran en segundos con precisión de microsegundos (μs) para mediciones precisas.

## 📊 Resultados

El algoritmo encuentra soluciones factibles que:
- Respeta la capacidad de los vehículos (Q)
- Respeta el límite de flota (B)
- Considera la autonomía de los vehículos (B_max)
- Minimiza el costo total (distancia + recargas)

## 🐛 Solución de Problemas

### Error: "Cliente inalcanzable con B_max y estaciones dadas"

- **Causa**: La autonomía `B_max` es insuficiente para llegar a algún cliente
- **Solución**: Verificar que `B_max` sea suficientemente grande o que haya estaciones adecuadas

### Error: "Demanda del cliente X excede capacidad del vehículo"

- **Causa**: Algún cliente tiene demanda mayor que Q
- **Solución**: Verificar que todas las demandas sean ≤ Q

### Advertencia: "El tamaño de la población excede el número máximo de permutaciones"

- **Causa**: Se solicita más individuos que permutaciones posibles (P!)
- **Solución**: El algoritmo ajusta automáticamente el tamaño de población

## 📚 Referencias

- **VRP con Split**: Prins, C. (2004). "A simple and effective evolutionary algorithm for the vehicle routing problem"
- **EVRP**: Variante del VRP que considera vehículos eléctricos y estaciones de recarga
- **Order Crossover**: Goldberg, D.E. (1989). "Genetic Algorithms in Search, Optimization, and Machine Learning"

## 📝 Licencia

Este proyecto es parte de una tarea académica.

## 👤 Autor

Implementado como parte del curso de Inteligencia Artificial.


