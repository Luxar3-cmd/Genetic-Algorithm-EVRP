# EVRP Solver - Algoritmo Genético

Sistema de resolución del Electric Vehicle Routing Problem (EVRP) mediante algoritmo genético con restricciones de capacidad y batería.

---

## 1. Introducción al Problema

### ¿Qué es EVRP?

El **Electric Vehicle Routing Problem** es una extensión del VRP clásico que considera vehículos eléctricos con batería limitada. El objetivo es diseñar rutas que minimicen el costo total mientras:

- Todos los clientes son visitados exactamente una vez
- La capacidad de los vehículos no se excede
- La batería es suficiente para completar las rutas
- Se pueden usar estaciones de recarga cuando sea necesario

### Características del Problema

- **Clientes**: Nodos con demanda que deben ser visitados
- **Depósito**: Punto de inicio y fin de rutas (batería siempre full)
- **Estaciones**: Puntos opcionales para recargar batería (con costo)
- **Capacidad Q**: Carga máxima por vehículo
- **Batería B_max**: Energía máxima del vehículo
- **Flota B**: Número máximo de vehículos disponibles

### Costos

- **C_km**: Costo por kilómetro recorrido
- **C_rec**: Costo fijo de recarga en estación

**Costo total** = distancia recorrida × C_km + recargas × C_rec

### Formato de Instancia

El solver espera archivos de instancia con el siguiente formato estricto:

```
B Q Bmax
Ckm Crec
P
S
q1 q2 ... qP  (demandas de clientes)
N             (Total nodos = 1 + P + S)
x1 y1
x2 y2
...
xN yN
```

---

## 2. Representación de Individuos

### Cromosoma

Cada individuo del GA es una **permutación de clientes**:

```
Cromosoma = [3, 1, 4, 2, 5]
```

**Características**:
- Solo contiene IDs de clientes (1 a P)
- NO incluye depósito ni estaciones
- El orden determina la secuencia de visita
- Longitud = número de clientes P

**Ejemplo**:
- Cliente 1, 2, 3, 4, 5
- Cromosoma [3, 1, 4, 2, 5] significa: visitar 3 → 1 → 4 → 2 → 5

### Decodificación: Split Algorithm Mejorado

El cromosoma no especifica rutas directamente. Para convertirlo en rutas factibles usamos **Split con DP multidimensional** y una estrategia de búsqueda inteligente.

#### Matrices Preprocesadas "Safe"

Para manejar situaciones donde el camino más corto agota la batería, precalculamos dos tipos de matrices:
1.  **Min Cost (`W`)**: Camino de costo mínimo (puede gastar mucha batería).
2.  **Max Battery (`W_safe`)**: Camino que maximiza la batería residual al llegar (puede ser más costoso).

#### Proceso de Búsqueda en Split

La función `calculate_route_cost` utiliza una **Priority Queue** para explorar combinaciones de caminos Min Cost y Max Battery. Esto permite al algoritmo:
- Elegir caminos rápidos cuando hay batería suficiente.
- Desviarse automáticamente a estaciones (vía caminos "Safe") cuando la batería es crítica.
- Garantizar factibilidad incluso en rutas complejas con múltiples recargas.

#### Concepto Clave: V[i][b]

`V[i][b]` = costo mínimo para atender clientes 0..i-1, terminando con batería `b` al regresar al depósito.

#### Detalles Técnicos

- Usa matrices preprocesadas `W[i][j][b]` y `W_safe[i][j][b]`
- Considera automáticamente estaciones de recarga
- Garantiza partición óptima del cromosoma
- Complejidad: O(P² × B_levels)

---

## 3. Inicialización de Población

### Generación Aleatoria

```cpp
Para i = 1 hasta pop_size:
    chromosome = permutación aleatoria de [1..P]
    solution = split.decode(chromosome)
    population[i] = chromosome
    fitness[i] = evaluar(solution)
```

**Parámetros**:
- `pop_size`: 50 (default)
- Cada individuo es evaluado inmediatamente

### Manejo de Individuos Infactibles

**¿Qué pasa si TODOS los individuos son infactibles?**

Gracias a la mejora en Split con "Safe Edges", la probabilidad de encontrar soluciones factibles ha aumentado drásticamente. Sin embargo, si aún aparecen infactibles:

**Comportamiento actual**:
- GA evoluciona individuos infactibles (fitness = 1e9)
- Busca minimizar costo entre infactibles
- Si aparece factible por cruce/mutación, domina inmediatamente
- Sistema reporta el mejor (aunque sea infactible)

### Evaluación de Fitness

```cpp
if (solution.is_feasible) {
    fitness = solution.total_cost;  // Minimizar costo
} else {
    fitness = 1e9;  // Penalización alta
}
```

---

## 4. Operadores Genéticos

### Selección: Binary Tournament Selection

**Proceso**:
```cpp
1. Seleccionar 2 individuos aleatorios
2. Comparar sus fitness
3. Retornar el de mejor (menor) fitness
```

**Características**:
- Torneo binario (tamaño fijo = 2)
- Balance entre presión selectiva y diversidad
- Simple y eficiente

### Cruce: Order Crossover (OX)

**Objetivo**: Preservar orden relativo de los padres

**Proceso**:
```
Parent1: [3, 1, 4, 2, 5]
Parent2: [2, 5, 1, 3, 4]

1. Seleccionar puntos de corte aleatorios: [2, 4]
2. Copiar segmento de Parent1:
   Child1: [_, _, 4, 2, 5]

3. Llenar con orden de Parent2:
   Secuencia Parent2: 2,5,1,3,4
   Eliminar {4,2,5}: quedan 1,3
   Child1: [1, 3, 4, 2, 5]
```

**Características**:
- Garantiza permutación válida
- Hereda bloques de ambos padres
- Probabilidad de aplicar: 0.8

### Mutación: Búsqueda Local

**IMPORTANTE**: NO es mutación aleatoria simple, es **búsqueda local inteligente**.

**3 Movimientos de Vecindario**:

1. **Swap**: Intercambiar 2 clientes
2. **2-opt**: Invertir un segmento
3. **Reassign**: Mover un cliente a otra posición

**Algoritmo**:
```
iterations_sin_mejora = 0
mejor = cromosoma_actual

while iterations_sin_mejora < max_iterations:
    vecino = aplicar_movimiento_aleatorio(mejor)
    solution_vecino = split.decode(vecino)
    
    if solution_vecino.cost < mejor.cost:
        mejor = vecino
        iterations_sin_mejora = 0
    else:
        iterations_sin_mejora++

return mejor
```

**Parámetros**:
- `max_iterations`: 10 (sin mejora)
- Probability: 0.2
- Evaluación completa con Split para cada vecino

---

## 5. Evolución

### Loop Principal

```
Inicializar población aleatoria
Evaluar todos los individuos

Para gen = 1 hasta generations:
    // Selección
    padre1 = binary_tournament(población)
    padre2 = binary_tournament(población)
    
    // Cruce (80% probabilidad)
    if random() < 0.8:
        (hijo1, hijo2) = order_crossover(padre1, padre2)
    else:
        hijo1, hijo2 = padre1, padre2
    
    // Mutación/Búsqueda Local (20% probabilidad)
    if random() < 0.2:
        hijo1 = local_search(hijo1)
    if random() < 0.2:
        hijo2 = local_search(hijo2)
    
    // Evaluación
    sol1 = split.decode(hijo1)
    sol2 = split.decode(hijo2)
    
    // Reemplazo
    if sol1 mejor que peor_individuo:
        reemplazar_peor(hijo1, sol1)
    if sol2 mejor que peor_individuo:
        reemplazar_peor(hijo2, sol2)
    
    // Reportar progreso
    if gen % 10 == 0:
        print("Gen", gen, "| Mejor:", mejor_costo)

Retornar mejor solución encontrada
```

### Estrategia de Reemplazo

- **Tipo**: Reemplazo del peor (Steady-State)
- **Criterio**: Solo si hijo tiene mejor fitness que el peor actual
- **Elitismo**: Implícito (el mejor nunca se pierde)

---

## 6. Parámetros del Sistema

### Parámetros del Problema (Instancia)

| Parámetro | Descripción |
|-----------|-------------|
| `P` | Número de clientes |
| `S` | Número de estaciones |
| `B` | Flota máxima (vehículos) |
| `Q` | Capacidad por vehículo |
| `B_max` | Batería máxima |
| `C_km` | Costo por km |
| `C_rec` | Costo de recarga |

### Parámetros del Algoritmo Genético

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `pop_size` | 50 | Tamaño de población |
| `generations` | 100 | Número de generaciones |
| `crossover_rate` | 0.8 | Probabilidad de cruce |
| `mutation_rate` | 0.2 | Probabilidad de mutación |
| `tournament_size` | 2 | Fijo (torneo binario) |
| `local_search_iters` | 10 | Iteraciones sin mejora |

### Parámetros de Discretización

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `battery_step` | 1 | Granularidad de batería |
| `B_levels` | (B_max/battery_step)+1 | Niveles discretos |

**Trade-off**:
- `battery_step` pequeño → más preciso, más lento, más memoria
- `battery_step` grande → menos preciso, más rápido, menos memoria

**Ejemplos**:
- `battery_step=0.1`: Ultra preciso, B_levels=2001 (B_max=200)
- `battery_step=1`: Balance, B_levels=201
- `battery_step=10`: Rápido, B_levels=21

⚠️ **Límite**: B_levels muy grande (>10,000) puede causar out of memory

---

## 7. Preprocesamiento

### Matriz W[i][j][b]

**Definición**: `W[i][j][b]` = costo mínimo para ir de nodo i a nodo j comenzando con batería b

**Características**:
- Calculada una sola vez al inicio con Dijkstra
- Considera automáticamente estaciones de recarga óptimas
- Dimensiones: M × M × B_levels
  - M = P + S + 1 (clientes + estaciones + depósito)
- Reutilizada millones de veces durante la evolución

### Matrices "Safe" (NUEVO)

- **W_safe[i][j][b]**: Costo del camino que maximiza la batería residual al llegar a j.
- **FinalBattery_safe[i][j][b]**: Batería residual correspondiente.

Estas matrices permiten al algoritmo encontrar soluciones factibles incluso cuando el camino más corto es inviable por batería.

### Tiempo de Preprocesamiento

Depende de M y B_levels:
- Instancia pequeña: ~30 ms
- Instancia mediana: ~100 ms
- Instancia grande con battery_step pequeño: varios segundos

---

## 8. Salida y Reportes

### Durante Ejecución

```
Gen  10 | Mejor: 58.68
Gen  20 | Mejor: 58.68
Gen  30 | Mejor: 56.32  ← Mejora encontrada
```

### Reporte Final

```
╔═══════════════════════════════════════════════════════════╗
║                    REPORTE FINAL                          ║
╚═══════════════════════════════════════════════════════════╝

Solución: FACTIBLE

Costo total:          58.68
Vehículos usados:     2
Número de rutas:      2 (Total de viajes/trips realizados)

Detalle de rutas:
  Vehiculo 1:
  Ruta: D -> C1 -> C2 -> D
  Distancia: 28.94
  Recargas: 0

  Vehiculo 2:
  Ruta: D -> C3 -> D
  Distancia: 29.73
  Recargas: 0

Tiempo de ejecución:
  Preprocesamiento:   29 ms
  Algoritmo genético: 1 ms
  TIEMPO TOTAL:       30 ms
                      (0.03 segundos)
```

---

## 9. Herramientas de Validación

El proyecto incluye herramientas para asegurar la correctitud de las soluciones:

### Auditoría de Instancias (`audit_instances.cpp`)
Verifica si una instancia es teóricamente factible analizando la conectividad y alcanzabilidad de todos los clientes desde el depósito.
```bash
make audit
./bin/audit_instances
```

### Validador Independiente (`validator.py`)
Script en Python que recalcula la factibilidad de la solución final desde cero, verificando restricciones de capacidad y visitas.
```bash
python3 validator.py <instancia> <archivo_salida>
```

---

## 10. Uso

### Compilación

```bash
make
```

### Ejecución

```bash
./bin/evrp_solver <instancia> [battery_step] [pop_size] [generations]
```

### Parámetros

- `instancia`: Archivo .txt con datos (requerido)
- `battery_step`: Discretización (default: 1)
- `pop_size`: Tamaño población (default: 50)
- `generations`: Número de generaciones (default: 100)

### Ejemplos

**Configuración default**:
```bash
./bin/evrp_solver instancias/instancia1.txt
```

**Custom**:
```bash
./bin/evrp_solver instancias/instancia5.txt 1 50 100
```

### Comandos Make

```bash
make              # Compilar
make run          # Ejecutar con defaults
make run-fast     # Ejecutar con battery_step=10
make run-long     # 100 población, 200 generaciones
make clean        # Limpiar compilados
make rebuild      # Recompilar todo
make help         # Ver ayuda
```

---

## 11. Arquitectura y Componentes

El sistema está diseñado con una arquitectura modular donde cada componente tiene una responsabilidad única y clara.

### 1. Preprocess (`Preprocess.hpp/cpp`)
**Responsabilidad**: Preparar el grafo y calcular matrices de costos para acelerar la evaluación.
- **Grafo Extendido**: Construye un grafo donde los nodos son Clientes, Estaciones y Depósito.
- **Matrices 3D**: Calcula `W[i][j][b]`, el costo mínimo para ir del nodo `i` al `j` comenzando con batería `b`.
- **Estrategia "Safe"**: Adicionalmente calcula `W_safe`, que prioriza maximizar la batería residual para garantizar factibilidad en tramos difíciles.
- **Eficiencia**: Se ejecuta una sola vez al inicio. Todas las demás clases acceden a él por referencia constante (`const &`), evitando copias costosas.

### 2. Split (`Split.hpp/cpp`)
**Responsabilidad**: Decodificar un cromosoma (permutación de clientes) en una solución completa (rutas con vehículos).
- **Algoritmo Split Adaptado**: Utiliza Programación Dinámica (DP) para partir la secuencia de clientes en rutas óptimas.
- **DP Multidimensional**: `V[i][b]` almacena el costo mínimo para atender los primeros `i` clientes terminando con batería `b`. Esto es crucial para el EVRP, ya que el estado de carga al volver al depósito afecta la factibilidad de la siguiente ruta (en modelos multi-trip) o simplemente la validación.
- **Búsqueda de Rutas**: La función `calculate_route_cost` usa A* / Dijkstra local para encontrar el camino óptimo entre dos clientes, insertando estaciones de recarga si es necesario usando las matrices precalculadas.

### 3. Population (`Population.hpp/cpp`)
**Responsabilidad**: Gestionar el conjunto de individuos (soluciones) actuales.
- **Inicialización**: Genera individuos aleatorios al inicio.
- **Evaluación**: Usa `Split` para calcular el fitness (costo) de cada cromosoma.
- **Reemplazo**: Implementa la lógica de reemplazo "Steady-State" (reemplazar al peor si el hijo es mejor).

### 4. Operators (`Operators.hpp/cpp`)
**Responsabilidad**: Implementar los operadores genéticos de variación.
- **Selección**: Torneo Binario (selecciona el mejor de 2 al azar).
- **Crossover**: Order Crossover (OX), que preserva la secuencia relativa de los padres, vital para problemas de permutación como VRP.
- **Mutación (Local Search)**: En lugar de una mutación aleatoria simple, aplica una búsqueda local agresiva (Swap, 2-opt, Reassign) para mejorar la calidad de los hijos.

### 5. GA (`GA.hpp/cpp`)
**Responsabilidad**: Orquestar el flujo principal del algoritmo.
- **Bucle Evolutivo**: Controla las generaciones, probabilidades de cruce/mutación y criterios de parada.
- **Reporting**: Muestra el progreso y la mejor solución encontrada.

### 6. Solution (`Solution.hpp/cpp`)
**Responsabilidad**: Definir las estructuras de datos.
- **Route**: Almacena clientes, costo, carga y recargas de una ruta.
- **Solution**: Agrupa múltiples rutas y métricas globales (costo total, factibilidad).

### Diagrama de Flujo de Datos

```
GA (Main Loop)
 │
 ├──> Population (Gestión)
 │     │
 │     └──> Split (Decodificación)
 │           │
 │           └──> Preprocess (Datos precalculados W/Rcnt)
 │
 └──> Operators (Variación)
       │
       └──> Split (Evaluación de hijos)
```

---

## 12. Limitaciones Conocidas

1. **Instancias infactibles**: El sistema reporta infactibilidad pero evoluciona de todas formas
2. **Discretización decimal fina**: `battery_step < 0.1` puede causar out of memory
3. **Convergencia prematura**: Posible con instancias muy pequeñas
4. **Sin garantía de optimalidad**: Es metaheurística, no método exacto

---

## 13. Referencias

- Prins, C. (2004). "A simple and effective evolutionary algorithm for the vehicle routing problem"
- Schneider, M., et al. (2014). "The Electric Vehicle-Routing Problem with Time Windows and Recharging Stations"
