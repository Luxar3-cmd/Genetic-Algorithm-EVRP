# Preprocesamiento del Grafo para EVRP con Representación TSP-like

## 📋 Resumen Ejecutivo

Este documento explica cómo el preprocesamiento transforma el problema EVRP (Electric Vehicle Routing Problem) en un problema VRP clásico, permitiendo usar la función `split` estándar sin modificaciones.

**Idea clave:** Colapsar el grafo completo (Depósito + Clientes + Estaciones) en un grafo contraído que solo incluye nodos obligatorios (Depósito + Clientes), donde los costos ya contemplan automáticamente las estaciones de recarga necesarias.

---

## 🎯 Objetivo

Generar tres matrices preprocesadas:

- **`W[i][j]`**: Costo mínimo para ir de U[i] a U[j] (incluye estaciones intermedias)
- **`Rcnt[i][j]`**: Número de recargas en ese camino
- **`PathUV[i][j]`**: Camino completo [U[i], ..., estaciones, ..., U[j]]

Donde `U = {Depósito} ∪ {Clientes}` y tiene tamaño M = P + 1.

---

## 🏗️ Arquitectura del Preprocesamiento

### FASE 0: Cálculo de Matriz de Distancias

**Función:** `matrix_distances()`

Calcula la distancia euclidiana entre todos los pares de nodos:

```
dist[i][j] = √[(x_j - x_i)² + (y_j - y_i)²]
```

**Entrada:** Coordenadas (x,y) de todos los N nodos  
**Salida:** Matriz simétrica N×N con distancias euclidianas

---

### FASE 1: Clasificación de Nodos

**Función:** `build_types()`

Clasifica cada nodo según su rol en el problema:

| Índice | Tipo | Descripción |
|--------|------|-------------|
| 0 | `Depot` | Depósito (inicio/fin de rutas) |
| 1..P | `Client` | Clientes (deben ser visitados) |
| P+1..N-1 | `Station` | Estaciones de recarga (opcionales) |

También construye el conjunto **U** de nodos obligatorios:
```
U = {0, 1, 2, ..., P}
|U| = M = P + 1
```

---

### FASE 2: Construcción del Grafo Extendido

**Función:** `build_graph()`

Construye un grafo dirigido considerando la **restricción de autonomía**:

#### Regla de Existencia de Aristas
```
Existe arista a → b  ⟺  dist[a][b] ≤ B_max
```

El vehículo solo puede moverse entre nodos alcanzables con una carga completa.

#### Cálculo de Peso de Arista
```cpp
w = C_km × dist[a][b] + (b es estación ? C_rec : 0)
```

**Componentes del peso:**
- `C_km × dist[a][b]`: Costo por kilometraje
- `+ C_rec`: Costo fijo de recarga (solo si el destino es estación)

#### Asunciones Importantes
1. **Al llegar a una estación, se recarga la batería al 100%**
2. El costo `C_rec` se cobra **al llegar** a la estación
3. La recarga es **completa** (no hay cargas parciales)

---

### FASE 3: Preprocesamiento de Caminos Óptimos

**Función:** `preprocess_paths()`

Esta es la **fase crítica** que colapsa el grafo completo en uno contraído.

#### Proceso

Para cada par de nodos (i, j) en U × U:

1. **Ejecutar Dijkstra lexicográfico** desde U[i]
2. **Extraer información** para U[j]:
   - Costo mínimo: `W[i][j] = distG[U[j]]`
   - Número de recargas: `Rcnt[i][j] = recG[U[j]]`
   - Camino completo: `PathUV[i*M + j] = reconstruct_path(...)`

#### Dijkstra Lexicográfico

**Función:** `dijkstra_lex(start_id, distG, recG, prev)`

Algoritmo de Dijkstra modificado con **orden de prioridad lexicográfico**:

1. **Criterio primario:** Minimizar distancia/costo total
2. **Criterio secundario:** Minimizar número de recargas (en caso de empate)

**Implementación:**
- Usa `priority_queue<State>` con operador `<` invertido (max-heap → min-heap)
- `State = {dist, rec, v}` representa un estado en la búsqueda
- Incrementa contador de recargas al llegar a estaciones: `new_rec = recG[u] + (isStation ? 1 : 0)`

**Salidas:**
- `distG[v]`: Costo mínimo desde start a v
- `recG[v]`: Número de recargas en ese camino
- `prev[v]`: Predecesor de v en el árbol de caminos mínimos

#### Reconstrucción de Camino

**Función:** `reconstruct_path(start_id, target_id, prev)`

Reconstruye el camino completo usando el vector de predecesores:

```
Input:  start = 1, target = 3, prev = [-1, 0, 5, 6, 0, 1, 5]
Output: [1, 5, 6, 3]  // Puede incluir estaciones intermedias
```

**Características:**
- Retorna vector con todos los nodos: `[start, ..., estaciones, ..., target]`
- Incluye estaciones de recarga intermedias si las hay
- Retorna vector vacío si no existe camino

#### Validación Final

Verifica que todos los clientes sean **alcanzables** desde el depósito (ida y vuelta):

```cpp
for (int i = 1; i < M; i++) {
    if (!isfinite(W[0][i]) || !isfinite(W[i][0])) {
        throw runtime_error("Cliente inalcanzable");
    }
}
```

---

## 📊 Ejemplo Ilustrativo

### Problema
- **Nodos:** D (depósito), C1, C2, C3 (clientes), E1 (estación)
- **B_max = 100 km**, **C_km = 1**, **C_rec = 50**
- **Distancias:** D-C1=80, C1-E1=60, E1-C2=50, C2-C3=90, C3-D=70

### Grafo Extendido (Fase 2)
```
Aristas creadas (solo si dist ≤ 100):
D → C1 (w=80)
C1 → E1 (w=60+50=110)  // +50 por llegar a estación
E1 → C2 (w=50)
C2 → C3 (w=90)
C3 → D (w=70)
```

### Grafo Contraído (Fase 3)
```
W[D][C2] = 190  // D → C1 → E1 → C2
Rcnt[D][C2] = 1  // Una recarga en E1
PathUV[D*M + C2] = [0, 1, 5, 2]  // Nodos D, C1, E1, C2
```

**El algoritmo genético solo verá:**
```
"Para ir de D a C2 cuesta 190 unidades"
(Sin saber que internamente pasa por E1)
```

---

## 🔗 Conexión con Split

Una vez preprocesado, el algoritmo genético funciona así:

### 1. Generar Permutación
```cpp
Cromosoma: [C2, C1, C3]  // Solo clientes, sin depósito ni estaciones
```

### 2. Aplicar Split
```cpp
split() usa la matriz W como si fuera un VRP clásico:
- Costo D → C2: W[0][2] = 190
- Costo C2 → C1: W[2][1] = ?
- Costo C1 → C3: W[1][3] = ?
- Costo C3 → D: W[3][0] = 70
```

### 3. Generar Solución Real
```cpp
Si split determina la ruta: D → C2 → C1 → C3 → D

El camino real se construye usando PathUV:
D → [PathUV[0][2]] → C1 → [PathUV[1][3]] → D
= D → C1 → E1 → C2 → C1 → E1 → C3 → D
```

**Ventaja:** Split no necesita saber nada sobre estaciones o batería. Todo está encapsulado en W.

---

## ⚙️ Complejidad Computacional

| Fase | Complejidad | Descripción |
|------|-------------|-------------|
| FASE 0 | O(N²) | Calcular distancias |
| FASE 1 | O(N) | Clasificar nodos |
| FASE 2 | O(N²) | Construir aristas |
| FASE 3 | O(M × (N log N + E)) | M dijkstras en grafo con E aristas |

**Total:** O(M × N log N) ≈ O(P × N log N)

**Nota:** El preprocesamiento se hace **una sola vez** al inicio. Durante la evolución del GA, solo se usan las matrices W, Rcnt y PathUV.

---

## 🎓 Ventajas de este Enfoque

1. **Separación de Preocupaciones:** Las estaciones y batería se manejan en preprocesamiento, no durante el GA
2. **Reutilización de Código:** Se puede usar split estándar sin modificaciones
3. **Eficiencia:** Cálculos pesados se hacen una vez, no en cada evaluación de fitness
4. **Flexibilidad:** Fácil cambiar algoritmo de búsqueda (Dijkstra → A*, etc.)
5. **Corrección:** Garantiza caminos factibles considerando restricción de autonomía

---

## 📚 Referencias Clave en el Código

| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `GA.h` | 45-53 | Clase principal y estructura de datos |
| `GA.h` | 105-135 | Declaraciones de funciones de preprocesamiento |
| `GA.cpp` | 212-248 | FASE 1: `build_types()` |
| `GA.cpp` | 250-287 | FASE 2: `build_graph()` |
| `GA.cpp` | 143-210 | FASE 2.5: `dijkstra_lex()` |
| `GA.cpp` | 76-113 | FASE 2.5: `reconstruct_path()` |
| `GA.cpp` | 12-74 | FASE 3: `preprocess_paths()` |
| `GA.cpp` | 289-344 | Constructor que orquesta todo |

---

## 🔧 Parámetros Importantes

- **`B_max`**: Autonomía máxima con batería completa (define conectividad del grafo)
- **`C_km`**: Costo por kilómetro (componente principal del costo)
- **`C_rec`**: Costo fijo de recarga (penalización por usar estaciones)
- **Orden lexicográfico**: Prioriza minimizar distancia, luego recargas

---

## 🚀 Próximos Pasos

Con el preprocesamiento completo, ahora puedes implementar:

1. **Función Split:** Usar W para calcular partición óptima de rutas
2. **Operadores Genéticos:** Crossover y mutación sobre permutaciones de clientes
3. **Evaluación de Fitness:** Usar split para obtener costo de cada cromosoma
4. **Reconstrucción de Solución:** Usar PathUV para obtener rutas detalladas con estaciones

