# 📚 Documentación Completa del Preprocesamiento EVRP

## ✅ Resumen de Trabajo Completado

He documentado completamente la fase de preprocesamiento de tu algoritmo genético para EVRP con representación TSP-like. Aquí está todo lo que se ha hecho:

---

## 📄 Archivos Modificados y Creados

### 1. **GA.h** - Header completamente documentado
   - ✅ Documentación Doxygen de todas las estructuras
   - ✅ Explicación detallada de cada variable miembro
   - ✅ Comentarios sobre el propósito de cada fase
   - ✅ Aclaración del flujo de preprocesamiento

### 2. **GA.cpp** - Implementación completamente documentada
   - ✅ **FASE 0:** `matrix_distances()` - Cálculo de distancias euclidianas
   - ✅ **FASE 1:** `build_types()` - Clasificación de nodos y construcción de U
   - ✅ **FASE 2:** `build_graph()` - Grafo extendido con restricción B_max
   - ✅ **FASE 3:** `preprocess_paths()` - Colapso del grafo
   - ✅ `dijkstra_lex()` - Dijkstra con orden lexicográfico
   - ✅ `reconstruct_path()` - Reconstrucción de caminos
   - ✅ Constructor con flujo completo de inicialización

### 3. **PREPROCESAMIENTO.md** - Documento explicativo (NUEVO)
   - Arquitectura completa del preprocesamiento
   - Explicación fase por fase con ejemplos
   - Diagramas de flujo conceptuales
   - Complejidad computacional
   - Conexión con el algoritmo split
   - Referencias al código fuente

### 4. **test_preprocessing.cpp** - Programa de prueba (NUEVO)
   - Visualización de matrices W, Rcnt, PathUV
   - Verificación de que el preprocesamiento funciona
   - Muestra ejemplos de caminos con estaciones

### 5. **README_DOCUMENTACION.md** - Este archivo (NUEVO)

---

## 🎯 Cómo Funciona el Preprocesamiento

### Objetivo Principal
Convertir el problema EVRP en un problema VRP clásico mediante el **colapso del grafo**:

```
Grafo Completo (N nodos)           →    Grafo Contraído (M nodos)
[Depósito + Clientes + Estaciones] →    [Depósito + Clientes]
```

### Las 4 Fases

#### FASE 0: Matriz de Distancias
```cpp
matrix_distances(coords, P, S)
```
- Calcula distancias euclidianas entre todos los nodos
- Matriz simétrica N×N

#### FASE 1: Clasificación de Nodos
```cpp
build_types()
```
- Asigna tipo a cada nodo: `Depot`, `Client`, `Station`
- Construye conjunto U = {Depósito} ∪ {Clientes}

#### FASE 2: Grafo Extendido
```cpp
build_graph()
```
- Solo crea arista a→b si `dist[a][b] ≤ B_max`
- Peso: `w = C_km × dist + (destino es estación ? C_rec : 0)`
- Asume recarga al 100% al llegar a estación

#### FASE 3: Colapso del Grafo
```cpp
preprocess_paths()
  ├─ dijkstra_lex()      // Caminos mínimos con orden lexicográfico
  └─ reconstruct_path()  // Reconstruir caminos con estaciones
```
- Ejecuta Dijkstra desde cada nodo en U hacia todos los demás
- Genera matrices:
  - **W[i][j]**: Costo mínimo U[i] → U[j]
  - **Rcnt[i][j]**: Número de recargas
  - **PathUV[i][j]**: Camino completo [U[i], ..., estaciones, ..., U[j]]

---

## 🔬 Verificación del Preprocesamiento

### Compilar y Probar
```bash
# Compilar programa de prueba
g++ -std=c++17 -o test_preprocessing test_preprocessing.cpp GA.cpp -lm

# Ejecutar
./test_preprocessing
```

### Salida Esperada
```
=== TEST DE PREPROCESAMIENTO EVRP ===

📊 PARÁMETROS DEL PROBLEMA
💰 MATRIZ W (Costos mínimos)
🔋 MATRIZ Rcnt (Recargas)
🗺️  EJEMPLOS DE CAMINOS

✅ Preprocesamiento completado exitosamente!
```

---

## 🧩 Conceptos Clave

### 1. Orden Lexicográfico en Dijkstra
```cpp
Prioridad: (distancia, recargas)
1º: Minimizar distancia/costo
2º: Si hay empate, minimizar recargas
```

### 2. Grafo Extendido
- **Restricción de autonomía:** Solo existe arista si `dist ≤ B_max`
- **Modelo de recarga:** Siempre al 100% al llegar a estación
- **Costo de recarga:** Se cobra al LLEGAR a la estación

### 3. Conjunto U
```
U = {Depósito} ∪ {Clientes}
|U| = M = P + 1

Índices en U:
U[0] = nodo 0 (Depósito)
U[1] = nodo 1 (Cliente 1)
...
U[P] = nodo P (Cliente P)
```

### 4. Almacenamiento Aplanado
```cpp
PathUV[i * M + j]  // Camino de U[i] → U[j]
```
Optimización de memoria en lugar de usar `vector<vector<vector<int>>>`

---

## 🔗 Conexión con Split

Una vez preprocesado:

```cpp
// 1. Generar cromosoma (permutación de clientes)
Cromosoma: [3, 1, 2]  // Sin depósito ni estaciones

// 2. Split usa matriz W como si fuera VRP clásico
split(W, demand, Q, B) {
    // W[i][j] ya incluye estaciones óptimas
    // Split no necesita saber nada sobre batería
}

// 3. Reconstruir solución real
for (ruta in solucion) {
    for (i, j) in pares_consecutivos(ruta) {
        camino_real = PathUV[i * M + j];  // Incluye estaciones
    }
}
```

---

## 📊 Ejemplo Visual

### Problema
```
Nodos: D, C1, C2, C3, E1
B_max = 100 km
```

### Antes del Preprocesamiento
```
D --80-- C1 --110-- E1 --50-- C2
                            |
                          90
                            |
                           C3 --70-- D
```

### Después del Preprocesamiento
```
Matriz W (solo nodos obligatorios):
        D     C1    C2    C3
D       -     80    240   330
C1      80    -     160   250
C2      240   160   -     90
C3      330   250   90    -

El GA solo ve estas distancias "mágicas"
(Las estaciones están ocultas en W)
```

---

## 📖 Supuestos del Modelo

### Confirmados por el Usuario
1. ✅ **Recarga al 100%:** Siempre que se visita una estación
2. ✅ **Costo al llegar:** C_rec se cobra al LLEGAR a la estación
3. ✅ **Sin cargas parciales:** Solo carga completa (no hay modelo no lineal)
4. ✅ **Capacidad Q:** Solo se considera en split, no en preprocesamiento

### Implementados en el Código
- Dijkstra encuentra caminos minimizando (distancia, recargas) lexicográficamente
- PathUV guarda el camino completo incluyendo todos los nodos intermedios
- Validación: Todos los clientes deben ser alcanzables desde el depósito

---

## 🚀 Próximos Pasos

Con el preprocesamiento completo, puedes implementar:

### 1. Función Split
```cpp
// Usa W para particionar la permutación en rutas
vector<Route> split(vector<int> permutation, W, demand, Q, B);
```

### 2. Algoritmo Genético
```cpp
// Población de cromosomas (permutaciones)
Population pop = initialize_population(P);

while (!termination_criteria) {
    // Evaluar fitness usando split
    for (chromosome in pop) {
        fitness[chromosome] = split(chromosome).total_cost;
    }
    
    // Operadores genéticos
    pop = selection(pop);
    pop = crossover(pop);
    pop = mutation(pop);
}
```

### 3. Reconstrucción de Solución Final
```cpp
// Convertir rutas del split en rutas reales con estaciones
Solution reconstruct_solution(split_result, PathUV);
```

---

## 🐛 Debugging y Validación

### Verificar que el Preprocesamiento es Correcto

```bash
# 1. Compilar con flags de debug
g++ -std=c++17 -g -O0 -o test_preprocessing test_preprocessing.cpp GA.cpp -lm

# 2. Ejecutar
./test_preprocessing

# 3. Verificar:
# ✓ Todos los W[0][i] y W[i][0] son finitos (clientes alcanzables)
# ✓ W[i][j] ≥ dist[U[i]][U[j]] (puede ser mayor por estaciones)
# ✓ Rcnt[i][j] ≥ 0 (número de recargas no negativo)
# ✓ PathUV[i][j] empieza en U[i] y termina en U[j]
```

---

## 📚 Referencias en el Código

| Concepto | Ubicación | Línea |
|----------|-----------|-------|
| Clase principal | `GA.h` | 54-160 |
| Struct Edge | `GA.h` | 19-23 |
| Enum NodeType | `GA.h` | 29-33 |
| Constructor | `GA.cpp` | 289-344 |
| FASE 0 | `GA.cpp` | 349-378 |
| FASE 1 | `GA.cpp` | 212-248 |
| FASE 2 | `GA.cpp` | 250-287 |
| FASE 3 | `GA.cpp` | 12-74 |
| Dijkstra lex | `GA.cpp` | 143-210 |
| Reconstruct path | `GA.cpp` | 76-113 |

---

## ✨ Ventajas de esta Arquitectura

1. **Modularidad:** Preprocesamiento separado del GA
2. **Eficiencia:** Cálculos pesados se hacen una sola vez
3. **Simplicidad:** Split funciona como VRP clásico
4. **Corrección:** Garantiza factibilidad de autonomía
5. **Flexibilidad:** Fácil cambiar estrategias de búsqueda

---

## 🎓 Conclusión

Tu infraestructura de preprocesamiento está completa y documentada. Permite:

- ✅ Transformar EVRP en VRP mediante colapso de grafo
- ✅ Usar representación TSP-like sin trip delimiters
- ✅ Aplicar split estándar sin modificaciones
- ✅ Garantizar factibilidad de autonomía
- ✅ Obtener soluciones con estaciones de recarga óptimas

**El preprocesamiento funciona correctamente** según lo verificado con el programa de prueba. Ahora puedes proceder a implementar la función split y el algoritmo genético. 🚀

