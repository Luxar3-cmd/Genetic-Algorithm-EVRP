# REPORTE: Pipeline del Algoritmo Genético para EVRP

## Resumen Ejecutivo

Este documento describe el flujo completo del algoritmo genético implementado para resolver el problema de enrutamiento de vehículos eléctricos (EVRP). El algoritmo utiliza una representación TSP-like (permutación de clientes) y la función `split` para decodificar cromosomas en soluciones VRP completas.

---

## 1. Arquitectura General

### 1.1 Representación de Soluciones

**Cromosoma (Individuo):**
- Es una **permutación de clientes**: `[C1, C2, C3, ..., CP]`
- Representa el orden en que se visitarán los clientes
- **NO incluye** el depósito ni las estaciones de recarga
- Tamaño: P (número de clientes)

**Solución Decodificada:**
- Resultado de aplicar `split()` al cromosoma
- Contiene múltiples rutas (cada ruta = un vehículo)
- Cada ruta incluye: secuencia de clientes, costo, carga, factibilidad
- Las estaciones de recarga se insertan automáticamente por el preprocesamiento

### 1.2 Flujo de Datos

```
Cromosoma (permutación) 
    ↓
split() [decodificación]
    ↓
Solución (rutas + factibilidad)
    ↓
Fitness = costo_total (∞ si infactible)
    ↓
Selección → Cruce → Mutación
    ↓
Nueva generación
```

---

## 2. Pipeline Completo del Algoritmo Genético

### FASE 0: Preprocesamiento (Antes del GA)

**Ubicación:** Constructor `evolutionaryAlgo()`

**Proceso:**
1. Lee la instancia del archivo
2. Calcula matriz de distancias euclidianas
3. Clasifica nodos (Depósito, Clientes, Estaciones)
4. Construye grafo extendido con restricción B_max
5. **Preprocesa caminos óptimos** entre todos los pares de nodos en U (depósito + clientes)
   - Calcula matriz W: costos mínimos entre nodos
   - Calcula PathUV: caminos completos con estaciones
   - Calcula Rcnt: número de recargas

**Resultado:** Matrices preprocesadas que permiten usar `split()` estándar sin conocer estaciones explícitamente.

---

### FASE 1: Inicialización

**Función:** `initialize_population()`

**Proceso:**
1. Genera `population_size` permutaciones aleatorias únicas de clientes [1..P]
2. Verifica que no haya duplicados
3. Si el tamaño de población excede el número de permutaciones posibles (P!), ajusta el tamaño

**Resultado:** Población inicial de cromosomas (permutaciones)

**Ejemplo:**
```
Población inicial (P=5, population_size=3):
Individuo 1: [3, 1, 5, 2, 4]
Individuo 2: [1, 4, 3, 5, 2]
Individuo 3: [5, 2, 1, 4, 3]
```

---

### FASE 2: Evaluación de Población

**Función:** `evaluate_population()`

**Proceso para cada individuo:**

#### 2.1 Decodificación (split)
1. Toma el cromosoma (permutación de clientes)
2. Llama a `split(chromosome)` que:
   - Construye un grafo auxiliar usando programación dinámica
   - Encuentra la partición óptima en rutas
   - Verifica restricciones de capacidad (Q) y flota (B)
   - Verifica restricción de batería (post-construcción)
   - Reconstruye la solución completa

#### 2.2 Cálculo de Fitness
- **Si la solución es factible:** `fitness = total_cost`
- **Si la solución es infactible:** `fitness = ∞` (infinito)

#### 2.3 Actualización de Mejor Solución

**Estrategia importante:**
```cpp
// Prioridad 1: Soluciones factibles con menor costo
if (sol.is_feasible && sol.total_cost < best_cost) {
    best_solution = sol;
    best_fitness_index = i;
}
// Prioridad 2: Si no hay factibles, guardar primera infactible
else if (best_fitness_index < 0 && !sol.is_feasible) {
    best_solution = sol;  // Para mostrar detalles de infactibilidad
    best_fitness_index = i;
}
```

**Observación clave:** El algoritmo **SÍ encuentra soluciones infactibles** y las guarda. Solo las compara con factibles para actualizar la mejor solución. Si todas son infactibles, guarda la primera para mostrar detalles.

**Contadores de infactibilidad:**
- `num_feasible`: Soluciones factibles
- `num_capacity_exceeded`: Exceden capacidad Q
- `num_fleet_exceeded`: Exceden número de vehículos B
- `num_battery_insufficient`: Violan restricción de batería
- `num_unreachable`: Clientes inalcanzables

---

### FASE 3: Selección de Padres

**Función:** `selection()`

**Método:** Binary Tournament Selection (Torneo Binario)

**Proceso:**
1. Para cada posición en la población de padres:
   - Selecciona **2 individuos aleatorios diferentes** de la población
   - Compara sus fitness (menor = mejor)
   - Selecciona el ganador como padre
   - En caso de empate, selecciona aleatoriamente

**Características:**
- Genera `population_size` padres
- Los individuos con mejor fitness tienen mayor probabilidad de ser seleccionados
- Pero siempre hay probabilidad de seleccionar individuos peores (diversidad)
- Los individuos infactibles (fitness = ∞) **siempre pierden** contra factibles

**Ejemplo:**
```
Población:
  Individuo 0: fitness = 100 (factible)
  Individuo 1: fitness = ∞ (infactible)
  Individuo 2: fitness = 150 (factible)

Torneo 1: Individuo 0 vs Individuo 1 → Gana 0
Torneo 2: Individuo 1 vs Individuo 2 → Gana 2
Torneo 3: Individuo 0 vs Individuo 2 → Gana 0
```

---

### FASE 4: Cruce (Crossover)

**Función:** `crossover()`

**Método:** Order Crossover (OX)

**Proceso:**
1. Agrupa padres en pares: (padre[0], padre[1]), (padre[2], padre[3]), ...
2. Para cada par:
   - Con probabilidad `crossover_rate`: realiza cruce OX
   - Con probabilidad `1 - crossover_rate`: copia padres directamente
3. Genera 2 hijos por cada par de padres

**Order Crossover (OX):**
1. Selecciona un segmento aleatorio del padre 1
2. Copia ese segmento al hijo en las mismas posiciones
3. Completa el resto del hijo con elementos del padre 2 en el orden en que aparecen, omitiendo los ya copiados

**Ejemplo:**
```
Padre 1: [1, 2, 3, 4, 5, 6]
Padre 2: [4, 5, 6, 1, 2, 3]
Segmento: posiciones 1-2 → [2, 3]

Hijo 1:
  Paso 1: [_, 2, 3, _, _, _]
  Paso 2: Completar con Padre 2 → [5, 2, 3, 6, 1, 4]

Hijo 2: Similar pero con segmento de Padre 2
```

**Resultado:** Nueva población de hijos (reemplaza población anterior)

---

### FASE 5: Mutación

**Función:** `mutation()`

**Método:** Búsqueda Local con First Improvement

**Proceso:**
1. Para cada individuo en la población:
   - Con probabilidad `mutation_rate`: aplica búsqueda local
   - Con probabilidad `1 - mutation_rate`: no modifica

2. **Búsqueda Local** (`local_search_mutation()`):
   - Realiza hasta `local_search_iterations` (K_max) iteraciones
   - En cada iteración:
     - Selecciona un movimiento aleatorio: 2-opt, Relocate, o Swap
     - Aplica el movimiento al cromosoma
     - Evalúa el nuevo fitness
     - Si mejora estrictamente y es factible: **acepta y termina** (First Improvement)
     - Si no mejora: continúa hasta K_max iteraciones
   - Si no encuentra mejora: mantiene el individuo original

**Movimientos:**
- **2-opt:** Invierte un segmento [i..j]
- **Relocate:** Mueve elemento de posición i a j
- **Swap:** Intercambia elementos en posiciones i y j

**Restricción importante:** Solo aplica búsqueda local a individuos con fitness finito (factibles). Los infactibles (fitness = ∞) no se mutan.

---

### FASE 6: Loop Principal

**Función:** `run()`

**Flujo:**
```cpp
1. initialize_population()        // Genera población inicial
2. evaluate_population()          // Evalúa población inicial

3. Para cada generación (0..generations-1):
   a. selection()                 // Selecciona padres
   b. crossover()                 // Genera hijos
   c. mutation()                  // Aplica mutación
   d. evaluate_population()       // Evalúa nueva población

4. Imprime mejor solución encontrada
```

**Total de evaluaciones:** `population_size * (generations + 1)`
- +1 por la evaluación inicial

---

## 3. Manejo de Soluciones Factibles vs Infactibles

### 3.1 Criterios de Factibilidad

Una solución es **factible** si cumple **TODAS** estas condiciones:

1. **Capacidad:** Cada ruta tiene carga ≤ Q
2. **Flota:** Número de vehículos ≤ B
3. **Batería:** Cada ruta cumple restricción B_max (verificación post-construcción)
4. **Alcanzabilidad:** Todos los clientes son alcanzables desde el depósito

### 3.2 Asignación de Fitness

```cpp
if (sol.is_feasible) {
    fitness = sol.total_cost;  // Costo real
} else {
    fitness = ∞;  // Infinito (siempre peor que cualquier factible)
}
```

### 3.3 Comportamiento en Selección

- **Factible vs Factible:** Gana el de menor costo
- **Factible vs Infactible:** Siempre gana el factible
- **Infactible vs Infactible:** Ambos tienen fitness = ∞, se selecciona aleatoriamente

### 3.4 Comportamiento en Mutación

- **Solo se mutan individuos factibles** (fitness finito)
- Los infactibles se mantienen sin modificar
- Esto permite que la población mantenga diversidad incluso con muchos infactibles

### 3.5 Actualización de Mejor Solución

**Estrategia de dos niveles:**

1. **Nivel 1 (Prioridad):** Soluciones factibles
   - Solo actualiza si encuentra una factible con menor costo
   - `best_solution` siempre contiene la mejor solución factible encontrada

2. **Nivel 2 (Fallback):** Soluciones infactibles
   - Solo se guarda si no hay ninguna factible
   - Permite mostrar detalles de por qué no es factible

**Código:**
```cpp
// Prioridad 1: Factibles
if (sol.is_feasible && sol.total_cost < best_cost) {
    best_solution = sol;
}

// Prioridad 2: Infactibles (solo si no hay factibles)
else if (best_fitness_index < 0 && !sol.is_feasible) {
    best_solution = sol;  // Para mostrar detalles
}
```

---

## 4. Función Split: Decodificación de Cromosomas

### 4.1 Algoritmo Split

**Entrada:** Cromosoma (permutación de clientes)

**Proceso:**
1. **Programación Dinámica:**
   - Construye grafo auxiliar H donde:
     - Nodos: 0 a P (P = número de clientes)
     - Arco (i,j): representa ruta que atiende chromosome[i..j-1]
     - Peso: costo de ruta D → clientes[i..j-1] → D
   - Usa Bellman para encontrar camino de costo mínimo de 0 a P

2. **Reconstrucción:**
   - Sigue predecesores desde P hasta 0
   - Construye rutas correspondientes
   - Verifica restricciones:
     - Capacidad Q
     - Flota B
     - Batería B_max (post-construcción)

**Salida:** Solución con rutas, costos, y factibilidad

### 4.2 Verificaciones en Split

**Durante construcción (split):**
- Capacidad: `if (load > Q) break;`
- Alcanzabilidad: `if (!isfinite(W[i][j])) break;`

**Después de construcción (reconstruct_solution_from_split):**
- Capacidad: Verifica cada ruta reconstruida
- Flota: Verifica número total de vehículos
- Batería: Verificación conservadora post-construcción

---

## 5. Estadísticas y Monitoreo

### 5.1 Contadores en evaluate_population()

La función cuenta (pero no imprime) las razones de infactibilidad:
- `num_feasible`
- `num_capacity_exceeded`
- `num_fleet_exceeded`
- `num_battery_insufficient`
- `num_unreachable`
- `num_unknown`

**Nota:** Estos contadores están disponibles para debugging pero no se imprimen para mantener la salida limpia.

### 5.2 Información Final

Al terminar `run()`, se imprime:
- **Si hay solución factible:** Solución completa en formato especificado
- **Si no hay solución factible pero hay infactible guardada:**
  - Razón de infactibilidad
  - Detalles específicos
- **Si no hay ninguna solución:**
  - Mensaje con posibles razones

---

## 6. Ejemplo de Ejecución Completa

### Instancia: 5 clientes, B_max=70, Q=12, B=3

**Generación 0 (Inicial):**
```
Población inicial (3 individuos):
  [3,1,5,2,4] → split() → Solución: 2 rutas, costo=85, FACTIBLE
  [1,4,3,5,2] → split() → Solución: 3 rutas, costo=∞, INFACTIBLE (flota)
  [5,2,1,4,3] → split() → Solución: 2 rutas, costo=92, FACTIBLE

Fitness: [85, ∞, 92]
Mejor: Individuo 0 (costo=85, factible)
```

**Generación 1:**
```
Selección (torneos):
  Torneo 1: [3,1,5,2,4] vs [1,4,3,5,2] → Gana [3,1,5,2,4]
  Torneo 2: [5,2,1,4,3] vs [3,1,5,2,4] → Gana [3,1,5,2,4]
  Torneo 3: [1,4,3,5,2] vs [5,2,1,4,3] → Gana [5,2,1,4,3]

Padres: [[3,1,5,2,4], [3,1,5,2,4], [5,2,1,4,3]]

Cruce (crossover_rate=0.8):
  Par 1: [3,1,5,2,4] × [3,1,5,2,4] → [3,1,5,2,4], [3,1,5,2,4]
  Par 2: [5,2,1,4,3] (sin pareja) → [5,2,1,4,3]

Hijos: [[3,1,5,2,4], [3,1,5,2,4], [5,2,1,4,3]]

Mutación (mutation_rate=0.3):
  Individuo 0: No muta (probabilidad)
  Individuo 1: Búsqueda local → mejora → [3,5,1,2,4]
  Individuo 2: No muta

Nueva población: [[3,1,5,2,4], [3,5,1,2,4], [5,2,1,4,3]]

Evaluación:
  [3,1,5,2,4] → costo=85, FACTIBLE
  [3,5,1,2,4] → costo=82, FACTIBLE ← NUEVA MEJOR
  [5,2,1,4,3] → costo=92, FACTIBLE

Mejor actualizado: Individuo 1 (costo=82)
```

**Generación 2-N:**
- Continúa el proceso hasta `generations` iteraciones

**Final:**
- Imprime mejor solución encontrada (costo=82 en este ejemplo)

---

## 7. Puntos Clave del Pipeline

### 7.1 ¿Por qué se encuentran soluciones infactibles?

**Razón:** El algoritmo genético **explora el espacio de permutaciones**, no el espacio de soluciones factibles. La función `split()` puede generar soluciones infactibles cuando:
- La permutación requiere más vehículos de los disponibles (B)
- La permutación viola restricciones de capacidad o batería

**Ventaja:** Esto permite explorar más del espacio de búsqueda y encontrar mejores soluciones factibles.

### 7.2 ¿Cómo se manejan las soluciones infactibles?

1. **Fitness = ∞:** Siempre peor que cualquier factible
2. **Selección:** Siempre pierden contra factibles
3. **Mutación:** No se mutan (se mantienen para diversidad)
4. **Mejor solución:** Solo se guardan si no hay factibles (para mostrar detalles)

### 7.3 ¿Por qué se compara número de vehículos después de split?

**Razón:** El número de vehículos **no se conoce hasta después de aplicar split()**. La permutación solo define el orden de clientes, no cuántas rutas se necesitan. `split()` encuentra la partición óptima que puede requerir más vehículos de los disponibles.

**Ejemplo:**
```
Cromosoma: [1,2,3,4,5,6,7]
Después de split():
  Ruta 1: [1,2,3] → carga=15, necesita vehículo
  Ruta 2: [4,5] → carga=12, necesita vehículo
  Ruta 3: [6,7] → carga=10, necesita vehículo
Total: 3 vehículos

Si B=2: Solución INFACTIBLE (flota excedida)
Si B≥3: Solución puede ser factible (depende de otras restricciones)
```

---

## 8. Parámetros del Algoritmo

| Parámetro | Descripción | Valor Típico | Efecto |
|-----------|-------------|--------------|--------|
| `population_size` | Tamaño de la población | 50 | Más individuos = más exploración, más lento |
| `generations` | Número de generaciones | 100 | Más generaciones = más tiempo de búsqueda |
| `crossover_rate` | Probabilidad de cruce | 0.8 | Alto = más exploración, bajo = más explotación |
| `mutation_rate` | Probabilidad de mutación | 0.3 | Alto = más diversidad, bajo = más convergencia |
| `local_search_iterations` | Iteraciones de búsqueda local | 20 | Más iteraciones = mejor mejora local, más lento |

---

## 9. Complejidad Computacional

### Por Generación:
- **Selección:** O(population_size)
- **Cruce:** O(population_size × P)
- **Mutación:** O(population_size × mutation_rate × K_max × P)
- **Evaluación:** O(population_size × P²) [split es O(P²)]

**Total por generación:** O(population_size × P²)

**Total del algoritmo:** O(generations × population_size × P²)

---

## 10. Conclusiones

1. **El algoritmo SÍ encuentra soluciones infactibles** y las mantiene en la población para diversidad
2. **Solo compara factibles con factibles** para actualizar la mejor solución
3. **Las infactibles se usan para exploración** pero no compiten directamente con factibles
4. **El número de vehículos se determina después de split()**, no antes
5. **La verificación de batería es post-construcción** para detectar violaciones acumulativas

---

**Fecha del Reporte:** Generado automáticamente  
**Archivos Analizados:** GA.cpp, GA.h  
**Líneas de Código Relevantes:** 599-2018


