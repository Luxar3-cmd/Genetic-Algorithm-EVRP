# REPORTE: Análisis del Problema con instancia4.txt

## Resumen Ejecutivo

La instancia4.txt produce una solución que **viola la restricción de batería** (B_max=70). La ruta del Vehículo 3 reporta una distancia total de 122 unidades sin recargas, cuando la autonomía máxima es 70. El problema radica en que el **preprocesamiento no considera el consumo acumulativo de batería** cuando se concatenan múltiples segmentos en una ruta completa.

---

## 1. Descripción del Problema

### 1.1 Instancia
- **B** (vehículos): 3
- **Q** (capacidad): 12
- **B_max** (autonomía): 70
- **P** (clientes): 6
- **S** (estaciones): 2

### 1.2 Solución Generada (INCORRECTA)
```
Vehiculo 3:
Ruta: D -> C5 -> C2 -> D
Distancia: 122
Recargas: 0
```

**Análisis:**
- D -> C5: 60.21 (dentro de B_max=70) ✓
- C5 -> C2: 52.01 (dentro de B_max=70) ✓
- C2 -> D: 10.00 (dentro de B_max=70) ✓
- **Distancia total: 122.22 > B_max=70** ✗

### 1.3 Problema Fundamental

El preprocesamiento calcula caminos **entre pares de nodos** asumiendo que cada segmento comienza con **batería completa (B_max)**. Sin embargo, cuando `split()` construye una ruta completa concatenando múltiples segmentos, la batería se consume **acumulativamente** y no se verifica la batería residual entre segmentos.

---

## 2. Análisis del Flujo de Ejecución

### 2.1 Fase de Preprocesamiento (`preprocess_paths()`)

**Ubicación:** `GA.cpp:40-94`

**Proceso:**
1. Para cada par de nodos (i,j) en U, ejecuta `dijkstra_lex(i)` 
2. Calcula el camino mínimo de i a j **asumiendo batería inicial = B_max**
3. Almacena en `W[i][j]` el costo y en `PathUV[i*M+j]` el camino completo

**Problema identificado:**
- `dijkstra_lex()` siempre inicia con `initial_battery = B_max` (línea 203)
- No hay forma de especificar batería residual al inicio de un segmento
- Los caminos preprocesados son **independientes** y no consideran el contexto de la ruta completa

### 2.2 Función Split (`split()`)

**Ubicación:** `GA.cpp:631-731`

**Proceso:**
1. Construye rutas concatenando segmentos usando la matriz W preprocesada
2. Calcula costos sumando: `W[depot][first] + W[first][second] + ... + W[last][depot]`
3. **NO verifica batería residual** entre segmentos

**Código problemático (líneas 728-710):**
```cpp
// Depósito → Primer cliente
route_cost += W[u_idx_depot][u_idx_first];

// Entre clientes consecutivos
for (size_t k = 0; k + 1 < route_clients.size() && route_feasible; k++) {
    route_cost += W[u_idx_from][u_idx_to];  // ⚠️ Asume batería completa
}

// Último cliente → Depósito
route_cost += W[u_idx_last][u_idx_depot];  // ⚠️ Asume batería completa
```

**Problema:**
- Cada segmento usa `W[i][j]` que asume batería completa al inicio
- No hay verificación de que la batería residual después del segmento anterior sea suficiente para el siguiente
- La batería se consume acumulativamente pero no se rastrea

### 2.3 Función Dijkstra Lexicográfico (`dijkstra_lex()`)

**Ubicación:** `GA.cpp:194-265`

**Proceso:**
1. Inicializa con `initial_battery = B_max` (línea 203)
2. Verifica batería en cada arco: `battery_after_move = s.battery - real_dist` (línea 228)
3. Si llega a estación, recarga a `B_max` (línea 240)

**Limitación:**
- Solo puede calcular caminos desde un nodo con batería completa
- No puede calcular caminos con batería residual arbitraria
- Esto es correcto para el preprocesamiento, pero **insuficiente** para verificar rutas completas

---

## 3. Funciones en Conflicto

### 3.1 Conflicto Principal: Preprocesamiento vs. Construcción de Rutas

**Función A:** `preprocess_paths()` + `dijkstra_lex()`
- **Asunción:** Cada segmento comienza con batería completa
- **Resultado:** Matriz W con costos óptimos entre pares

**Función B:** `split()` + `reconstruct_solution_from_split()`
- **Uso:** Concatena segmentos de W para formar rutas completas
- **Problema:** No verifica batería residual acumulativa

**Conflicto:**
- El preprocesamiento es correcto para segmentos individuales
- Pero `split()` asume incorrectamente que puede concatenar segmentos sin verificar batería residual
- La batería se consume a lo largo de la ruta completa, no se reinicia en cada segmento

### 3.2 Función de Expansión de Rutas (`expand_route()`)

**Ubicación:** `GA.cpp:1527-1608`

**Proceso:**
- Expande una ruta usando `PathUV` para mostrar el camino completo con estaciones
- Usa los caminos preprocesados que ya incluyen estaciones si son necesarias

**Observación:**
- Esta función es correcta porque solo **muestra** la solución
- El problema está en la **construcción** de la solución, no en su visualización

### 3.3 Función de Cálculo de Distancia (`calculate_route_distance()`)

**Ubicación:** `GA.cpp:1618-1626`

**Proceso:**
- Suma distancias euclidianas entre nodos consecutivos en el camino expandido
- No verifica restricciones de batería

**Observación:**
- Esta función solo calcula la distancia total para mostrar
- No participa en la validación de factibilidad

---

## 4. Análisis de la Instancia Específica

### 4.1 Ruta Problemática: D -> C5 -> C2 -> D

**Segmentos individuales (según preprocesamiento):**
- D -> C5: distancia 60.21, costo 60.21, 0 recargas
- C5 -> C2: distancia 52.01, costo 52.01, 0 recargas  
- C2 -> D: distancia 10.00, costo 10.00, 0 recargas

**Análisis de batería (simulación manual):**
1. **Inicio en D:** batería = 70
2. **D -> C5:** consume 60.21, batería residual = 9.79
3. **C5 -> C2:** necesita 52.01, pero solo hay 9.79 ✗ **INVÁLIDO**
4. **Solución:** Debe pasar por estación entre C5 y C2

**Camino correcto debería ser:**
- D -> C5: 60.21 (batería: 70 -> 9.79)
- C5 -> R1 o R2 -> C2: necesita recarga intermedia
- C2 -> D: 10.00

### 4.2 Por qué el Preprocesamiento No Detecta Esto

El preprocesamiento calcula `W[C5_idx][C2_idx]` asumiendo:
- Batería inicial en C5 = 70 (completa)
- Puede ir directamente C5 -> C2 (52.01 < 70) ✓

Pero en la ruta real:
- Batería en C5 después de D -> C5 = 9.79 (residual)
- No puede ir directamente C5 -> C2 (52.01 > 9.79) ✗

**El preprocesamiento es correcto para segmentos aislados, pero incorrecto cuando se concatenan en rutas completas.**

---

## 5. Razones por las que Otras Instancias Funcionan

### 5.1 Instancias que Funcionan Correctamente

Probablemente tienen una o más de estas características:
1. **Rutas cortas:** Cada ruta tiene pocos clientes, por lo que la batería residual es suficiente
2. **Clientes cercanos:** Las distancias entre clientes consecutivos son pequeñas
3. **Estaciones estratégicas:** Las estaciones están ubicadas de manera que los caminos preprocesados ya las incluyen automáticamente

### 5.2 Por qué instancia4.txt Falla

1. **Cliente C5 muy lejano:** D -> C5 consume casi toda la batería (60.21 de 70)
2. **C5 -> C2 largo:** 52.01 requiere más batería de la disponible después de D -> C5
3. **Preprocesamiento optimista:** W[C5][C2] asume batería completa, pero en la ruta real hay batería residual

---

## 6. Funciones que Deberían Verificar Batería (pero no lo hacen)

### 6.1 `split()` - Construcción de Rutas

**Ubicación:** `GA.cpp:631-731`

**Problema:**
- Líneas 728-710: Suma costos de segmentos sin verificar batería residual
- Debería simular batería a lo largo de la ruta completa
- Debería detectar cuando un segmento requiere más batería de la disponible

**Solución requerida (NO implementar, solo documentar):**
- Rastrear batería residual después de cada segmento
- Si batería residual < distancia del siguiente segmento, forzar paso por estación
- O recalcular camino con batería residual usando función auxiliar

### 6.2 `reconstruct_solution_from_split()` - Reconstrucción de Solución

**Ubicación:** `GA.cpp:745-938`

**Problema:**
- Líneas 877-904: Calcula costos de rutas usando W sin verificar batería
- Asume que si W es finito, la ruta es factible
- No simula batería a lo largo de la ruta completa

**Solución requerida (NO implementar, solo documentar):**
- Después de calcular cada ruta, simular batería paso a paso
- Verificar que batería nunca sea negativa
- Si se detecta violación, marcar ruta como infactible

### 6.3 `dijkstra_lex()` - Cálculo de Caminos

**Ubicación:** `GA.cpp:194-265`

**Limitación (no es un error, es una limitación de diseño):**
- Solo puede calcular caminos con batería inicial = B_max
- No puede calcular caminos con batería residual arbitraria
- Esto es correcto para el preprocesamiento, pero limita la verificación de rutas completas

**Solución requerida (NO implementar, solo documentar):**
- Agregar parámetro opcional `initial_battery` a `dijkstra_lex()`
- Permitir calcular caminos con batería residual
- Usar en `split()` para verificar segmentos con batería residual

---

## 7. Conclusión

### 7.1 Problema Raíz

El problema fundamental es un **desajuste entre el modelo de preprocesamiento y el modelo de construcción de rutas**:

- **Preprocesamiento:** Asume batería completa al inicio de cada segmento
- **Construcción de rutas:** Concatena segmentos sin verificar batería residual acumulativa

### 7.2 Por qué es Difícil de Arreglar

1. **Cambiar preprocesamiento:** Requeriría calcular caminos con todas las posibles baterías residuales (explosión combinatoria)
2. **Cambiar split():** Requería verificación de batería en tiempo de construcción, pero W no contiene información de batería residual
3. **Verificación post-construcción:** Podría detectar el problema pero no prevenirlo, requiriendo reconstrucción de rutas

### 7.3 Recomendaciones (sin modificar código)

1. **Verificación post-construcción:** Agregar función que simule batería en rutas completas después de `reconstruct_solution_from_split()`
2. **Marcar rutas infactibles:** Si se detecta violación de batería, marcar solución como infactible con `FeasibilityReason::BatteryInsufficient`
3. **Penalización en fitness:** Asignar fitness infinito a soluciones con violación de batería
4. **Reconstrucción inteligente:** Si se detecta violación, intentar insertar estaciones en puntos críticos

### 7.4 Impacto en Otras Instancias

El problema probablemente afecta otras instancias con características similares:
- Clientes muy lejanos del depósito
- Rutas con múltiples clientes lejanos
- Distancias entre clientes consecutivos que suman más de B_max

Las instancias que funcionan probablemente tienen rutas más cortas o clientes más cercanos entre sí.

---

## 8. Análisis Técnico Detallado

### 8.1 Matriz W Preprocesada (instancia4.txt)

```
D0 -> C5: 60.21 (asume batería=70, factible)
C5 -> C2: 52.01 (asume batería=70, factible)
C2 -> D0: 10.00 (asume batería=70, factible)
```

**Problema:** Cada entrada asume batería completa, pero en la ruta real:
- Después de D -> C5, batería = 9.79
- C5 -> C2 necesita 52.01, pero solo hay 9.79 ✗

### 8.2 Flujo de Datos Problemático

```
preprocess_paths()
  └─> dijkstra_lex(start, ...)  [batería inicial = B_max siempre]
       └─> Calcula W[i][j] para todos los pares
            └─> Almacena en matriz W

split(chromosome)
  └─> Usa W[depot][client1] + W[client1][client2] + ... + W[clientN][depot]
       └─> ⚠️ NO verifica batería residual entre segmentos
            └─> Construye ruta que puede violar B_max

reconstruct_solution_from_split()
  └─> Usa W para calcular costos de rutas
       └─> ⚠️ NO simula batería a lo largo de la ruta
            └─> Marca solución como factible aunque viole B_max
```

### 8.3 Puntos de Verificación Faltantes

1. **En `split()` (línea ~715):** Debería verificar batería antes de aceptar ruta
2. **En `reconstruct_solution_from_split()` (línea ~904):** Debería simular batería después de calcular cada ruta
3. **En `evaluate_population()` (línea ~981):** Debería verificar batería antes de aceptar solución como factible

---

## 9. Referencias de Código

### Funciones Clave:
- `preprocess_paths()`: `GA.cpp:40-94`
- `dijkstra_lex()`: `GA.cpp:194-265`
- `split()`: `GA.cpp:631-731`
- `reconstruct_solution_from_split()`: `GA.cpp:745-938`
- `expand_route()`: `GA.cpp:1527-1608`
- `calculate_route_distance()`: `GA.cpp:1618-1626`

### Estructuras de Datos:
- `W`: Matriz de costos preprocesados (asume batería completa)
- `PathUV`: Caminos completos con estaciones (asume batería completa)
- `Route`: Estructura que almacena ruta sin información de batería residual

---

**Fecha del Reporte:** Generado automáticamente  
**Instancia Analizada:** instancia4.txt  
**Estado:** Problema identificado, solución requiere modificación de arquitectura

