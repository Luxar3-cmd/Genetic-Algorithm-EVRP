# 🚀 Guía Rápida - Preprocesamiento EVRP

## 📋 TL;DR

**Problema:** Resolver EVRP con GA usando representación TSP-like  
**Solución:** Preprocesar el grafo para ocultar estaciones y batería  
**Resultado:** Split funciona como VRP clásico

---

## ⚡ Flujo del Preprocesamiento

```
Archivo de instancia
       ↓
  Constructor
       ↓
┌──────────────────┐
│ FASE 0: Distancias │ → dist[N][N]
└──────────────────┘
       ↓
┌──────────────────┐
│ FASE 1: Tipos    │ → type[], U[]
└──────────────────┘
       ↓
┌──────────────────┐
│ FASE 2: Grafo    │ → adj[]
└──────────────────┘
       ↓
┌──────────────────┐
│ FASE 3: Colapso  │ → W[][], Rcnt[][], PathUV[]
└──────────────────┘
       ↓
  ✅ Listo para GA
```

---

## 🔑 Estructuras Clave

### Matrices de Entrada
```cpp
dist[i][j]   // Distancia euclidiana entre nodos i y j
type[i]      // Depot, Client o Station
adj[u]       // Lista de aristas desde u (solo si dist ≤ B_max)
```

### Matrices de Salida (las importantes)
```cpp
W[i][j]      // Costo mínimo: U[i] → U[j] (incluye estaciones)
Rcnt[i][j]   // Número de recargas en ese camino
PathUV[i][j] // Camino completo: [U[i], ..., estaciones, ..., U[j]]
```

---

## 💡 Conceptos en 3 Líneas

### Grafo Extendido
Solo conecta nodos si `dist ≤ B_max`. Peso = `C_km × dist + C_rec` (si llega a estación).

### Dijkstra Lexicográfico
Minimiza primero distancia, luego recargas. Usa priority_queue invertido.

### Colapso del Grafo
Ejecuta Dijkstra desde cada U[i], guarda costos hacia cada U[j]. Estaciones quedan ocultas en W.

---

## 🎯 Uso del Preprocesamiento

### En tu GA
```cpp
// 1. Inicializar (hace preprocesamiento automáticamente)
evolutionaryAlgo EVRP("instancia.txt");

// 2. Usar matrices preprocesadas
double costo_depot_a_cliente1 = EVRP.W[0][1];
int recargas_necesarias = EVRP.Rcnt[0][1];
vector<int> camino = EVRP.PathUV[0 * M + 1];

// 3. En split, usar W como matriz de distancias
split(cromosoma, EVRP.W, EVRP.demand, EVRP.Q, EVRP.B);
```

---

## 🔍 Debugging Rápido

### Verificar Preprocesamiento
```bash
./test_preprocessing
```

### Checklist
- [ ] Todos los W[0][i] son finitos (clientes alcanzables)
- [ ] W[i][j] ≥ dist[U[i]][U[j]] (puede incluir estaciones)
- [ ] Rcnt[i][j] ≥ 0
- [ ] PathUV empieza en U[i] y termina en U[j]

---

## 📐 Fórmulas Importantes

### Peso de Arista
```
w(a→b) = C_km × dist[a][b] + (b es estación ? C_rec : 0)
```

### Orden Lexicográfico
```
(d1, r1) < (d2, r2)  ⟺  d1 < d2  ∨  (d1 = d2  ∧  r1 < r2)
```

### Índice Aplanado
```
PathUV[i * M + j]  donde M = P + 1
```

---

## 🎨 Ejemplo Mínimo

### Instancia
```
2 13 200 1 8 3 1    # B Q B_max C_km C_rec P S
8 7 10              # demandas
5                   # N nodos
0 0                 # Depot
10 0                # Cliente 1
6 8                 # Cliente 2
14 5                # Cliente 3
8 3                 # Estación 1
```

### Resultado del Preprocesamiento
```
U = {0, 1, 2, 3}   # Depósito + 3 clientes

W[0][1] = 10.0     # Depot → C1: directo
W[0][2] = 10.0     # Depot → C2: directo
W[1][2] = 8.9      # C1 → C2: directo

PathUV[0→1] = [0, 1]         # Sin estaciones intermedias
PathUV[1→2] = [1, 2]         # Sin estaciones intermedias
PathUV[0→2] = [0, 2]         # Sin estaciones intermedias
```

---

## 🚨 Puntos Críticos

### 1. B_max es Crítico
Si B_max es muy pequeño, algunos clientes serán inalcanzables → Error.

### 2. Costo de Recarga
Se cobra al **LLEGAR** a la estación, no al salir.

### 3. Recarga Completa
Siempre al 100%, no hay cargas parciales.

### 4. Conjunto U
Solo contiene **Depósito + Clientes**, NO estaciones.

---

## 📚 Documentación Completa

- **PREPROCESAMIENTO.md**: Explicación detallada de cada fase
- **README_DOCUMENTACION.md**: Resumen del trabajo completo
- **GA.h / GA.cpp**: Código comentado línea por línea
- **test_preprocessing.cpp**: Programa de prueba

---

## ✅ Checklist de Implementación

### Preprocesamiento (Completado ✓)
- [x] Matriz de distancias euclidianas
- [x] Clasificación de nodos por tipo
- [x] Construcción de grafo con restricción B_max
- [x] Dijkstra lexicográfico
- [x] Colapso del grafo a matriz W

### Por Implementar
- [ ] Función Split
- [ ] Inicialización de población
- [ ] Operadores genéticos (crossover, mutación)
- [ ] Selección de padres
- [ ] Evaluación de fitness
- [ ] Criterio de terminación
- [ ] Reconstrucción de solución final

---

## 🎓 Reglas de Oro

1. **W ya incluye todo:** No calcules costos manualmente en el GA
2. **U son los únicos nodos visibles:** El GA nunca ve estaciones directamente
3. **Split es estándar:** No necesita modificaciones para EVRP
4. **PathUV para solución final:** Úsalo para mostrar rutas con estaciones
5. **Preprocesamiento una vez:** Se hace en el constructor, no se repite

---

## 🔧 Compilación Rápida

```bash
# Compilar programa principal
g++ -std=c++17 -o program main.cpp GA.cpp -lm

# Compilar y ejecutar test
g++ -std=c++17 -o test_preprocessing test_preprocessing.cpp GA.cpp -lm
./test_preprocessing
```

---

## 💬 Contacto Rápido

**¿Necesitas más ayuda?**
- Revisa PREPROCESAMIENTO.md para detalles
- Ejecuta test_preprocessing para verificar
- Lee comentarios en GA.cpp para entender el código

---

**¡Listo para implementar split y el GA!** 🚀

