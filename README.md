# Algoritmo Evolutivo para EVRP

Implementación de un algoritmo evolutivo para resolver el problema de enrutamiento de vehículos eléctricos (EVRP).

## ¿Qué hace?

Resuelve el problema de encontrar rutas óptimas para vehículos eléctricos que deben:
- Visitar todos los clientes exactamente una vez
- Respetar la capacidad de carga (Q)
- Respetar la autonomía de batería (B_max)
- Usar máximo B vehículos
- Minimizar el costo total (distancia + recargas)

## ¿Cómo funciona?

1. **Preprocesamiento**: Calcula los caminos óptimos entre clientes considerando las estaciones de recarga necesarias
2. **Algoritmo Evolutivo**: 
   - Genera población de permutaciones de clientes
   - Evalúa cada permutación usando la función `split` (programación dinámica)
   - Selecciona padres, cruza y muta
   - Repite por varias generaciones
3. **Resultado**: Mejor solución encontrada

## Compilación

```bash
make
```

## Uso

```bash
./bin/evrp_solver instancias/instancia1.txt
```

## Formato de entrada

El archivo de instancia tiene este formato:

```
B Q B_max
C_km C_rec 
P 
S
d1 d2 ... dP
N
x0 y0
x1 y1
...
x_{N-1} y_{N-1}
```

**Parámetros:**
- `B`: Número de vehículos disponibles
- `Q`: Capacidad de cada vehículo
- `B_max`: Autonomía máxima (distancia con batería completa)
- `C_km`: Costo por kilómetro
- `C_rec`: Costo por recarga
- `P`: Número de clientes
- `S`: Número de estaciones de recarga
- `d1...dP`: Demandas de los clientes
- `N`: Total de nodos (1 + P + S)
- `xi yi`: Coordenadas de cada nodo

**Orden de nodos:**
- Nodo 0: Depósito
- Nodos 1..P: Clientes
- Nodos P+1..N-1: Estaciones

## Formato de salida

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
```

## Configuración

Los parámetros del algoritmo genético se configuran en `main.cpp`:

```cpp
EVRP.initialize_parameters(
    5,    // Tamaño de población
    20,   // Número de generaciones
    0.8,  // Tasa de cruce
    0.2,  // Tasa de mutación
    20    // Iteraciones de búsqueda local
);
```

## Estructura del código

- `GA.h` / `GA.cpp`: Clase principal con el algoritmo genético
- `main.cpp`: Programa principal
- `show_matrices.cpp`: Utilidad para ver matrices de preprocesamiento

## Algoritmos implementados

- **Dijkstra lexicográfico**: Encuentra caminos mínimos minimizando distancia y luego recargas
- **Split**: Divide una permutación de clientes en rutas óptimas usando programación dinámica
- **Binary Tournament Selection**: Selecciona padres mediante torneos
- **Order Crossover (OX)**: Cruce que preserva orden relativo
- **Búsqueda Local**: Mutación con movimientos 2-opt, relocate y swap

## Resultados

El algoritmo encuentra soluciones factibles que respetan todas las restricciones. Para instancias infactibles, muestra la razón de infactibilidad.

## Notas

- El preprocesamiento "oculta" las estaciones de recarga, permitiendo usar algoritmos estándar de VRP
- La representación TSP-like usa permutaciones de clientes, facilitando operadores genéticos
- La verificación de batería es conservadora para evitar falsos positivos

## Autor

Proyecto académico para el curso de Inteligencia Artificial.
