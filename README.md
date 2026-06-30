# ATM Trajectory Optimization — Sector LECMBLU

Optimización de trayectorias aéreas en el sector **LECMBLU** (ACC Madrid) para la resolución de conflictos entre aeronaves, comparando dos enfoques computacionales: **Gurobi** (Programación Entera Mixta) y **DEAP** (Algoritmo Genético).

## Caso de estudio

**Sector LECMBLU** — 24/08/2025, ventana horaria 11:09–11:19 UTC

Se adopta el rol del **Network Manager** en fase de planificación pre-táctica, reasignando ETOTs (Estimated Take-Off Times) a 8 aeronaves para eliminar conflictos en waypoints compartidos bajo el supuesto de mismo nivel de vuelo.

| Aeronave | Tipo | Entrada al sector | Duración en sector | Waypoints |
|---|---|---|---|---|
| N251SB | A320 | 11:09:00 | 8.0 min | BLV → BISKA → BADRU → BELEN |
| NJE299D | — | 11:11:00 | 12.1 min | DELOG |
| EXS2UY | — | 11:10:49 | 14.4 min | DELOG |
| TLJ263E | — | 11:11:16 | 18.6 min | NUBLO → RONSI → DELOG |
| RYR8605 | — | 11:13:25 | 14.1 min | DELOG |
| SCO282 | — | 11:14:11 | 9.7 min | BLV → BISKA → BADRU → BELEN |
| TAP642 | — | 11:15:51 | 18.9 min | NUBLO → RONSI → DELOG |
| DAH2700 | — | 11:16:10 | 3.1 min | BLV |

Datos extraídos de **EUROCONTROL NEST** (Network Strategic Tool).

## Problema de optimización

### Objetivo

Minimizar la desviación total respecto al plan de vuelo original:

$$\min \sum_{i=1}^{n} |t_i - t_i^0|$$

### Restricciones

- **Separación en waypoints** — cada par de aeronaves que comparte un waypoint debe mantener ≥ 2 min de separación temporal
- **Capacidad del sector** — máximo 8 aeronaves simultáneas en LECMBLU
- **Ventana temporal** — los nuevos tiempos de entrada deben estar en [11:09, 11:19]

### Matriz de conectividad

Describe qué waypoints transita cada aeronave dentro de LECMBLU (7 waypoints: BLV, BISKA, BADRU, NUBLO, RONSI, DELOG, BELEN):

```
         BLV  BISKA  BADRU  NUBLO  RONSI  DELOG  BELEN
DAH2700   1     0      0      0      0      0      0
TLJ263E   0     0      0      1      1      1      0
EXS2UY    0     0      0      0      0      1      0
TAP642    0     0      0      1      1      1      0
RYR8605   0     0      0      0      0      1      0
NJE299D   0     0      0      0      0      1      0
SCO282    1     1      1      0      0      0      1
N251SB    1     1      1      0      0      0      1
```

## Enfoques de resolución

### Gurobi — Programación Entera Mixta (MIP)

Solver determinista que garantiza la solución matemáticamente óptima mediante Branch and Bound. Los conflictos se eliminan como **hard constraints** inviolables usando variables binarias de orden.

Se implementan dos variantes:

- **`opti-TRY-GUROBI-defined.py`** — Tiempos de paso por waypoint calculados a partir de datos reales de NEST y fórmula de Haversine
- **`opti-TRY-GUROBI-random.py`** — Tiempos de paso generados aleatoriamente en el rango operacional plausible

### DEAP — Algoritmo Genético

Metaheurística evolutiva (50 individuos, 100 generaciones) que sustituye las restricciones por penalizaciones dominantes (10 000 unidades por conflicto) en la función de fitness. No garantiza óptimo global pero escala a problemas de mayor dimensión.

- **`opti-TRY-DEAP.py`** — Tiempos de paso aleatorios, mutación continua, selección por torneo

## Resultados

| Métrica | Gurobi (NEST) | Gurobi (random) | DEAP |
|---|---|---|---|
| Desviación total | **1.35 min** | 3.0 min | 3.3 min |
| Aeronaves afectadas | 1 | 3 | 6 |
| Conflictos resueltos | 1 | 3 | 3 |
| Conflictos residuales | 0 | 0 | 0 |
| Ocupación máxima | 8 | 8 | 8 |

Gurobi con datos reales de NEST obtiene la solución más eficiente, actuando solo sobre DAH2700 (+1.35 min). Sin embargo, en el contexto pre-táctico real los tiempos de paso por waypoint son inherentemente inciertos — el ANSP no tiene acceso a los datos del FMS — lo que relativiza la ventaja de la optimalidad exacta. Los algoritmos genéticos como DEAP son la alternativa más utilizada en la industria ATM por su capacidad de escalar sin requerir formulación matemática cerrada.

## Uso

### Requisitos

```
numpy
deap          # para el algoritmo genético
gurobipy      # para el solver MIP (requiere licencia Gurobi)
```

### Ejecución

```bash
# Algoritmo genético (sin licencia comercial)
python opti-TRY-DEAP.py

# Gurobi con datos reales de NEST
python opti-TRY-GUROBI-defined.py

# Gurobi con tiempos aleatorios
python opti-TRY-GUROBI-random.py
```

> **Nota:** Gurobi requiere una [licencia](https://www.gurobi.com/academia/academic-program-and-licenses/) (gratuita para uso académico). DEAP se ejecuta sin dependencias comerciales.

## Estructura del proyecto

```
atm-trajectory-optimization/
├── opti-TRY-DEAP.py             # Algoritmo genético
├── opti-TRY-GUROBI-defined.py   # Gurobi con datos NEST + Haversine
├── opti-TRY-GUROBI-random.py    # Gurobi con tiempos aleatorios
├── requirements.txt
├── LICENSE
└── README.md
```

## Contexto

Proyecto desarrollado en la asignatura de **Predicción, Sincronización y Optimización de Trayectorias** del Máster en Sistemas del Transporte Aéreo, ETSIAE — Universidad Politécnica de Madrid. 

Los datos operacionales proceden de EUROCONTROL **NEST** y la herramienta **Insignia** de ENAIRE.

## Licencia

[MIT](LICENSE)
