import gurobipy as gp
from gurobipy import GRB
import numpy as np

# Datos de entrada
n = 8  # CAMBIO: 8 aeronaves reales identificadas en LECMBLU entre 11:09-11:19. Fuente: Entry List NEST 24/08/2025.

original_entry_times = np.array([16.167, 11.267, 10.817, 15.850, 13.417, 11.000, 14.183, 9.000])  # CAMBIO: tiempos reales de entrada al sector en minutos desde las 11:00. Rango: 9.0 (N251SB, 11:09) a 16.17 (DAH2700, 11:16). Orden: DAH2700, TLJ263E, EXS2UY, TAP642, RYR8605, NJE299D, SCO282, N251SB. Fuente: Entry List NEST.

flight_durations = np.array([3.13, 18.55, 14.42, 18.85, 14.08, 12.10, 9.72, 8.0])  # CAMBIO: duraciones reales de vuelo en el sector en minutos. Calculadas como Cross_duration(s)/60. Fuente: Entry List NEST.

waypoints = 7  # CAMBIO: 7 waypoints reales de LECMBLU. Orden: BLV(0), BISKA(1), BADRU(2), NUBLO(3), RONSI(4), DELOG(5), BELEN(6). Fuente: Flight Route Viewer NEST.

# CAMBIO: tiempos de paso por waypoint en minutos desde la entrada al sector.
# Fuente: Flight Route Viewer NEST para tiempos disponibles directamente.
# Para TLJ263E→DELOG, TAP642→DELOG, SCO282→BELEN y todos los de N251SB:
# calculados mediante formula Haversine con coordenadas NEST, dado que el
# predictor de trayectorias del ANSP no tiene acceso a los datos precisos del FMS.
# Valor 0 indica que la aeronave no transita ese waypoint (ver waypoint_usage).
# Orden filas: DAH2700, TLJ263E, EXS2UY, TAP642, RYR8605, NJE299D, SCO282, N251SB.
# Orden cols:   BLV     BISKA   BADRU   NUBLO   RONSI   DELOG   BELEN
waypoint_times = np.array([
    [2.950, 0.000, 0.000,  0.000,  0.000,  0.000, 0.000],  # DAH2700: BLV de NEST
    [0.000, 0.000, 0.000,  2.833, 10.633, 18.550, 0.000],  # TLJ263E: NUBLO y RONSI de NEST, DELOG Haversine
    [0.000, 0.000, 0.000,  0.000,  0.000, 14.417, 0.000],  # EXS2UY:  DELOG de NEST
    [0.000, 0.000, 0.000,  2.933, 10.733, 18.850, 0.000],  # TAP642:  NUBLO y RONSI de NEST, DELOG Haversine
    [0.000, 0.000, 0.000,  0.000,  0.000, 14.083, 0.000],  # RYR8605: DELOG de NEST
    [0.000, 0.000, 0.000,  0.000,  0.000,  0.000, 0.000],  # NJE299D: entra al sector en DELOG, offset = 0
    [4.283, 7.617, 8.083,  0.000,  0.000,  0.000, 9.717],  # SCO282:  BLV, BISKA, BADRU de NEST, BELEN Haversine
    [3.000, 6.000, 7.000,  0.000,  0.000,  0.000, 8.500],  # N251SB:  todos Haversine
])

# CAMBIO: matriz de conectividad aeronave x waypoint.
# Valor 1: la aeronave transita ese waypoint dentro de LECMBLU. Valor 0: no lo transita.
# Fuente: Flight Route Viewer NEST. Orden filas igual que waypoint_times.
waypoint_usage = np.array([
    [1, 0, 0, 0, 0, 0, 0],  # DAH2700: BLV
    [0, 0, 0, 1, 1, 1, 0],  # TLJ263E: NUBLO, RONSI, DELOG
    [0, 0, 0, 0, 0, 1, 0],  # EXS2UY:  DELOG
    [0, 0, 0, 1, 1, 1, 0],  # TAP642:  NUBLO, RONSI, DELOG
    [0, 0, 0, 0, 0, 1, 0],  # RYR8605: DELOG
    [0, 0, 0, 0, 0, 1, 0],  # NJE299D: DELOG (entra al sector en este punto)
    [1, 1, 1, 0, 0, 0, 1],  # SCO282:  BLV, BISKA, BADRU, BELEN
    [1, 1, 1, 0, 0, 0, 1],  # N251SB:  BLV, BISKA, BADRU, BELEN
])

# Parámetros para la optimización multiobjetivo
alpha = 1.0  # CAMBIO: alpha=1.0 porque el único objetivo es minimizar desviaciones. Los conflictos se eliminan mediante hard constraints, no mediante la función objetivo.
beta = 1 - alpha  # Sin cambio estructural. Con alpha=1.0 resulta beta=0.0, el término de balanceo de carga no contribuye al objetivo.
time_intervals = 10  # CAMBIO: 10 minutos para cubrir el periodo real 11:09-11:19 (minutos 9 a 19 desde las 11:00). Ajustado desde 11:10-11:20 porque N251SB entra al sector a las 11:09. Anexo original: 60.

# Crear el modelo
model = gp.Model("AircraftSchedulingMultiObjective")

# Variables de decisión
entry_times = model.addVars(n, lb=9, ub=19, vtype=GRB.CONTINUOUS, name="EntryTimes")  # CAMBIO: lb=9 (11:09, entrada N251SB), ub=19 (11:19, fin del periodo ajustado). CONTINUOUS para permitir decimales coherentes con los tiempos reales.

# Variables auxiliares para la función objetivo (valor absoluto)
deviation_plus  = model.addVars(n, lb=0, vtype=GRB.CONTINUOUS, name="DeviationPlus")   # CAMBIO: CONTINUOUS por coherencia con entry_times.
deviation_minus = model.addVars(n, lb=0, vtype=GRB.CONTINUOUS, name="DeviationMinus")  # CAMBIO: CONTINUOUS por coherencia con entry_times.

# Variables para controlar el orden de llegada a los waypoints
is_before = model.addVars([(i, j, k) for i in range(n) for j in range(n) if i != j
                            for k in range(waypoints) if waypoint_usage[i][k] and waypoint_usage[j][k]],
                           vtype=GRB.BINARY, name="IsBefore")  # Sin cambio.

# Variables para contar aeronaves en cada intervalo de tiempo
aircraft_count = model.addVars(time_intervals, lb=0, ub=8, vtype=GRB.CONTINUOUS, name="AircraftCount")  # CAMBIO: ub=8 capacidad máxima real de LECMBLU (8 aeronaves/10 min). Fuente: Trabajo 2. CONTINUOUS por coherencia.

# Variables para medir la desviación de la ocupación ideal
occupancy_deviation = model.addVars(time_intervals, lb=0, vtype=GRB.CONTINUOUS, name="OccupancyDeviation")  # CAMBIO: CONTINUOUS por coherencia.

# Restricciones para calcular la desviación de tiempos de entrada
for i in range(n):
    model.addConstr(entry_times[i] - original_entry_times[i] == deviation_plus[i] - deviation_minus[i])  # Sin cambio.

# Restricción de separación en waypoints
for i in range(n):
    for j in range(i+1, n):
        for k in range(waypoints):
            if waypoint_usage[i][k] and waypoint_usage[j][k]:
                arrival_i_k = entry_times[i] + waypoint_times[i][k]  # Sin cambio estructural. Usa waypoint_times de NEST y Haversine.
                arrival_j_k = entry_times[j] + waypoint_times[j][k]  # Sin cambio estructural. Usa waypoint_times de NEST y Haversine.
                model.addConstr((is_before[i, j, k] == 1) >> (arrival_i_k + 2 <= arrival_j_k))  # Sin cambio.
                model.addConstr((is_before[i, j, k] == 0) >> (arrival_j_k + 2 <= arrival_i_k))  # Sin cambio.

# Calcular el número de aeronaves en cada intervalo de tiempo
for t in range(time_intervals):
    in_sector = gp.quicksum(  # CAMBIO: se corrige el bug del Anexo original. El operador & de Python no es válido en Gurobi. Se sustituye por gp.quicksum con expresión lineal equivalente.
        1 for i in range(n)
        if original_entry_times[i] <= t + 9 <= original_entry_times[i] + flight_durations[i]  # CAMBIO: t+9 para alinear el índice del bucle con los minutos reales desde las 11:00.
    )
    model.addConstr(aircraft_count[t] == in_sector)  # Sin cambio estructural.
    model.addConstr(aircraft_count[t] <= 8)  # CAMBIO: capacidad máxima real de LECMBLU = 8 aeronaves. Fuente: Trabajo 2.

    ideal_occupancy = 4  # CAMBIO: ocupación ideal = 4 (mitad de la capacidad máxima de 8). Con beta=0 este término no afecta al objetivo.
    model.addConstr(occupancy_deviation[t] >= aircraft_count[t] - ideal_occupancy)  # Sin cambio.
    model.addConstr(occupancy_deviation[t] >= ideal_occupancy - aircraft_count[t])  # Sin cambio.

# Función objetivo multiobjetivo:
# 1. Minimizar desviaciones de tiempos originales
schedule_deviation = gp.quicksum(deviation_plus[i] + deviation_minus[i] for i in range(n))  # Sin cambio.

# 2. Minimizar desviaciones de la ocupación ideal (balancear carga)
load_balancing = gp.quicksum(occupancy_deviation[t] for t in range(time_intervals))  # Sin cambio.

# Función objetivo combinada con pesos
model.setObjective(alpha * schedule_deviation + beta * load_balancing, GRB.MINIMIZE)  # Sin cambio estructural. Con alpha=1.0 y beta=0.0 solo minimiza desviaciones.

# Optimizar
model.optimize()

# Mostrar resultados
callsigns = ['DAH2700', 'TLJ263E', 'EXS2UY', 'TAP642', 'RYR8605', 'NJE299D', 'SCO282', 'N251SB']  # CAMBIO: lista de callsigns reales para identificar cada aeronave en los resultados.

if model.status == GRB.OPTIMAL:
    print("Solución óptima encontrada:")
    print("Aeronave | Tiempo Original | Tiempo Reprogramado | Desviación")
    for i in range(n):
        print(f"{callsigns[i]:8s} | {original_entry_times[i]:14.3f} | {entry_times[i].X:19.3f} | {abs(entry_times[i].X - original_entry_times[i]):10.3f}")  # CAMBIO: muestra callsign real y tiempos con tres decimales.

    print("\nDistribución de carga del espacio aéreo:")
    for t in range(time_intervals):
        print(f"Minuto {t+9:02d} (11:{t+9:02d}): {int(aircraft_count[t].X)} aeronaves")  # CAMBIO: t+9 para mostrar el minuto real desde las 11:00 y la hora UTC correspondiente.

    total_deviation = sum(abs(entry_times[i].X - original_entry_times[i]) for i in range(n))
    avg_occupancy   = sum(aircraft_count[t].X for t in range(time_intervals)) / time_intervals
    max_occupancy   = max(aircraft_count[t].X for t in range(time_intervals))
    min_occupancy   = min(aircraft_count[t].X for t in range(time_intervals))
    occupancy_std   = np.std([aircraft_count[t].X for t in range(time_intervals)])

    print("\nMétricas de la solución:")
    print(f"Desviación total de horarios: {total_deviation:.3f} min")
    print(f"Ocupación media del espacio aéreo: {avg_occupancy:.2f} aeronaves")
    print(f"Ocupación máxima: {max_occupancy:.0f} aeronaves")
    print(f"Ocupación mínima: {min_occupancy:.0f} aeronaves")
    print(f"Desviación estándar de ocupación: {occupancy_std:.2f} (menor valor indica mejor balance)")
else:
    print("No se encontró solución óptima.")