import numpy as np
import random
# import string  # CAMBIO: eliminado. Se usaba exclusivamente para generar nombres aleatorios de waypoints ficticios, innecesario al trabajar con waypoints reales de NEST.
from deap import base, creator, tools, algorithms

# Parámetros
num_aeronaves = 8  # CAMBIO: 8 aeronaves reales identificadas en LECMBLU entre 11:09-11:19. Fuente: Entry List NEST 24/08/2025.
# num_aerovias = 5        # CAMBIO: eliminado. Parámetro de generación aleatoria de aerovías, innecesario al usar waypoint_usage real.
# min_velocidad = 400     # CAMBIO: eliminado. Parámetro de generación aleatoria de velocidades, innecesario al usar flight_durations reales.
# max_velocidad = 500     # CAMBIO: eliminado. Mismo motivo que min_velocidad.
# min_waypoints = 3       # CAMBIO: eliminado. Parámetro de generación aleatoria de waypoints por aerovía, innecesario al usar waypoint_usage real.
# max_waypoints = 5       # CAMBIO: eliminado. Mismo motivo que min_waypoints.
# min_separacion = 5      # CAMBIO: eliminado. Parámetro de generación aleatoria de distancias entre waypoints, innecesario al usar waypoint_times.
# max_separacion = 15     # CAMBIO: eliminado. Mismo motivo que min_separacion.
periodo_estudio = 10  # CAMBIO: 10 minutos para cubrir el periodo real 11:09-11:19. Ajustado desde 11:10-11:20 porque N251SB entra al sector a las 11:09. Original: 15.
max_ocupacion = 8  # CAMBIO: capacidad máxima real de LECMBLU = 8 aeronaves/10 min. Fuente: Trabajo 2. Original: 5.

# CAMBIO: se eliminan las siguientes funciones y estructuras de generación aleatoria:
# generar_nombre_waypoint(), waypoints_globales, aerovias (generación aleatoria),
# velocidades (aleatoria), asignaciones_aerovias.
# Se sustituyen por waypoint_usage y waypoint_times reales de NEST,
# y por flight_durations reales de la Entry List de NEST.

# CAMBIO: tiempos reales de entrada al sector en minutos desde las 11:00.
# Orden: DAH2700, TLJ263E, EXS2UY, TAP642, RYR8605, NJE299D, SCO282, N251SB.
# Fuente: Entry List NEST 24/08/2025.
tiempos_entrada_originales = [16.167, 11.267, 10.817, 15.850, 13.417, 11.000, 14.183, 9.000]

# CAMBIO: duraciones reales de vuelo en el sector en minutos.
# Calculadas como Cross_duration(s)/60. Fuente: Entry List NEST.
# Orden: DAH2700, TLJ263E, EXS2UY, TAP642, RYR8605, NJE299D, SCO282, N251SB.
flight_durations = [3.13, 18.55, 14.42, 18.85, 14.08, 12.10, 9.72, 8.0]

# CAMBIO: número de waypoints reales de LECMBLU.
# Orden: BLV(0), BISKA(1), BADRU(2), NUBLO(3), RONSI(4), DELOG(5), BELEN(6).
# Fuente: Flight Route Viewer NEST.
num_waypoints = 7

# CAMBIO: tiempos de paso por waypoint en minutos desde la entrada al sector.
# Se mantienen aleatorios dentro del rango operacional plausible dado que el
# predictor de trayectorias del ANSP no tiene acceso a los datos precisos del FMS.
# Valor 0 en posiciones donde la aeronave no transita ese waypoint (ver waypoint_usage).
waypoint_times = np.random.uniform(2, 30, (num_aeronaves, num_waypoints))

# CAMBIO: matriz de conectividad aeronave x waypoint.
# Valor 1: la aeronave transita ese waypoint dentro de LECMBLU. Valor 0: no lo transita.
# Fuente: Flight Route Viewer NEST. Orden filas: DAH2700, TLJ263E, EXS2UY, TAP642, RYR8605, NJE299D, SCO282, N251SB.
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

# CAMBIO: callsigns reales para identificar cada aeronave en los resultados.
callsigns = ['DAH2700', 'TLJ263E', 'EXS2UY', 'TAP642', 'RYR8605', 'NJE299D', 'SCO282', 'N251SB']

# Función para calcular la ocupación
def calcular_ocupacion(tiempos_entrada):
    ocupacion = np.zeros(periodo_estudio + 1, dtype=int)
    for i in range(num_aeronaves):
        entrada = tiempos_entrada[i] - 9  # CAMBIO: offset de 9 para alinear con el inicio del periodo (minuto 9 = 11:09).
        salida = entrada + flight_durations[i]  # CAMBIO: usa flight_durations reales en lugar de calcular desde distancias aleatorias.
        for t in range(periodo_estudio + 1):
            if entrada <= t <= salida:  # CAMBIO: condición continua en lugar de índices enteros, coherente con tiempos reales con decimales.
                ocupacion[t] += 1
    return ocupacion

# Calcular la ocupación inicial
ocupacion_inicial = calcular_ocupacion(tiempos_entrada_originales)
# print("Ocupación inicial por minuto:")
# print(ocupacion_inicial)
# print("Momentos de sobreocupación inicial:", np.where(ocupacion_inicial > max_ocupacion)[0])

# Función para calcular conflictos en waypoints
def calcular_conflictos(tiempos_entrada):
    conflictos = 0
    for i in range(num_aeronaves):
        for j in range(i + 1, num_aeronaves):
            for k in range(num_waypoints):
                if waypoint_usage[i][k] and waypoint_usage[j][k]:  # CAMBIO: usa waypoint_usage real en lugar de recorrer estructura de aerovías aleatoria.
                    tiempo_wp_i = tiempos_entrada[i] + waypoint_times[i][k]  # CAMBIO: tiempo de llegada al waypoint usando waypoint_times.
                    tiempo_wp_j = tiempos_entrada[j] + waypoint_times[j][k]  # CAMBIO: tiempo de llegada al waypoint usando waypoint_times.
                    if abs(tiempo_wp_i - tiempo_wp_j) < 2:  # CAMBIO: < 2 en lugar de <= 2 para alinear con el estándar operacional real y con Gurobi. Separación exacta de 2 minutos es válida.
                        conflictos += 1
    return conflictos

# Calcular ocupación y conflictos iniciales
ocupacion_inicial = calcular_ocupacion(tiempos_entrada_originales)
conflictos_iniciales = calcular_conflictos(tiempos_entrada_originales)
print("Ocupación inicial por minuto:")
print(ocupacion_inicial)
print("Momentos de sobreocupación inicial:", np.where(ocupacion_inicial > max_ocupacion)[0])
print("Conflictos iniciales en waypoints:", conflictos_iniciales)

# Función de evaluación
def evaluar(individual):
    tiempos_entrada_optimizados = individual
    ocupacion = calcular_ocupacion(tiempos_entrada_optimizados)

    # Penalización por sobreocupación (muy alta para evitar soluciones inválidas)
    penalizacion_ocupacion = 10000 * np.sum(ocupacion > max_ocupacion)

    # Calcular la desviación absoluta de cada aeronave
    desviaciones = [abs(tiempos_entrada_optimizados[i] - tiempos_entrada_originales[i]) for i in range(num_aeronaves)]
    suma_desviaciones = np.sum(desviaciones)

    # Penalización por conflictos en waypoints
    conflictos = calcular_conflictos(tiempos_entrada_optimizados)
    penalizacion_conflictos = 10000 * conflictos

    # Función objetivo combinada
    return (penalizacion_ocupacion + penalizacion_conflictos + suma_desviaciones,)

# Configuración del algoritmo genético
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, 9, 19)  # CAMBIO: rango 9-19 (minutos 11:09-11:19) en lugar de 0-10. Cambiado de randint a uniform para permitir valores continuos coherentes con los tiempos reales.
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=num_aeronaves)  # Sin cambio estructural.
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("evaluate", evaluar)
toolbox.register("mate", tools.cxTwoPoint)
def mutar_continuo(individual, indpb):  # CAMBIO: función de mutación continua personalizada. Sustituye tools.mutUniformInt que generaba valores enteros, inconsistente con la inicialización continua mediante random.uniform. Mantiene la precisión decimal de los tiempos a lo largo de todo el proceso evolutivo.
    for i in range(len(individual)):
        if random.random() < indpb:
            individual[i] = random.uniform(9, 19)  # CAMBIO: rango 9-19 coherente con el periodo de estudio 11:09-11:19 y con attr_float.
    return individual,

toolbox.register("mutate", mutar_continuo, indpb=0.1)  # CAMBIO: registra la mutación continua personalizada manteniendo indpb=0.1 del original.
toolbox.register("select", tools.selTournament, tournsize=3)

# Parámetros del algoritmo genético
population_size = 50
num_generations = 100
cx_prob = 0.7
mut_prob = 0.2

# Crear la población inicial
population = toolbox.population(n=population_size)

# Ejecutar el algoritmo genético
algorithms.eaSimple(
    population,
    toolbox,
    cxpb=cx_prob,
    mutpb=mut_prob,
    ngen=num_generations,
    verbose=False
)

# Obtener la mejor solución
best_individual = tools.selBest(population, k=1)[0]
best_ocupacion = calcular_ocupacion(best_individual)
best_conflictos = calcular_conflictos(best_individual)

# Calcular las desviaciones de la mejor solución
desviaciones = [abs(best_individual[i] - tiempos_entrada_originales[i]) for i in range(num_aeronaves)]
suma_desviaciones = np.sum(desviaciones)

# Resultados
print("\nSolución óptima encontrada:")
print(f"{'Aeronave':<10} {'T.Original':>10} {'T.Optimizado':>13} {'Desviación':>11}")
for i in range(num_aeronaves):
    desv = best_individual[i] - tiempos_entrada_originales[i]
    print(f"{callsigns[i]:<10} {tiempos_entrada_originales[i]:>10.3f} {best_individual[i]:>13.3f} {desv:>+11.3f}")  # CAMBIO: muestra callsign real y tiempos con tres decimales.

print("\nOcupación optimizada por minuto:")
for t in range(periodo_estudio + 1):
    print(f"Minuto {t+9:02d} (11:{t+9:02d}): {best_ocupacion[t]} aeronaves")  # CAMBIO: t+9 para mostrar el minuto real desde las 11:00 y la hora UTC correspondiente.

print("Momentos de sobreocupación optimizada:", np.where(best_ocupacion > max_ocupacion)[0])
print("\nConflictos en waypoints optimizados:", best_conflictos)
print(f"\nDesviación total de horarios: {suma_desviaciones:.3f} min")