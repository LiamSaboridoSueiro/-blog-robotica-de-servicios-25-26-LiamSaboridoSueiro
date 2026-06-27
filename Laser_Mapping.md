## Índice
<details>
<summary>Práctica 5 - Laser Mapping</summary>

- [Errores comentados en tutoría](#errores-comentados-en-tutoría)
- [Objetivo](#objetivo)
- [Teoría y Funcionamiento](#teoría-y-funcionamiento)
  * [LIDAR y Adquisición de Datos](#lidar-y-adquisición-de-datos)
  * [Mapa de Ocupación Probabilístico](#mapa-de-ocupación-probabilístico)
  * [Implementación Bayesiana con Log-Odds](#implementación-bayesiana-con-log-odds)
  * [Filtrado de Rayos y Observaciones Independientes](#filtrado-de-rayos-y-observaciones-independientes)
  * [Trazado de Láser con Bresenham](#trazado-de-láser-con-bresenham)
  * [Exploración y Navegación](#exploración-y-navegación)
  * [Prueba con getPose3d y getOdom](#prueba-con-getpose3d-y-getodom)
- [Dificultades Encontradas](#dificultades-encontradas)
- [Video](#video)
- [Mapa final](#mapa-final)

</details>


<p align="center">
  <img src="https://github.com/user-attachments/assets/938b747e-b036-4242-87e4-c90aafabc33c" width="900" height="600">
</p>


## Errores comentados en tutoría

Tras la revisión de la práctica, los errores principales estaban relacionados con la calidad del mapa generado. El primero fue que el robot se movía demasiado rápido, lo que hacía que las lecturas del láser se integrasen con poses poco estables y el mapa terminase saliendo torcido. Para corregirlo reduje las velocidades de navegación y separé las velocidades según el modo de movimiento: búsqueda de pared, seguimiento de pared, recuperación y barrido.

El segundo punto fue que no convenía usar todos los rayos del láser en todas las iteraciones, porque muchas observaciones seguidas son casi idénticas y pueden hacer que el mapa se vuelva demasiado rígido o ruidoso. Para solucionarlo añadí un submuestreo con `PASO_LASER = 2`, filtros de lecturas inválidas y un umbral mínimo de movimiento antes de actualizar de nuevo el mapa.

El tercer punto fue implementar realmente un mapa probabilístico usando Bayes. En la versión corregida no guardo simplemente estados discretos de libre/ocupado, sino una rejilla de log-odds (`mapa_log_odds`) que acumula evidencia de ocupación o de espacio libre y se satura para evitar una inercia excesiva.

Además, aunque no fue uno de los errores mencionados directamente en la revisión, aproveché la corrección para mejorar la navegación y hacerla más robusta. La versión final no depende solo de avanzar y girar de forma simple, sino que usa una máquina de estados con búsqueda de pared, seguimiento por la derecha, retorno al origen y barrido por filas. Esto hizo que el recorrido fuese más estable y que el mapa saliese menos deformado.

## Objetivo

El objetivo de esta práctica es diseñar un sistema capaz de explorar un entorno desconocido utilizando un sensor LIDAR y construir simultáneamente un mapa de ocupación probabilístico. El robot debe desplazarse de forma autónoma, detectar obstáculos, actualizar el mapa en tiempo real y tomar decisiones de navegación basadas en la información del láser.

Además, el enunciado propone comprobar cómo cambia la calidad del mapa al usar distintas fuentes de localización. Por eso en el vídeo incluyo una ejecución usando `HAL.getPose3d()` y tres ejecuciones usando `HAL.getOdom()` con distintos niveles de ruido configurados en el mapa: low, medium y high noise.

## Teoría y Funcionamiento

### LIDAR y Adquisición de Datos

El sensor LIDAR mide distancias entre el robot y los obstáculos del entorno. Cada lectura tiene una distancia y un ángulo relativo al robot. A partir de esos datos se calcula la posición del punto detectado en coordenadas locales y después se transforma a coordenadas globales usando la pose actual del robot (`x`, `y`, `yaw`).

En el código, esta conversión se realiza en `procesar_laser()`. Primero se obtiene el ángulo real de cada rayo con `angulo_rayo_laser()`, teniendo en cuenta si el láser devuelve 180, 360 u otro número de medidas. Después se pasa de coordenadas polares a cartesianas:

```python
x_local = distancia * math.cos(angulo_rayo)
y_local = distancia * math.sin(angulo_rayo)
```

Y finalmente se rota y traslada ese punto al sistema global:

```python
x_global = x_rob + x_local * coseno_robot - y_local * seno_robot
y_global = y_rob + x_local * seno_robot + y_local * coseno_robot
```

### Mapa de Ocupación Probabilístico

El mapa se representa como una rejilla de `970 x 1500` celdas, que es el tamaño que espera `WebGUI.setUserMap()`. Aunque visualmente el mapa se ve como una imagen en escala de grises, internamente no trabajo directamente con valores blanco/gris/negro, sino con una matriz probabilística:

```python
mapa_log_odds = np.full((ALTO, ANCHO), LO_INICIAL, dtype=np.float32)
```

Cada celda empieza con incertidumbre total, equivalente a probabilidad `0.5`. Si un rayo del láser atraviesa una celda, esa celda recibe evidencia de espacio libre. Si el rayo termina en una celda, esa celda recibe evidencia de obstáculo. Así, el mapa no depende de una sola observación, sino de la acumulación de muchas observaciones mientras el robot se mueve.

Para mostrar el mapa en la interfaz, convierto los log-odds de nuevo a probabilidad y después a escala de grises:

```python
probabilidad_ocupacion = 1.0 / (1.0 + np.exp(-mapa_log_odds))
return ((1.0 - probabilidad_ocupacion) * 255).astype(np.uint8)
```

Con esta conversión, las zonas libres tienden al blanco, las zonas desconocidas quedan en gris y los obstáculos tienden al negro.

### Implementación Bayesiana con Log-Odds

Para implementar Bayes uso la forma log-odds, que permite acumular evidencias mediante sumas. En vez de recalcular la probabilidad completa de cada celda en cada observación, sumo una evidencia positiva si el láser detecta ocupación y una evidencia negativa si detecta espacio libre:

```python
LO_INICIAL = 0.0
LO_LIBRE = math.log(0.3 / 0.7)
LO_OCUPADO = math.log(0.7 / 0.3)
```

Cuando el rayo atraviesa una celda, actualizo esa celda como libre:

```python
mapa_log_odds[mapa_y, mapa_x] = max(
    LO_MINIMO,
    mapa_log_odds[mapa_y, mapa_x] + LO_LIBRE
)
```

Cuando el rayo termina en un obstáculo, actualizo la celda final como ocupada:

```python
mapa_log_odds[obstaculo_y, obstaculo_x] = min(
    LO_MAXIMO,
    mapa_log_odds[obstaculo_y, obstaculo_x] + LO_OCUPADO
)
```

También uso saturación con `LO_MINIMO` y `LO_MAXIMO`. Esto es importante porque, si una celda acumula demasiada confianza, luego sería muy difícil corregirla aunque lleguen observaciones nuevas. Con la saturación evito que el mapa se vuelva excesivamente inercial.

### Filtrado de Rayos y Observaciones Independientes

Una de las correcciones importantes fue no procesar todos los puntos del láser sin control. En `procesar_laser()` uso `PASO_LASER = 2`, de forma que proceso un rayo de cada dos. Esto reduce ruido y coste de cálculo, pero mantiene suficiente información para construir el mapa.

También descarto medidas inválidas:

```python
if math.isinf(distancia) or math.isnan(distancia):
    continue
if distancia <= rango_minimo or distancia >= distancia_maxima_util:
    continue
```

Además, solo actualizo el mapa si el robot se ha movido lo suficiente desde la última actualización:

```python
return distancia_desde_ultima_pose >= DIST_MIN or giro_desde_ultima_pose >= ANG_MIN
```

En mi caso uso `DIST_MIN = 0.10` metros y `ANG_MIN = 0.05` radianes. Esto evita meter muchas observaciones casi repetidas desde la misma pose, que podrían sobrecargar la evidencia bayesiana sin aportar información nueva.

### Trazado de Láser con Bresenham

Para saber qué celdas atraviesa cada rayo del LIDAR, utilizo el algoritmo de Bresenham. Primero convierto la posición del robot y el punto final del rayo a píxeles del mapa mediante `WebGUI.poseToMap()`. Después genero la línea entre ambos puntos:

```python
celdas_rayo = linea_bresenham(robot_columna, robot_fila, obstaculo_columna, obstaculo_fila)
```

Las celdas intermedias del rayo se actualizan como libres, porque el láser ha pasado por ellas antes de impactar. La celda final se actualiza como ocupada:

```python
for columna, fila in celdas_rayo[:-2]:
    actualizar_log_libre(mapa_log_odds, columna, fila)

actualizar_log_ocupado(mapa_log_odds, obstaculo_columna, obstaculo_fila)
```

Este paso es esencial para que el mapa no solo marque paredes, sino también el espacio libre que el robot ha observado.

### Exploración y Navegación

La navegación final no es una exploración aleatoria simple. Implementé una máquina de estados más estructurada para conseguir recorridos lentos y repetibles, porque con movimientos rápidos el mapa se deformaba bastante.

El flujo principal empieza orientando el robot hacia la derecha del mapa y buscando una pared. Cuando detecta una pared, gira para seguirla por la derecha. En el estado `FOLLOW_WALL`, el robot mantiene una distancia objetivo respecto a la pared usando sectores del láser: frontal, derecha y derecha-delante. Si encuentra una esquina, gira; si pierde la pared, se acerca de nuevo suavemente.

También guardo una referencia al empezar a seguir la pared. Cuando el robot se aleja de ese punto y más tarde vuelve a estar cerca, considero que ha cerrado una vuelta aproximada al perímetro. Entonces el robot vuelve al origen y arranca un barrido inicial por filas, con pasadas horizontales y desplazamientos hacia abajo. Esta parte está formada por estados como `PRE_SWEEP_ALIGN_TOP`, `PRE_SWEEP_FORWARD`, `PRE_SWEEP_ALIGN_DOWN` y `PRE_SWEEP_DISPLACE_DOWN`.

Las velocidades están limitadas para mejorar la calidad del mapa. Algunos valores usados son:

```python
V_BUSCAR_PARED = 0.22
V_SEGUIR_PARED = 0.17
V_RECUPERAR_PARED = 0.12
V_BARRIDO_INICIAL = 0.20
V_DESPLAZAMIENTO_INICIAL = 0.16
```

Esta reducción de velocidad fue una de las correcciones más importantes, porque el mapa depende mucho de que la pose del robot y las lecturas del láser estén bien sincronizadas.

En la versión con odometría ruidosa separo la pose usada para mapear de la pose usada para navegar. El gridmap se actualiza con `HAL.getOdom()`, pero la máquina de estados recibe también `HAL.getPose3d()` para mantener el recorrido estable y poder comparar mejor los mapas resultantes:

```python
odometria = HAL.getOdom()
pose_real = HAL.getPose3d()
lecturas_laser = HAL.getLaserData()

maquina_estados(lecturas_laser, pose_real)
```

### Prueba con getPose3d y getOdom

Para tener una referencia limpia, primero pruebo el algoritmo usando `HAL.getPose3d()` para obtener la pose del robot durante el mapeo:

```python
pose_mapeo = HAL.getPose3d()
```

Esta pose es más precisa y permite construir un mapa más limpio. Después pruebo el mismo algoritmo usando `HAL.getOdom()` para insertar los rayos en el mapa:

```python
odometria = HAL.getOdom()
```

En las pruebas con odometría, el nivel de ruido se cambia cargando mapas/configuraciones distintas del ejercicio. Por eso los resultados se comparan como `GetOdom Low Noise`, `GetOdom Medium Noise` y `GetOdom High Noise`.

Esta comparación ayuda a ver una limitación importante del mapeo con posición conocida: el algoritmo de ocupación puede estar bien implementado, pero si la pose usada para insertar los rayos no es buena, el mapa final pierde calidad. A medida que aumenta el ruido en `getOdom`, las paredes aparecen más desplazadas o torcidas porque pequeños errores de posición y orientación se acumulan al proyectar los rayos del láser sobre el gridmap.

## Dificultades Encontradas

Una de las principales dificultades fue ejecutar y grabar la práctica en mi ordenador, porque la simulación iba bastante justa de rendimiento. Al moverse demasiado rápido, el robot generaba mapas torcidos, así que tuve que bajar velocidades y hacer la navegación más suave.

También fue complicado ajustar el mapa probabilístico. Si procesaba demasiados rayos o actualizaba muchas veces desde la misma posición, algunas zonas acumulaban demasiada confianza demasiado pronto. La solución fue combinar submuestreo del láser, umbrales de movimiento y saturación en los log-odds.

Otra dificultad fue validar el trazado con Bresenham. Un pequeño error al transformar coordenadas del mundo a píxeles del mapa hace que las paredes aparezcan desplazadas, así que tuve que depurar la conversión con `WebGUI.poseToMap()` y comprobar que las celdas libres y ocupadas se actualizaban en el lugar correcto.

Por último, la prueba con `getOdom` dejó claro que el mapeo depende muchísimo de la localización. Con `getPose3d` el mapa queda más consistente; con odometría ruidosa se nota el drift y las paredes pierden alineación, especialmente en los casos medium y high noise.

## Video

El vídeo nuevo incluye las cuatro formas de probar el código del ejercicio:

- [00:06 - GetPose3D](https://www.youtube.com/watch?v=6Oi5-tQqboQ&t=6s)
- [01:50 - GetOdom Low Noise](https://www.youtube.com/watch?v=6Oi5-tQqboQ&t=110s)
- [03:40 - GetOdom Medium Noise](https://www.youtube.com/watch?v=6Oi5-tQqboQ&t=220s)
- [05:30 - GetOdom High Noise](https://www.youtube.com/watch?v=6Oi5-tQqboQ&t=330s)

[![Vídeo Laser Mapping](https://img.youtube.com/vi/6Oi5-tQqboQ/0.jpg)](https://www.youtube.com/watch?v=6Oi5-tQqboQ)

## Mapa final

En las siguientes imágenes se ven los resultados guardados en la carpeta `resources/resultados_P5` del blog. La comparación muestra cómo el mapa se degrada progresivamente al aumentar el ruido de la odometría.

<p align="center">
  <img src="resources/resultados_P5/Getpose3d.png" width="450">
  <br>
  <em>Resultado usando HAL.getPose3d().</em>
</p>

<p align="center">
  <img src="resources/resultados_P5/GetOdom_low_noise.png" width="450">
  <br>
  <em>Resultado usando HAL.getOdom() con low noise.</em>
</p>

<p align="center">
  <img src="resources/resultados_P5/GetOdom_medium_noise.png" width="450">
  <br>
  <em>Resultado usando HAL.getOdom() con medium noise.</em>
</p>

<p align="center">
  <img src="resources/resultados_P5/GetOdom_high_noise.png" width="450">
  <br>
  <em>Resultado usando HAL.getOdom() con high noise.</em>
</p>
