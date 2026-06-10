
## Índice
<details>
<summary>Práctica 6</summary>

- [Errores comentados en tutoría](#errores-comentados-en-tutoría)
- [Objetivo](#objetivo)
  * [Detección de Marcadores AprilTag](#detección-de-marcadores-apriltag)
  * [Estimación de Pose 3D mediante PnP](#estimación-de-pose-3d-mediante-pnp)
  * [Transformación al Sistema de Coordenadas del Robot](#transformación-al-sistema-de-coordenadas-del-robot)
  * [Proyección a Coordenadas Globales del Mapa](#proyección-a-coordenadas-globales-del-mapa)
  * [Selección de la Baliza más Cercana](#selección-de-la-baliza-más-cercana)
  * [Fusión con Odometría Incremental](#fusión-con-odometría-incremental)
  * [Navegación por Tramos y Giros](#navegación-por-tramos-y-giros)
  * [Bucle Principal de Localización](#bucle-principal-de-localización)
- [Dificultades y Decisiones de Diseño](#dificultades-y-decisiones-de-diseño)
- [Video](#video)

</details>

## Práctica 6 - Marker Visual Loc


<p align="center">
  <img src="https://github.com/user-attachments/assets/1ad29982-f42d-4c97-8db9-21566723ba46" width="900" height="600">
</p>


## Errores comentados en tutoría

Tras la revisión de la práctica, el principal error detectado fue que el robot no seleccionaba correctamente el QR/AprilTag más cercano cuando había varias balizas visibles a la vez. El enunciado indica que, en ese caso, conviene quedarse con la estimación obtenida desde la baliza más cercana, ya que será la que aparece más grande en la imagen y normalmente proporciona una medida más fiable.

Para corregirlo, modifiqué el criterio de selección: ahora el programa procesa todas las detecciones visibles, calcula con PnP la pose relativa de cada marcador y guarda también su distancia relativa al robot. Después, la función `seleccionar_baliza()` elige la detección con menor distancia euclídea relativa, usando `x_rel` e `y_rel`:

```python
mejor = min(lista_poses, key=lambda p: p[4]**2 + p[5]**2)
```

También se comentó que el vídeo debía mostrar mejor la ejecución. Por eso he sustituido el vídeo anterior por uno nuevo que incluye 4 ejemplos de ejecución, para que se vea con más claridad el comportamiento del robot en distintas situaciones.

## Objetivo

El objetivo de esta práctica es implementar un sistema de localización visual para un robot móvil utilizando marcadores AprilTag distribuidos por el entorno. El robot debe ser capaz de estimar su posición y orientación en un mapa conocido mediante la detección de estos marcadores, fusionando esta información con datos odométricos para mantener una estimación precisa incluso cuando pierde de vista las balizas. Además, el robot debe navegar de forma autónoma explorando el entorno mientras actualiza continuamente su posición.

La idea es combinar visión por computador para detectar marcadores fiduciales, geometría 3D para calcular poses relativas, y un sistema de fusión de sensores que combine la información visual con la odometría del robot. Todo esto coordinado mediante una navegación por estados sencilla, basada en avanzar una distancia fija y después realizar un giro controlado.

### Detección de Marcadores AprilTag

Los AprilTag son marcadores visuales similares a códigos QR, pero diseñados específicamente para robótica. Cada marcador tiene un patrón único que permite identificarlo y calcular su posición en 3D a partir de una imagen 2D. En mi implementación, utilicé la librería `pyapriltags` que se encarga de detectar estos marcadores en las imágenes capturadas por la cámara del robot.

El proceso comienza convirtiendo la imagen en color a escala de grises, ya que los marcadores funcionan mejor con imágenes en blanco y negro donde el contraste es máximo. Una vez procesada la imagen, el detector devuelve una lista de todos los marcadores encontrados, cada uno con su ID único y las coordenadas de sus cuatro esquinas en píxeles. Esta información es fundamental porque las esquinas son la entrada que se utiliza después en `solvePnP()` para calcular la orientación y distancia del marcador respecto a la cámara.

Para visualizar las detecciones, dibujo un recuadro verde alrededor de cada marcador identificado, marco su centro y escribo información de depuración sobre la imagen. Además, en cada marcador se muestra la distancia estimada, lo que permite comprobar visualmente cuál debería ser seleccionado cuando aparecen varios a la vez.

### Estimación de Pose 3D mediante PnP

Una vez detectadas las esquinas de un marcador en la imagen, el siguiente paso es determinar dónde está ese marcador en el espacio 3D respecto a la cámara. Para esto utilizo el algoritmo **PnP (Perspective-n-Point)** mediante `cv2.solvePnP()` con el flag `cv2.SOLVEPNP_IPPE_SQUARE`, que está pensado específicamente para marcadores planos cuadrados. La idea básica es: si conozco las dimensiones reales del marcador, 0.24 metros de lado para la zona negra utilizada en el ejercicio, y veo cómo aparece en la imagen, puedo calcular su posición y orientación en el espacio.

El PnP necesita dos cosas: los puntos 3D del marcador en su propio sistema de coordenadas y los puntos 2D correspondientes en la imagen. En el código genero las cuatro esquinas 3D del cuadrado centradas en el marcador, usando `radio_marcador = 0.12`, y las comparo con las esquinas detectadas por `pyapriltags`. Este orden de esquinas encaja con `SOLVEPNP_IPPE_SQUARE`, y con esta información `solvePnP()` devuelve un vector de rotación y un vector de traslación.

La matriz intrínseca de la cámara se construye a partir del tamaño de la imagen: uso como distancia focal el ancho de la imagen, el centro óptico en el centro del frame y coeficientes de distorsión nulos. Con esto se obtiene una estimación suficientemente buena para transformar las esquinas 2D en una pose 3D relativa.

### Transformación al Sistema de Coordenadas del Robot

El PnP me da la pose del marcador respecto a la **cámara**, pero lo que realmente necesito es saber dónde está el **robot** respecto al marcador. Para hacer ese cambio de referencia, convierto primero el vector de rotación en una matriz de rotación con `cv2.Rodrigues()`.

Después invierto la transformación: si `solvePnP()` me da la rotación y traslación del marcador respecto a la cámara, yo calculo la cámara respecto al marcador transponiendo la matriz de rotación y recalculando el vector de traslación:

```python
matriz_rot_tag_cam = matriz_rot_cam_tag.T
vector_tras_tag_cam = -matriz_rot_tag_cam @ vector_tras_cam_tag
```

De esa transformación extraigo tres valores relativos: `x_rel`, `y_rel` y `ang_rel`. En mi código, `x_rel` se toma desde el eje `Z` de la cámara transformado al sistema del marcador, `y_rel` desde el eje `X` cambiado de signo, y el ángulo se calcula a partir del eje frontal de la cámara. Estos valores son relativos al marcador, no al mundo, pero son el paso previo necesario para calcular la posición global del robot.

### Proyección a Coordenadas Globales del Mapa

Una vez que sé dónde está el robot respecto a un marcador específico, necesito convertir esa información relativa en una posición absoluta dentro del mapa del entorno. Para esto utilizo el archivo YAML del ejercicio, situado en `/resources/exercises/marker_visual_loc/apriltags_poses.yaml`, que contiene las posiciones conocidas de todos los marcadores. Cada marcador tiene almacenadas sus coordenadas `X`, `Y`, `Z` y su orientación `yaw` en el sistema de referencia del mapa.

La idea es sencilla en concepto pero requiere cuidado en la implementación: si sé dónde está el marcador en el mundo y sé dónde estoy yo respecto al marcador, puedo calcular dónde estoy en el mundo. Matemáticamente esto implica una **rotación 2D** seguida de una **traslación**. Primero roto las coordenadas relativas del robot según la orientación del marcador en el mapa, y luego las sumo a la posición del marcador. Es como si el marcador fuera un sistema de coordenadas local que tengo que "pegar" en el mapa global.

La fórmula exacta implica usar seno y coseno de la orientación del marcador para rotar correctamente las coordenadas:

```python
coord_x_global = coord_x_marcador + (cos_rot * x_rel - sin_rot * y_rel)
coord_y_global = coord_y_marcador + (sin_rot * x_rel + cos_rot * y_rel)
angulo_global = rotacion_marcador + angulo_rel
```

Así, cada estimación visual se convierte en una posible pose global del robot. Si el marcador detectado no aparece en el mapa YAML, esa detección se descarta.

### Selección de la Baliza más Cercana

Cuando el robot detecta múltiples marcadores simultáneamente, necesita decidir cuál usar para actualizar su posición. Esta fue precisamente una de las correcciones principales tras la tutoría: el criterio correcto no era priorizar ciertos IDs ni quedarse con el marcador más centrado, sino seleccionar la baliza más cercana.

Para ello, el programa procesa cada detección individualmente y guarda una tupla con:

```python
(tag_id, x_mundo, y_mundo, ang_mundo, x_rel, y_rel)
```

La distancia se calcula en el sistema relativo al marcador mediante `x_rel` e `y_rel`. No hace falta calcular la raíz cuadrada para comparar, porque comparar `x_rel**2 + y_rel**2` da el mismo orden y evita una operación innecesaria:

```python
mejor = min(lista_poses, key=lambda p: p[4]**2 + p[5]**2)
```

Después de seleccionar la baliza más cercana, su pose global se usa como `estimacion_visual`. De esta manera, si aparecen dos o más AprilTags, el robot cumple el criterio del enunciado y se queda con la estimación que debería ser más fiable.

### Fusión con Odometría Incremental

La odometría es el sistema que estima el movimiento del robot basándose en las lecturas de sus ruedas o sensores internos. Aunque es precisa a corto plazo, acumula error con el tiempo (efecto conocido como **drift**). La visión, por otro lado, proporciona mediciones absolutas pero solo cuando hay marcadores visibles. La clave está en combinar ambas fuentes de información de forma inteligente.

Mi implementación sigue la condición del enunciado: la odometría no se usa como posición absoluta, sino únicamente como incremento desde la última lectura. Cuando hay una estimación visual válida, el robot actualiza directamente `posicion_estimada` con esa pose calculada desde el marcador y guarda la lectura de odometría actual como referencia.

Cuando no se ve ninguna baliza, el programa calcula el desplazamiento entre la lectura odométrica anterior y la lectura actual:

```python
delta_x = pose_actual[0] - pose_anterior[0]
delta_y = pose_actual[1] - pose_anterior[1]
delta_ang = normalizar_angulo(pose_actual[2] - pose_anterior[2])
```

Ese incremento se suma a la última posición estimada. Así, la posición visual corrige el error acumulado cada vez que aparece una baliza, y la odometría permite mantener una estimación razonable durante los tramos en los que no hay marcadores visibles.

### Navegación por Tramos y Giros

Para que el robot vaya visitando distintas zonas, implementé una navegación sencilla basada en dos estados: `avanzar` y `girar`. En el estado de avance, el robot se mueve con velocidad lineal `0.2` hasta recorrer aproximadamente `3.0` metros medidos con odometría. Cuando supera esa distancia, cambia al estado de giro.

El giro también se controla con odometría. Al comenzar a girar, se guarda la orientación inicial y se acumula el cambio de yaw hasta alcanzar un objetivo de `140` grados. Durante este estado, la velocidad lineal se pone a cero y se aplica una velocidad angular de `0.2`. Cuando el giro acumulado alcanza el objetivo, el robot vuelve al estado de avance y reinicia el punto desde el que mide el siguiente tramo.

Este patrón avance-giro no necesita un mapa del entorno para moverse. Su objetivo dentro de la práctica es generar movimiento suficiente para que la cámara encuentre distintos AprilTags y la localización pueda alternar entre medidas visuales y odometría incremental.

### Bucle Principal de Localización

Todo el sistema se coordina mediante un bucle principal que ejecuta las siguientes etapas en cada iteración:

1. **Adquisición de datos** → Captura la imagen de la cámara y lee la odometría actual
2. **Preparación de imagen** → Convierte el frame a escala de grises y construye la matriz intrínseca
3. **Detección de marcadores** → Busca AprilTags con `pyapriltags`
4. **Procesamiento de detecciones** → Para cada marcador visible, calcula su pose relativa y global
5. **Selección de baliza** → Elige el marcador más cercano al robot
6. **Fusión** → Usa visión si hay baliza o aplica odometría incremental si no la hay
7. **Navegación** → Ejecuta la máquina de estados de avance y giro
8. **Visualización** → Muestra la pose estimada y la imagen con marcadores dibujados

Esta arquitectura separa la localización de la navegación. La navegación solo se encarga de mover el robot por el entorno, mientras que la localización decide en cada iteración si debe confiar en una medición visual o continuar con incrementos de odometría.

## Dificultades y Decisiones de Diseño

Una de las principales dificultades fue conseguir que las **transformaciones de coordenadas** funcionaran correctamente. El proceso de pasar de la pose del marcador respecto a la cámara a la pose del robot respecto al marcador, y finalmente a coordenadas globales del mapa, requiere aplicar múltiples rotaciones y traslaciones. Cualquier error en el orden de las operaciones o en los signos provocaba que el robot apareciera en posiciones completamente incorrectas. Tuve que verificar paso a paso cada transformación usando prints y visualización en el mapa hasta que todo encajó correctamente.

Otra dificultad fue elegir correctamente entre varias detecciones simultáneas. Mi versión anterior no garantizaba que se usase la baliza más cercana, y eso podía introducir errores cuando el robot veía más de un marcador. La solución final fue guardar todas las poses candidatas y comparar sus distancias relativas antes de actualizar la estimación visual.

La fusión con odometría también fue importante. No quería usar la odometría como posición absoluta, porque el propio enunciado indica que debe utilizarse solo para incrementos. Por eso, el código guarda la última lectura odométrica válida y únicamente suma las diferencias de movimiento cuando no hay una baliza visible.

En la navegación opté por un comportamiento simple y controlado: avanzar una distancia fija y después girar un ángulo fijo. No es una navegación óptima, pero sí suficiente para esta práctica, ya que permite que el robot recorra el entorno y vaya encontrando marcadores desde distintas posiciones.

## Video

El vídeo nuevo incluye 4 ejemplos de ejecución. En ellos se puede ver cómo el robot detecta AprilTags, selecciona la baliza más cercana cuando hay varias visibles y continúa estimando su pose con odometría incremental cuando deja de ver marcadores. Para generar recorridos distintos en las grabaciones, en algunos ejemplos modifiqué temporalmente los grados del giro, manteniendo la misma lógica de navegación por tramos y giros.

**Video de la Práctica - Localización Visual con Marcadores**  
[![Video de la Práctica - Marker Visual Loc](https://img.youtube.com/vi/Vjsi0aLf1Cg/0.jpg)](https://youtu.be/Vjsi0aLf1Cg)
