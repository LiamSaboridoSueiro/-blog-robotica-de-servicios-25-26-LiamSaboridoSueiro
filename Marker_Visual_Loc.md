
## Índice
<details>
<summary>Práctica 6</summary>

- [Objetivo](#objetivo)
  * [Detección de Marcadores AprilTag](#detección-de-marcadores-apriltag)
  * [Estimación de Pose 3D mediante PnP](#estimación-de-pose-3d-mediante-pnp)
  * [Transformación al Sistema de Coordenadas del Robot](#transformación-al-sistema-de-coordenadas-del-robot)
  * [Proyección a Coordenadas Globales del Mapa](#proyección-a-coordenadas-globales-del-mapa)
  * [Sistema de Selección y Estabilidad Visual](#sistema-de-selección-y-estabilidad-visual)
  * [Fusión con Odometría](#fusión-con-odometría)
  * [Navegación Reactiva con Exploración](#navegación-reactiva-con-exploración)
  * [Máquina de Estados de Localización](#máquina-de-estados-de-localización)
- [Dificultades Encontradas](#dificultades-encontradas)
- [Video](#video)

</details>

## Práctica 6 - Marker Visual Loc


<p align="center">
  <img src="https://github.com/user-attachments/assets/1ad29982-f42d-4c97-8db9-21566723ba46" width="900" height="600">
</p>


## Objetivo

El objetivo de esta práctica es implementar un sistema de localización visual para un robot móvil utilizando marcadores AprilTag distribuidos por el entorno. El robot debe ser capaz de estimar su posición y orientación en un mapa conocido mediante la detección de estos marcadores, fusionando esta información con datos odométricos para mantener una estimación precisa incluso cuando pierde de vista las balizas. Además, el robot debe navegar de forma autónoma explorando el entorno mientras actualiza continuamente su posición.

La idea es combinar visión por computador para detectar marcadores fiduciales, geometría 3D para calcular poses relativas, y un sistema de fusión de sensores que combine la información visual con la odometría del robot. Todo esto coordinado mediante una navegación reactiva que permite explorar el entorno de forma segura.

### Detección de Marcadores AprilTag

Los AprilTag son marcadores visuales similares a códigos QR, pero diseñados específicamente para robótica. Cada marcador tiene un patrón único que permite identificarlo y calcular su posición en 3D a partir de una imagen 2D. En mi implementación, utilicé la librería `pyapriltags` que se encarga de detectar estos marcadores en las imágenes capturadas por la cámara del robot.

El proceso comienza convirtiendo la imagen en color a escala de grises, ya que los marcadores funcionan mejor con imágenes en blanco y negro donde el contraste es máximo. Una vez procesada la imagen, el detector devuelve una lista de todos los marcadores encontrados, cada uno con su ID único y las coordenadas de sus cuatro esquinas en píxeles. Esta información es fundamental porque las esquinas nos permiten calcular la orientación y distancia del marcador respecto a la cámara.

Para visualizar las detecciones, dibujo un recuadro verde alrededor de cada marcador identificado y muestro su ID. Esto no solo ayuda a depurar el sistema, sino que también permite verificar visualmente qué marcadores está viendo el robot en cada momento. La detección es robusta incluso con iluminación variable o cuando los marcadores están parcialmente ocluidos, lo que hace que esta tecnología sea muy fiable para aplicaciones de localización en interiores.

### Estimación de Pose 3D mediante PnP

Una vez detectadas las esquinas de un marcador en la imagen, el siguiente paso es determinar dónde está ese marcador en el espacio 3D respecto a la cámara. Para esto utilizo el algoritmo **PnP (Perspective-n-Point)**, que es el estándar en visión por computador para resolver este tipo de problemas. La idea básica es: si conozco las dimensiones reales del marcador (0.24 metros de lado en este caso) y veo cómo aparece en la imagen, puedo calcular su posición y orientación en el espacio.

El PnP necesita dos cosas: los puntos 3D del marcador en su propio sistema de coordenadas (las cuatro esquinas del cuadrado) y los puntos 2D correspondientes en la imagen (en píxeles). Con esta información, el algoritmo encuentra la mejor transformación que relaciona ambos conjuntos de puntos. Esta transformación se representa mediante un vector de rotación y un vector de traslación, que nos dicen exactamente cómo está orientado el marcador y a qué distancia se encuentra.

Matemáticamente, esto resuelve un sistema de ecuaciones que relaciona coordenadas 3D con sus proyecciones 2D a través del modelo de cámara. El resultado es extremadamente preciso: puedo saber si el marcador está a 2 metros delante del robot y girado 45 grados, por ejemplo. Este paso es crucial porque convierte información visual plana (píxeles en una imagen) en información espacial real (metros y ángulos en el mundo), que es lo que necesito para localizar al robot.

### Transformación al Sistema de Coordenadas del Robot

El PnP me da la pose del marcador respecto a la **cámara**, pero lo que realmente necesito es saber dónde está el **robot** respecto al marcador. Esto requiere una transformación de coordenadas, que es como traducir entre dos idiomas matemáticos diferentes.

Primero invierto la transformación que obtuve del PnP. Si el PnP me dice "la cámara ve el marcador aquí", yo necesito voltear esa información para decir "el marcador ve la cámara aquí". Esto se hace invirtiendo la matriz de rotación (transponiendo) y recalculando la traslación. Luego, como la cámara está montada en el robot pero no justo en su centro, aplico otra transformación que corrige esta diferencia. En mi caso, la cámara está 20 cm por delante del centro del robot.

El resultado final son tres valores: **x_rel** (cuánto está el robot delante/detrás del marcador), **y_rel** (cuánto está a la izquierda/derecha), y **yaw_rel** (cuánto está girado). Estos valores son relativos al marcador, no al mundo, pero son el paso previo indispensable para calcular la posición global del robot. Esta transformación puede parecer técnica, pero básicamente es como calcular tu posición en una habitación sabiendo dónde está una lámpara y qué distancia y ángulo hay entre tú y ella.

### Proyección a Coordenadas Globales del Mapa

Una vez que sé dónde está el robot respecto a un marcador específico, necesito convertir esa información relativa en una posición absoluta dentro del mapa del entorno. Para esto utilizo un archivo YAML que contiene las posiciones conocidas de todos los marcadores en coordenadas globales. Cada marcador tiene almacenadas sus coordenadas X, Y y su orientación (yaw) en el sistema de referencia del mapa.

La idea es sencilla en concepto pero requiere cuidado en la implementación: si sé dónde está el marcador en el mundo y sé dónde estoy yo respecto al marcador, puedo calcular dónde estoy en el mundo. Matemáticamente esto implica una **rotación 2D** seguida de una **traslación**. Primero roto las coordenadas relativas del robot según la orientación del marcador en el mapa, y luego las sumo a la posición del marcador. Es como si el marcador fuera un sistema de coordenadas local que tengo que "pegar" en el mapa global.

La fórmula exacta implica usar seno y coseno de la orientación del marcador para rotar correctamente las coordenadas. Por ejemplo, si el marcador está orientado 90 grados en el mapa, lo que para mí es "adelante" respecto al marcador, en el mapa global será "a la izquierda". Esta transformación garantiza que las direcciones se preserven correctamente. Además, añadí un pequeño **offset frontal** de 5 cm que compensa ligeras imprecisiones en el modelo de la cámara, mejorando la precisión final de la localización.

### Sistema de Selección y Estabilidad Visual

Cuando el robot detecta múltiples marcadores simultáneamente, necesita decidir cuál usar para actualizar su posición. Usar todos a la vez mediante un promedio podría parecer buena idea, pero en la práctica genera inestabilidad si los marcadores están en diferentes ángulos o distancias. Mi solución fue implementar un **sistema de selección por prioridad** que elige un único marcador como referencia.

El criterio de selección funciona en dos niveles: primero busca marcadores con IDs específicos (0, 1, 2) que están estratégicamente ubicados en el entorno. Si encuentra alguno de estos IDs preferidos, lo usa automáticamente. Si no, selecciona el marcador más frontal, es decir, aquel que está más alineado con el eje delantero del robot. Esto se calcula usando el ángulo relativo: un marcador perfectamente centrado frente al robot tiene ángulo cero, mientras que uno a un lado tiene un ángulo mayor.

Pero seleccionar un marcador no es suficiente. Para evitar saltos bruscos en la estimación cuando el robot cambia entre diferentes marcadores, implementé un **sistema de estabilidad por frames**. El robot solo confía en una detección visual después de ver el mismo marcador durante al menos 3 frames consecutivos. Esto filtra detecciones esporádicas o erróneas y garantiza que solo se actualice la posición cuando hay confianza real en la medición. Si el robot pierde de vista todos los marcadores, el sistema reduce gradualmente la confianza visual hasta volver a depender únicamente de la odometría.

### Fusión con Odometría

La odometría es el sistema que estima el movimiento del robot basándose en las lecturas de sus ruedas o sensores internos. Aunque es precisa a corto plazo, acumula error con el tiempo (efecto conocido como **drift**). La visión, por otro lado, proporciona mediciones absolutas pero solo cuando hay marcadores visibles. La clave está en combinar ambas fuentes de información de forma inteligente.

Mi implementación mantiene dos poses separadas: una odométrica que se actualiza constantemente con los incrementos de movimiento, y una visual que solo se actualiza cuando hay detecciones válidas. La pose odométrica funciona calculando cuánto se ha movido el robot desde la última iteración (delta_x, delta_y, delta_yaw) y sumando estos incrementos a la posición anterior. Esto permite seguir el movimiento del robot incluso sin ver marcadores.

Cuando el sistema visual está activo (tras pasar el filtro de estabilidad), la pose fusionada se **reemplaza directamente** por la estimación visual. Esta decisión de diseño es deliberadamente simple pero efectiva: confío plenamente en la visión cuando está disponible y estable, y confío en la odometría cuando no hay información visual. No uso un filtro de Kalman ni promedios ponderados complejos porque en este entorno la visión es suficientemente precisa y la odometría suficientemente estable en periodos cortos. Esta fusión binaria (o visión o odometría) resulta robusta y fácil de depurar.

### Navegación Reactiva con Exploración

Para que el robot pueda descubrir marcadores por el entorno, implementé un sistema de navegación basado en una **máquina de estados simple** con dos modos: avanzar y girar. El robot va recto a velocidad constante (20 unidades) hasta que el láser frontal detecta un obstáculo a menos de 0.5 metros. En ese momento, cambia al modo de giro.

El giro no es fijo ni predeterminado, sino **aleatorio** entre 90 y 180 grados. Esto introduce variabilidad en la exploración y evita que el robot se quede atrapado en bucles repetitivos. Por ejemplo, si siempre girara exactamente 90 grados, podría acabar moviéndose en un patrón cuadrado predecible. La aleatoriedad garantiza que explore diferentes zonas del entorno.

Para ejecutar el giro de forma controlada, el robot mide su orientación inicial cuando detecta el obstáculo y va acumulando el ángulo girado frame a frame. Cuando el giro acumulado alcanza el objetivo aleatorio, vuelve al modo de avance. Este comportamiento reactivo es muy robusto: no requiere mapas ni planificación compleja, simplemente evita obstáculos y sigue explorando. El láser cubre un arco frontal de ±30 grados, lo que da al robot suficiente margen para detectar obstáculos antes de chocar pero sin ser excesivamente conservador.

### Máquina de Estados de Localización

Todo el sistema se coordina mediante un bucle principal que ejecuta las siguientes etapas en cada iteración:

1. **Adquisición de datos** → Captura la imagen de la cámara y lee la odometría actual
2. **Actualización odométrica** → Calcula y aplica los incrementos de movimiento a la pose fusionada
3. **Detección de marcadores** → Procesa la imagen en escala de grises y busca AprilTags
4. **Procesamiento de detecciones** → Para cada marcador visible, calcula su pose global
5. **Selección y estabilidad** → Elige el mejor marcador y verifica estabilidad temporal
6. **Fusión** → Actualiza la pose fusionada si el modo visual está activo
7. **Navegación** → Ejecuta la máquina de estados de exploración según lecturas del láser
8. **Visualización** → Muestra la pose estimada y la imagen con marcadores dibujados

Esta arquitectura modular permite que cada componente funcione de forma independiente pero coordinada. Si falla la detección visual, la odometría mantiene la localización. Si el robot está quieto, el sistema visual puede corregir el drift acumulado. La máquina de estados no tiene conocimiento directo de la localización, simplemente explora, mientras que el sistema de localización no controla el movimiento, solo observa y estima. Esta separación de responsabilidades hace el código más robusto y fácil de mantener.

## Dificultades y Decisiones de Diseño

Una de las principales dificultades fue conseguir que las **transformaciones de coordenadas** funcionaran correctamente. El proceso de pasar de la pose del marcador respecto a la cámara a la pose del robot respecto al marcador, y finalmente a coordenadas globales del mapa, requiere aplicar múltiples rotaciones y traslaciones. Cualquier error en el orden de las operaciones o en los signos provocaba que el robot apareciera en posiciones completamente incorrectas. Tuve que verificar paso a paso cada transformación usando prints y visualización en el mapa hasta que todo encajó correctamente.

El **sistema de estabilidad visual** también requirió varios ajustes. Al principio el robot saltaba bruscamente entre posiciones cuando detectaba diferentes marcadores, lo que generaba movimientos erráticos. Implementar el filtro de frames consecutivos mejoró mucho esto, pero encontrar el número óptimo de frames (finalmente 3) fue cuestión de prueba y error. Si ponía muy pocos, seguía habiendo saltos; si ponía demasiados, el sistema tardaba en reaccionar ante nuevas detecciones válidas.


La **navegación reactiva** presentó el reto de evitar que el robot se quedara atrapado en bucles. Los primeros intentos con giros fijos de 90 grados hacían que el robot se moviera en patrones predecibles y a veces acabara oscilando entre dos paredes. Introducir aleatoriedad en los giros (entre 90 y 180 grados) resolvió este problema y permitió que el robot explorara de forma mucho más natural y efectiva todo el entorno.

## Video

Disculpa la calidad de los videos, mi ordenador no da para mucho...

**Video de la Práctica - Localización Visual con Marcadores**  
[![Video de la Práctica - Marker Visual Loc](https://img.youtube.com/vi/xBiRHfzs4Zo/0.jpg)](https://youtu.be/xBiRHfzs4Zo)
