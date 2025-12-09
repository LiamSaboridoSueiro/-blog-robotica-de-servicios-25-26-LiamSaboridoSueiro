## Índice
<details>
<summary>Práctica 5 – Laser Mapping</summary>

- [Objetivo](#objetivo)
- [Teoría y Funcionamiento](#teoría-y-funcionamiento)
  * [LIDAR y Adquisición de Datos](#lidar-y-adquisición-de-datos)
  * [Mapa de Ocupación](#mapa-de-ocupación)
  * [Algoritmo de Mapeo](#algoritmo-de-mapeo)
  * [Trazado de Láser con Bresenham](#trazado-de-láser-con-bresenham)
  * [Exploración Reactiva / Máquina de Estados](#exploración-reactiva--máquina-de-estados)
  * [Algoritmo de Navegacion](#algoritmo-de-navegacion)
- [Dificultades Encontradas](#dificultades-encontradas)
- [Video](#video)

</details>


<p align="center">
  <img src="https://github.com/user-attachments/assets/938b747e-b036-4242-87e4-c90aafabc33c" width="900" height="600">
</p>


## Objetivo

El objetivo de esta práctica es diseñar un sistema capaz de explorar un entorno desconocido utilizando únicamente un sensor LIDAR, construyendo simultáneamente un mapa de ocupación del área visitada. El robot debe desplazarse de forma autónoma, detectando obstáculos, actualizando el mapa en tiempo real y tomando decisiones de navegación basadas en su percepción del entorno. El resultado esperado es un mapa fiable que represente correctamente las zonas libres y los obstáculos recorriendo el escenario propuesto.

## Teoría y Funcionamiento

### LIDAR y Adquisición de Datos

El sensor LIDAR es el dispositivo encargado de medir distancias entre el robot y los objetos del entorno mediante pulsos láser. Cada barrido proporciona una serie de lecturas angulares, donde cada valor representa la distancia medida en una dirección concreta alrededor del robot. Estas mediciones permiten identificar obstáculos, calcular su posición relativa y reconstruir progresivamente la geometría del escenario. El uso del LIDAR facilita la creación de un mapa preciso sin necesidad de visión artificial o sensores adicionales, siempre que el robot pueda estimar correctamente su posición y orientación en el mundo.

### Mapa de Ocupación

El mapa de ocupación es la representación interna que utiliza el robot para “entender” el entorno que está explorando. En esta práctica se modela como una rejilla 2D (una matriz de 970 x 1500 celdas) donde cada celda almacena un valor entero que indica el estado de esa zona del mundo. En concreto, se usan tres niveles principales: **NO_VISIT (gris)** para las celdas que aún no han sido observadas, **VACIO (blanco)** para las que el LIDAR ha identificado como espacio libre y **OBSTACULO (negro)** para aquellas donde se ha detectado un impacto del láser contra un objeto. De esta forma, el mapa funciona como una imagen en escala de grises donde el negro representa obstáculos, el blanco zonas transitables y el gris áreas desconocidas.

Cada vez que el robot recibe nuevas lecturas del LIDAR, estas se convierten primero a coordenadas del mundo y después a coordenadas de píxel mediante la función `WebGUI.poseToMap`. Con esa información, se actualizan las celdas correspondientes del mapa: las posiciones por las que pasa el rayo hasta llegar al obstáculo se marcan como **VACIO**, mientras que el punto final donde el láser impacta se marca como **OBSTACULO**. Este proceso se repite en cada iteración, de modo que, a medida que el robot se mueve, el mapa va “pintando” el entorno explorado y construyendo una representación global del almacén que refleja cuáles zonas son accesibles y cuáles no.

Aunque para visualizar el mapa en WebGUI se utilizan valores discretos en escala de grises (gris, blanco y negro), internamente el gridmap se actualiza de forma probabilística siguiendo el modelo del sensor láser. Las celdas no se interpretan como estados binarios fijos, sino como probabilidades de ocupación que se combinan con las observaciones sucesivas.


### Algoritmo de Mapeo

El algoritmo de mapeo se basa en la interpretación probabilística de las lecturas del LIDAR, actualizando el gridmap a medida que el robot explora el entorno. El proceso comienza en la función `procesar_laser`, que recorre todas las mediciones del sensor, filtra valores inválidos y convierte cada lectura a coordenadas del mundo aplicando rotación y traslación según la pose actual del robot (`x_rob`, `y_rob`, `theta_rob`). Este modelo sensorial se asume axial, ya que el haz láser es estrecho y puede considerarse prácticamente lineal con respecto a la dirección de avance.

Una vez obtenidos los puntos en coordenadas globales, la función `actualizar_mapa` los proyecta sobre la rejilla probabilística utilizando `WebGUI.poseToMap`. Para cada rayo, se genera la trayectoria completa entre el robot y el punto de impacto mediante el algoritmo de Bresenham. Las celdas intermedias del rayo representan evidencia de espacio libre, mientras que la celda final representa evidencia de ocupación.

La actualización de cada celda se realiza siguiendo la combinación bayesiana de evidencias: cuando el láser confirma que una celda es libre, su probabilidad de ocupación disminuye; cuando detecta un obstáculo, su probabilidad aumenta. Para evitar que el sistema se vuelva excesivamente inercial, se aplican mecanismos de saturación, de manera que las probabilidades se mantienen dentro de un rango limitado (p. ej. entre \( p_{\min} \) y \( p_{\max} \)), garantizando que observaciones nuevas puedan corregir evidencias previas.

En cada iteración del bucle principal, se procesan las nuevas mediciones, se aplican las actualizaciones probabilísticas sobre el gridmap y se envía el resultado a la interfaz gráfica mediante `WebGUI.setUserMap(grid)`. Con ello, el mapa se refina de manera incremental y coherente a medida que el robot recorre zonas previamente desconocidas, obteniendo finalmente una representación fiable del entorno.


### Trazado de Láser con Bresenham

Para representar correctamente cada rayo del LIDAR sobre el mapa, es necesario saber qué celdas de la rejilla atraviesa el haz desde la posición del robot hasta el punto donde impacta. En lugar de aproximarlo de forma continua, se utiliza el algoritmo de Bresenham, que permite calcular de manera eficiente todos los píxeles que forman una línea entre dos puntos discretos.

En la implementación, la función `linea_bresenham(x1, y1, x2, y2)` recibe las coordenadas en píxeles del robot y del obstáculo (`(x1, y1)` y `(x2, y2)`) y devuelve una lista de pares `(x, y)` que representan todos los puntos intermedios de la línea. Internamente, se trabaja únicamente con operaciones enteras: se calcula la diferencia en X e Y, se determina el sentido de avance en cada eje y se actualiza un término de error (`error`) que decide cuándo incrementar X, Y o ambos. De esta forma, se recorre paso a paso la rejilla de píxeles hasta llegar al destino.

Esta lista de puntos se utiliza en `actualizar_mapa`: todos los puntos de la línea menos el último se marcan como **VACIO** (espacio libre), ya que representan el recorrido del rayo hasta el obstáculo, mientras que el último punto se marca como **OBSTACULO**. Gracias a Bresenham, el trazado de cada rayo es preciso y eficiente, y el mapa de ocupación refleja fielmente qué zonas están despejadas y dónde se encuentran los objetos detectados por el láser.

### Exploración Reactiva / Máquina de Estados

La lógica de exploración se plantea como un sistema reactivo controlado mediante una máquina de estados sencilla. La idea es que el robot tome decisiones de movimiento únicamente a partir de la información que le proporciona el LIDAR, sin depender de rutas prefijadas ni de waypoints globales. De este modo, el comportamiento se adapta en tiempo real a la presencia de obstáculos y a la estructura del entorno.

El diseño básico consta de al menos dos estados principales: **EXPLORAR** y **GIRAR**. En el estado **EXPLORAR**, el robot avanza hacia delante mientras realiza pequeños ajustes aleatorios en la velocidad angular para no seguir siempre una trayectoria perfectamente recta. Este comportamiento le permite ir “abriendo” el mapa y cubriendo distintas zonas del almacén. Cuando el LIDAR detecta un obstáculo a una distancia menor que un umbral de seguridad en la dirección frontal, la máquina de estados cambia al modo **GIRAR**. En este segundo estado, el robot detiene el avance lineal y ejecuta un giro controlado (por ejemplo, de unos 90 grados) hacia el lado más despejado según las lecturas del láser. Una vez completado el giro y orientado hacia una zona libre, la máquina vuelve al estado **EXPLORAR**. Con este esquema, el robot es capaz de navegar de forma autónoma evitando colisiones y, al mismo tiempo, ir construyendo el mapa de ocupación.

### Algoritmo de Navegación

El algoritmo de navegación se basa en un patrón de exploración reactiva cuyo objetivo principal es visitar zonas aún no observadas del entorno, incrementando progresivamente la cobertura del mapa. En cada iteración se analizan las distancias medidas por el LIDAR en el eje frontal y en los laterales, aplicando un modelo sensorial axial (el haz se considera prácticamente lineal), lo que simplifica el cálculo geométrico del espacio libre. A partir de esa información, el robot decide el movimiento local: si el frente está despejado y el gridmap indica baja probabilidad de ocupación, el robot avanza con una velocidad lineal constante. Para evitar movimientos repetitivos y fomentar la visita de áreas desconocidas, se añade una pequeña variación angular que permite explorar trayectorias distintas y así cubrir mejor la superficie del entorno.

Cuando la probabilidad de ocupación en la dirección frontal supera un umbral (bien sea porque la distancia medida por el LIDAR es pequeña o porque el mapa acumulado indica alta evidencia de obstáculo), la máquina de estados activa la maniobra de giro. En ese momento el robot compara la probabilidad acumulada a izquierda y derecha —combinada mediante actualización bayesiana de evidencias— y selecciona el lado con menor ocupación estimada. El giro se realiza de forma controlada hasta completar aproximadamente un desfase angular fijo (en torno a 90°), tras el cual el robot vuelve al estado de avance.

Este enfoque permite una exploración autónoma basada en la incertidumbre del mapa: el robot prioriza avanzar hacia zonas poco observadas o con baja probabilidad de obstáculo, mientras evita colisiones y continúa ampliando el gridmap. La incorporación de mecanismos de saturación garantiza que la probabilidad almacenada no crezca indefinidamente, evitando excesiva inercia y permitiendo que nuevas observaciones del LIDAR modifiquen de forma razonable la confianza en cada celda. De este modo, navegación y mapeo probabilístico se ejecutan conjuntamente, logrando cubrir el entorno mientras el mapa se vuelve cada vez más fiable.


## Dificultades Encontradas

Una de las principales dificultades fue programar la práctica en mi ordenador, como este lo he comprado en 2015 y la gpu no es muy potente. Iba muy laggeado en mi ordenador :(

Otra dificultad adicional fue conseguir un comportamiento de navegación reactivo estable, ya que pequeños cambios en las ganancias del movimiento podían hacer que el robot quedara atascado al girar repetidamente o que avanzara demasiado agresivo sin cubrir bien el entorno. Fue necesario ajustar los umbrales de detección del LIDAR y la duración de los giros hasta obtener un equilibrio aceptable entre exploración y seguridad.

También resultó complejo validar el trazado del mapa, ya que los errores en la transformación de coordenadas o en la línea de Bresenham no eran visibles al instante. En varias ocasiones fue necesario depurar comparando posiciones impresas por consola con las celdas actualizadas del mapa, para asegurar que la representación fuese coherente.

Asimismo, ajustar los límites de saturación y los umbrales probabilísticos también requirió varios intentos, ya que afectaban directamente a la estabilidad del gridmap y al comportamiento de la navegación reactiva.



## Video

Disculpa la calidad de los videos, lo tuve que grabar con el móvil ya que mi ordenador no tiene la capacidad suficiente para grabar y ejecutar la simulación al mismo tiempo.

[![Vídeo Laser Mapping](https://img.youtube.com/vi/EFyaFYUgBHA/0.jpg)](https://youtube.com/shorts/EFyaFYUgBHA)

## Mapa final:

<img width="702" height="412" alt="image" src="https://github.com/user-attachments/assets/a0733e27-6920-4461-9624-b8d2515e9ad8" />


