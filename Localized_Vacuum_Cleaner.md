## Índice
<details>
<summary>Práctica 1</summary>

- [Objetivo](#objetivo)
- [Teoría y Funcionamiento](#teoría-y-funcionamiento)
  * [Localización y Transformaciones](#localización-y-transformaciones)
  * [Rejilla del Mapa](#rejilla-del-mapa)
  * [Algoritmo de Cobertura BSA (Backtracking Spiral Algorithm)](#algoritmo-de-cobertura-bsa-backtracking-spiral-algorithm)
  * [Control del Movimiento del Robot](#control-del-movimiento-del-robot)
  * [Comportamiento Reactivo (Bumper)](#comportamiento-reactivo-bumper)
- [Dificultades Encontradas](#dificultades-encontradas)
- [Video](#video)

</details>


## Práctica 1 - Localized Vacuum Cleaner



<p align="center">
  <img src="https://github.com/user-attachments/assets/3c1b089b-d558-4f60-9974-e04d17946d2d" width="900" height="600">
</p>

## Objetivo

El objetivo de esta práctica, **Localized Vacuum Cleaner**, es conseguir que un robot aspiradora sea capaz de **cubrir completamente un entorno conocido** de forma autónoma.

Para ello, el robot debe interpretar el mapa del escenario, transformarlo en una **rejilla de navegación** que distinga entre zonas libres y paredes, y aplicar un algoritmo de cobertura (en este caso, **BSA - Backtracking Spiral Algorithm**) para recorrer todas las celdas posibles.

Durante la ejecución, el robot sigue los **waypoints** generados en la planificación, ajustando su movimiento mediante giros de 90° perfectos y avanzando en línea recta hasta encontrar paredes o zonas ya visitadas. Además, incorpora un sistema de reacción con el **bumper**, que le permite retroceder y girar en caso de colisión, evitando quedarse atascado.

El objetivo final es que el aspirador sea completamente autónomo, combinando **planificación global** con **comportamientos reactivos locales**, logrando cubrir toda la superficie de limpieza de forma eficiente y sin intervención humana.

## Teoría y Funcionamiento

Para esta práctica se ha utilizado una combinación de **localización**, **planificación por celdas** y un algoritmo de **cobertura sistemática** para simular el comportamiento de un robot aspiradora inteligente.  

La idea principal es que el robot pueda entender su entorno a partir de un mapa, dividirlo en zonas navegables y recorrerlo siguiendo una estrategia ordenada que garantice que no deje ninguna parte sin limpiar.

### Localización y Transformaciones

Una de las partes más importantes fue conseguir pasar las coordenadas del mundo de Gazebo a las del mapa en píxeles. Para hacerlo, coloqué el robot manualmente en las **cuatro esquinas del mapa** y saqué sus posiciones con `HAL.getPose3d()`. Con eso tenía los pares de puntos del tipo `(x, y)` en el mundo y `(u, v)` en la imagen.


Para hacerlo, primero obtuve las **poses del robot en las cuatro esquinas del mapa**, colocándolo manualmente en cada esquina y leyendo su posición con `HAL.getPose3d()`.  
Así conseguí pares de puntos del tipo:

- Esquina inferior izquierda: (5.47, 5.47)
- Esquina superior izquierda: (5.48, -3.87)
- Esquina superior derecha: (-4.10, -3.84)
- Esquina inferior derecha: (-4.10, 5.46)

De esta forma, conocía cómo se correspondían los puntos extremos del mundo con los de la imagen.  

A partir de ahí, el problema se reduce a una **transformación afín bidimensional**, que básicamente es una ecuación del tipo:


```
u = a₁x + a₂y + a₃
v = b₁x + b₂y + b₃
```

Sirve para pasar de coordenadas reales a píxeles del mapa.  
Usando las cuatro esquinas se pueden sacar los coeficientes resolviendo el sistema, y con eso obtuve las funciones finales:

```python
def world_to_img(x, y):
    u = 579.07853885 - 106.87212028405666 * x - 0.03885165140519265 * y
    v = 419.45699739 - 0.03885165140519265 * x + 106.87212028405666 * y
    return u, v

def img_to_world(u, v):
    du = u - 579.07853885
    dv = v - 419.45699739
    x = -0.00935697603 * du - 0.00000340157910 * dv
    y = -0.00000340157910 * du + 0.00935697603 * dv
    return x, y
```

La primera pasa de mundo a imagen y la segunda al revés.

Gracias a esto el robot puede saber exactamente en qué celda del mapa está y moverse de forma precisa. Fue clave para que todo lo demás funcionara bien.







### Rejilla del Mapa

Para poder aplicar el algoritmo de limpieza y planificación, primero necesitaba dividir el mapa en una **rejilla de celdas**. Básicamente lo que hice fue transformar la imagen del mapa en una cuadrícula donde cada celda representa una zona navegable o una pared.  

Lo primero fue convertir el mapa en una **imagen binaria**, donde los píxeles blancos representan zonas libres (1) y los negros paredes (0). Luego, dividí toda la imagen en bloques cuadrados del tamaño de una celda (en mi caso 36x36 píxeles) y calculé cuántos píxeles libres había dentro de cada bloque. Si más del 93% de los píxeles eran blancos, esa celda se marcaba como **libre**, y si no, como **ocupada**.  

Con eso, el robot ya no trabaja a nivel de píxeles sino a nivel de celdas, lo que simplifica mucho el movimiento y la planificación. Además, así puedo saber de un vistazo qué zonas son navegables y cuáles no, y el robot puede decidir fácilmente por qué celdas moverse.  

También añadí líneas de cuadrícula para visualizarlo mejor en el mapa, y colores distintos para las zonas libres, las paredes y los diferentes estados del algoritmo. Esto me ayudó un montón para depurar y entender cómo el robot interpretaba el entorno.  



### Algoritmo de Cobertura BSA (Backtracking Spiral Algorithm)

Una vez tuve la rejilla lista, necesitaba que el robot fuera capaz de recorrer todo el mapa sin dejar zonas sin limpiar. Para eso implementé el **algoritmo BSA (Backtracking Spiral Algorithm)**, que básicamente hace que el robot se mueva como si estuviera dibujando una espiral dentro del mapa.  

El funcionamiento es bastante simple: el robot avanza en línea recta hasta que se encuentra con una pared o con una celda que ya ha visitado. En ese momento, gira 90º en sentido horario y vuelve a avanzar hasta volver a toparse con un obstáculo o una celda recorrida. De esta forma, el movimiento que hace se parece mucho al de una espiral que va rellenando el espacio libre poco a poco.  

Cuando llega a un punto donde ya no tiene ninguna celda libre alrededor, entra en una **fase de retroceso o búsqueda**. En esta fase, busca la celda libre más cercana que aún no haya sido visitada utilizando un **BFS (Breadth-First Search)**, y genera un pequeño camino hasta llegar allí. A partir de esa nueva posición vuelve a empezar el algoritmo.  

Durante el recorrido, cada tipo de celda se pinta con un color distinto para poder visualizar lo que está ocurriendo:
- **Amarillo:** cuando el robot avanza en línea recta.  
- **Naranja:** cuando gira 90º.  
- **Verde:** la celda inicial del algoritmo o las que el robot ya ha pasado.  
- **Rojo:** cuando ya no hay más celdas por recorrer (fin de la planificación).  
- **Azul:** cuando el robot se mueve hacia una nueva zona libre tras hacer un salto con BFS.

(Me inspiré en el video de Javier Izquierdo que está en el enunciado para reutilizar los colores de los movimientos)

<img width="385" height="461" alt="image" src="https://github.com/user-attachments/assets/bf524978-c5f3-45c9-a51c-05d6285223c9" />


Gracias a este enfoque, el robot consigue recorrer todas las celdas libres del mapa sin necesidad de tener una ruta predefinida. Es un método muy intuitivo pero efectivo, y al verlo pintarse en tiempo real se entiende perfectamente cómo va “varrendo” todo el entorno paso a paso.

<img width="934" height="458" alt="image" src="https://github.com/user-attachments/assets/c2e3fd78-c746-4fc0-ad33-72b72f535f0a" />

### Control del Movimiento del Robot

Una vez el algoritmo BSA termina de planificar el recorrido, el siguiente paso es hacer que el robot lo ejecute en el entorno real de **Gazebo**.  

Durante la planificación, cada celda libre que el robot visita se guarda como un **waypoint** (punto intermedio). Luego, en la fase de ejecución, el robot se mueve de waypoint en waypoint siguiendo el orden en el que fueron generados, lo que permite una limpieza completa y ordenada del entorno.

Para poder controlar correctamente el movimiento del robot, utilicé la función `absolute2relative` (procedente de la práctica **[Global Navigation](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/)** de Robótica Móvil, no está en el enunciado pero la daba Fran Moreno en clase).

``` python
def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
    return x_rel, y_rel

```

Esta función convierte las coordenadas **absolutas del mundo** en **coordenadas relativas al robot**, permitiendo saber en cada momento en qué dirección y a qué distancia se encuentra el siguiente waypoint respecto a su orientación actual.  

Gracias a esta conversión, el control de movimiento se vuelve mucho más preciso, ya que puedo ajustar las velocidades de la siguiente forma:

- **Velocidad lineal (V):** se calcula en función de la distancia al waypoint, controlando la velocidad hacia delante o atrás.  
- **Velocidad angular (W):** depende del ángulo entre la orientación actual del robot y el waypoint, permitiendo que gire suavemente hacia su destino.  

El control se realiza de forma continua dentro del bucle principal, haciendo que el robot se corrija constantemente mientras avanza sin necesidad de recalcular toda la ruta. 

Durante esta fase, las celdas que el robot va recorriendo se pintan de **verde**, mostrando visualmente en el mapa el progreso de la limpieza en tiempo real.  



### Comportamiento Reactivo (Bumper)

Además del movimiento planificado, el robot incluye un **comportamiento reactivo** mediante el sensor **bumper**.  

Si detecta una colisión, el robot retrocede ligeramente y gira en función del lado del impacto (izquierda, derecha o centro), permitiéndole salir de situaciones de atasco.  

Si los choques se repiten, realiza una maniobra de escape más amplia con un giro completo de 90°.

### Dificultades Encontradas

Una de las mayores dificultades fue **ajustar correctamente el mapa**. La conversión entre coordenadas del mundo real y las coordenadas de la imagen fue bastante delicada, ya que si los valores no coincidían exactamente, el robot no aparecía en la posición correcta o incluso se movía fuera de los límites del mapa.  

Tuve que sacar manualmente las **poses de las esquinas** del mapa y usarlas para calcular las transformaciones lineales entre el mundo y la imagen. Fue un proceso algo tedioso porque cualquier pequeño error en los coeficientes hacía que el robot se desplazara de forma rara o que los waypoints no coincidieran con las zonas del mapa.

Otra cosa que me dio bastantes quebraderos de cabeza fue el **control del movimiento del robot**. Al principio el robot no se movía bien hacia los waypoints, giraba de forma extraña o incluso iba en dirección contraria. Luego me acordé de la función `absolute2relative` que habíamos usado en la práctica de [Global Navigation](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/), y eso fue clave para resolverlo. Con esa conversión a coordenadas relativas, pude controlar mucho mejor las velocidades lineal y angular del robot.

También tuve algunos problemas con los **choques y los atascos**. Había momentos en que el robot se quedaba trabado en las patas de las mesas o contra las paredes y no conseguía salir. Al final añadí un sistema de escapes con retroceso y giros de 90º que lo saca de casi cualquier situación.

## Video

Disculpa la calidad de los videos, lo tuve que grabar con el móvil ya que mi ordenador no tiene la capacidad suficiente para grabar y ejecutar la simulación al mismo tiempo.


**Video**  
[![Video de la Práctica 1 - Localized Vacuum Cleaner](https://img.youtube.com/vi/YM52bC-10-A/0.jpg)](https://youtu.be/YM52bC-10-A?si=imU6zbs2-S5Oo5Zq)




