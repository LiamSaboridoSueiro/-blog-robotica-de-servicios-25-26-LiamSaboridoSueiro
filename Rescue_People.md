## Índice
<details>
<summary>Practica 2</summary>

- [Objetivo](#objetivo)
- [Teoría y Funcionamiento](#teoría-y-funcionamiento)
  * [Conversión de Coordenadas (GPS-UTM-Local)](#conversión-de-coordenadas-gps-utm-local)
  * [Máquina de Estados Finita (FSM)](#máquina-de-estados-finita-fsm)
  * [Patrón de Búsqueda (Lawnmower)](#patrón-de-búsqueda-lawnmower)
  * [Detección de Caras (Cámara Ventral)](#detección-de-caras-cámara-ventral)
  * [Control del Vuelo (Velocidad, Yaw y Altura)](#control-del-vuelo-velocidad-yaw-y-altura)
  * [Aterrizaje y Gestión de Batería](#aterrizaje-y-gestión-de-batería)
- [Dificultades Encontradas](#dificultades-encontradas)
- [Video](#video)

</details>


## Práctica 2 - Rescue people



<p align="center">
  <img src="https://github.com/user-attachments/assets/414f19a4-b370-468b-9346-6b699298faf2" width="900" height="600">
</p>

## Objetivo

El objetivo de esta práctica, **Rescue People**, es programar un dron autónomo capaz de realizar una misión de búsqueda y rescate en la que debe despegar, navegar hasta una zona determinada, recorrer el área siguiendo un patrón de búsqueda y detectar personas mediante visión por computador usando un clasificador Haar Cascade. Durante la misión, el dron registra las posiciones aproximadas de las detecciones en coordenadas GPS evitando duplicados cercanos, y finalmente regresa automáticamente a la base para aterrizar de forma segura, mostrando un resumen con todas las localizaciones encontradas.

## Teoría y Funcionamiento

### Conversión de Coordenadas (GPS-UTM-Local)

Para que el dron pueda relacionar sus movimientos en el simulador con posiciones reales sobre el terreno, se utiliza un sistema de conversión entre **coordenadas locales**, **UTM (Universal Transverse Mercator)** y **GPS (latitud y longitud)**.  

El sistema local del dron se basa en el punto de despegue, donde `(0, 0)` representa la posición inicial. A partir de ahí, los desplazamientos en metros se transforman a coordenadas **UTM**, que permiten expresar la posición en un sistema cartográfico métrico. Finalmente, estas coordenadas UTM se convierten a **latitud y longitud**, proporcionando una estimación aproximada en grados decimales del lugar donde se ha detectado cada persona.

Para realizar estas transformaciones, se parte de un punto de referencia (la posición GPS del barco base) y se aplican conversiones proporcionales teniendo en cuenta la distancia en metros por grado de latitud y longitud. Así, el dron puede registrar y mostrar en consola las coordenadas GPS estimadas de cada detección como si se tratara de una misión real de búsqueda aérea.

### Máquina de Estados Finita (FSM)

El comportamiento completo del dron se organiza mediante una **Máquina de Estados Finita (FSM)** que controla de forma ordenada cada fase de la misión.  
Cada estado representa una etapa del vuelo y define las acciones que el dron debe realizar, así como las condiciones para pasar al siguiente estado.  
El flujo principal sigue la siguiente secuencia:

<p align="center">
  <img src="https://github.com/user-attachments/assets/ceb8ed91-1fe7-48db-9cb5-f3ff8f955fb7" width="512" height="768">
</p>



1. **INICIO** → el dron se prepara para despegar.  
2. **DESPEGUE** → asciende hasta alcanzar la altura de vuelo establecida.  
3. **ORIENTAR** → ajusta su orientación (yaw) hacia la posición objetivo.  
4. **NAVEGAR_OBJ** → se dirige hacia el punto de inicio de la búsqueda.  
5. **BUSQUEDA** → ejecuta el patrón de exploración mientras analiza la cámara ventral.  
6. **VUELTA_BASE** → retorna automáticamente al origen al finalizar la misión o alcanzar el límite de detecciones.  
7. **ATERRIZAJE** → realiza el descenso y aterriza con seguridad.  
8. **DONE** → estado final, donde se muestran los resultados y coordenadas de las detecciones.

Gracias a esta estructura modular, el dron mantiene un **comportamiento robusto y predecible**, permitiendo una transición fluida entre tareas críticas sin necesidad de intervención externa.


### Patrón de Búsqueda (Lawnmower)

Para cubrir toda el área de rescate de forma sistemática, el dron utiliza un **patrón de búsqueda tipo cortacésped** (en forma de zigzag).Este patrón garantiza que se inspecciona la totalidad del terreno sin dejar zonas sin explorar.  

El algoritmo genera una lista de **waypoints** que alternan entre movimientos horizontales y descensos verticales en franjas separadas por una distancia fija.  
El dron comienza desde la **esquina superior izquierda del área** y avanza hacia la derecha, desciende una franja y vuelve hacia la izquierda, repitiendo el proceso hasta cubrir toda la zona definida.

Este método es muy eficiente para tareas de búsqueda aérea, ya que proporciona **cobertura completa** con una planificación simple y predecible, optimizando el tiempo de vuelo y reduciendo el solapamiento entre pasadas.

### Detección de Caras (Cámara Ventral)

La detección de personas se realiza mediante la **cámara ventral** del dron, utilizando un **clasificador Haar Cascade** entrenado para el reconocimiento de rostros humanos.  
Durante la fase de búsqueda, el dron toma imágenes desde la vista cenital y aplica este algoritmo de visión artificial para identificar posibles personas dentro del área de rescate.  

Para mejorar la detección, cada cierto número de fotogramas la imagen ventral se **rota ligeramente en varios ángulos** (de -90° a +90°), lo que permite reconocer rostros en diferentes orientaciones.  
Cuando se detecta una o varias caras, el dron guarda su posición actual (x, y) y la convierte en coordenadas **UTM y GPS aproximadas**, registrando el lugar de la detección.  
Además, se aplica un filtro de distancia mínima (5 metros) para **evitar duplicados**, asegurando que cada detección única se contabiliza una sola vez.


### Control del Vuelo (Velocidad, Yaw y Altura)

El control del vuelo se basa en un conjunto de **controladores proporcionales (P)** que ajustan en tiempo real la velocidad y la orientación del dron.  
Estos controladores actúan sobre tres ejes principales:

- **Velocidad lineal (avance):** controlada por la ganancia `k_v`, ajusta la aceleración del dron en función de la distancia al objetivo.  
- **Orientación (yaw):** regulada con `k_yaw`, mantiene el dron alineado con la dirección del waypoint siguiente.  
- **Altura (z):** gestionada por `k_z`, garantiza que el dron conserve una altitud constante (`altura_vuelo`) durante toda la misión.

De esta forma, el dron es capaz de navegar suavemente entre puntos de búsqueda, compensar desviaciones y mantener estabilidad incluso ante pequeñas variaciones en la detección de posición o viento simulado.


### Aterrizaje y Gestión de Batería

Al completar el área de búsqueda o alcanzar el número máximo de detecciones, el dron entra en el estado de **VUELTA A BASE**.  
Durante esta fase, regresa automáticamente al punto de origen siguiendo una trayectoria directa, manteniendo la misma altitud de crucero. Una vez llega a la base, la FSM activa el estado de **aterrizaje**, reduciendo progresivamente la velocidad vertical hasta tocar tierra de forma segura.

También se incluye una simulación de **batería baja**, que fuerza la secuencia “**Batería baja -> Vuelta a base**” para garantizar la seguridad de la misión. Esto se activa cada 8 minutos (apróximadamente lo que dura la batería).  

Finalmente, el sistema imprime en consola un **resumen con las coordenadas GPS de todas las personas detectadas**, cerrando la misión con un “Aterrizaje exitoso”.


## Dificultades Encontradas

Una de las principales dificultades fue programar la práctica en mi ordenador, como este lo he comprado en 2015 y la gpu no es muy potente. Como no tengo la granja activada tuve que pedir ayuda para ejecutarlo para el video. Tuve que programar un poco a ciegas ya que solo podía ver la terminal en mi ordenador, las camaras y gazebo no llegaban a ejecutar.  

También resultó complejo lograr un **vuelo estable durante la búsqueda**, especialmente al combinar la rotación periódica de la cámara ventral con los pequeños ajustes de yaw que realiza el controlador.  
Fue necesario ajustar las ganancias `k_v`, `k_yaw` y `k_z` para evitar oscilaciones y lograr que el dron avanzara de forma suave entre waypoints.

Finalmente, la **detección de rostros** presentó varios retos: el ángulo de la cámara y el movimiento del dron afectaban al rendimiento del clasificador Haar. Lo solucioné rotando la imagen en distintos ángulos y filtrando detecciones cercanas para reducir falsos positivos.



## Video

Lo tuve que grabar con el ordenador de otra persona ya que mi ordenador no tiene la capacidad suficiente ejecutar la práctica, por eso hay una subida de calidad respecto a la práctica anterior.


[![Video de la Práctica 2 - Rescue People](https://img.youtube.com/vi/oX7O_JYJCCk/0.jpg)](https://youtu.be/oX7O_JYJCCk)






