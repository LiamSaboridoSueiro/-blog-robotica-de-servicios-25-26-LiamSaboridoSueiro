## Índice
<details>
<summary>Práctica 4</summary>

- [Objetivo](#objetivo)
  * [Mapa, Límites y Conversión de Coordenadas](#mapa-límites-y-conversión-de-coordenadas)
  * [Construcción del Mapa de Obstáculos e Inflado](#construcción-del-mapa-de-obstáculos-e-inflado)
  * [Planificación de Ruta con OMPL](#planificación-de-ruta-con-ompl)
  * [Simplificación de Waypoints](#simplificación-de-waypoints)
  * [Control y Seguimiento de la ruta](#control-y-seguimiento-de-la-ruta)
  * [Máquina de Estados](#máquina-de-estados)
- [Dificultades y Decisiones de Diseño](#dificultades-y-decisiones-de-diseño)
- [Video](#video)

</details>


## Objetivo

El objetivo de esta práctica es montar un robot “estilo Amazon” capaz de moverse por un almacén, encontrar una estantería concreta, recogerla y volver a su punto de partida sin chocarse con nada en el camino.  

La idea es combinar varias piezas: procesar el mapa del almacén, detectar e inflar obstáculos, planificar rutas con OMPL, seguir la trayectoria mediante un controlador sencillo y, por último, coordinar toda la misión con una pequeña máquina de estados.

### Mapa, Límites y Conversión de Coordenadas

Para poder planificar rutas dentro del almacén, lo primero que necesitaba era una forma fiable de relacionar las coordenadas reales del robot con las coordenadas del mapa en formato imagen (en píxeles). El simulador proporciona un mapa en PNG y, por otra parte, medí las esquinas del almazen usando el getPose. A partir de esos datos calculé una transformación lineal que me permitiera ir de coordenadas de mundo a píxeles. Esto es fundamental porque tanto el planificador como el robot operan en metros, mientras que la detección de obstáculos se realiza sobre una matriz de píxeles. Si esta conversión no es precisa, el robot “cree” que está navegando por un espacio diferente al real, generando rutas no válidas o colisiones inesperadas.

Además, esta conversión es necesaria para poder visualizar correctamente las trayectorias en la interfaz gráfica del simulador. Una vez obtenida la escala en ambos ejes, pude transformar cualquier punto de la planificación a su posición correspondiente dentro del mapa y mostrar las rutas generadas en tiempo real. En definitiva, disponer de una conversión consistente entre mundo físico y mapa fue la base que permitió que el resto del sistema funcionase de manera coherente y sincronizada.


### Construcción del Mapa de Obstáculos e Inflado
El siguiente paso fue crear una representación binaria de los obstáculos. Para ello convertí la imagen original a escala de grises y apliqué un umbral para distinguir entre zonas libres y ocupadas. Sin embargo, trabajar únicamente con los obstáculos “tal cual aparecen” puede ser engañoso, ya que el robot tiene un cuerpo físico con volumen y puede colisionar incluso cuando su punto central no está tocando un obstáculo. Para solucionar esto, generé dos versiones infladas del mapa: una para cuando el robot va vacío y otra para cuando vuelve transportando la estantería, considerando así el mayor espacio que ocupa.

La inflación se realizó mediante dilatación morfológica, utilizando un kernel circular cuyo tamaño depende del radio efectivo del robot en cada situación. Esto garantiza que OMPL planifique rutas que mantengan una distancia mínima segura a las estanterías y paredes. Esta técnica es muy común en robótica móvil, ya que simplifica el espacio de planificación a un robot puntual mientras compensa su volumen real mediante la modificación del mapa. En resumen, el inflado es lo que permite que la planificación sea conservadora y evita que el robot pase por pasillos demasiado estrechos o situaciones comprometidas.

### Planificación de Ruta con OMPL
Con el mapa inflado y la conversión correctamente implementada, el siguiente paso fue calcular rutas fiables desde la posición actual del robot hasta el objetivo. Para ello utilicé la librería OMPL (Open Motion Planning Library), configurando un espacio de estados bidimensional que representa únicamente la posición en el plano (x, y). Esta aproximación es válida porque el robot empleado es holonómico, por lo que su orientación no limita sus movimientos. Junto con el espacio de estados, definí un comprobador de validez que consulta el mapa inflado para determinar si un punto es seguro o si colisionaría con un obstáculo.

Para la planificación escogí el algoritmo **RRTConnect**, un planificador rápido y muy adecuado para encontrar rutas en entornos con obstáculos complejos. Tras obtener la ruta inicial, apliqué las herramientas de simplificación de OMPL, como la reducción de segmentos y el suavizado mediante B-Splines, lo que permite obtener caminos más cortos y suaves sin perder seguridad. De esta manera el planificador produce rutas eficientes, comprensibles para el controlador y suficientemente robustas para evitar colisiones durante la navegación real.


### Simplificación de Waypoints
Una vez obtenida la ruta desde OMPL, el resultado suele ser una secuencia muy densa de puntos consecutivos que describen con detalle el camino planificado. Aunque esta representación es adecuada para el análisis, no es ideal para el control del robot, ya que puede generar oscilaciones o cambios frecuentes de dirección. Por este motivo, implementé un proceso de simplificación que reduce la ruta a un conjunto más manejable de waypoints. El criterio principal consiste en mantener únicamente aquellos puntos que se encuentren a una distancia mínima entre sí, garantizando que el robot avance de manera fluida sin perder la forma general del camino.

Este proceso de filtrado no solo ayuda a que el controlador trabaje con señales más estables, sino que también hace que la navegación resulte más natural y predecible. Al eliminar puntos redundantes, el robot se centra en objetivos más amplios y evita reaccionar a ligeras variaciones que no tienen relevancia real en su movimiento. En conjunto, la simplificación permite que la trayectoria tenga un comportamiento más suave y reduce la probabilidad de que el robot se desvíe o tenga dificultades para estabilizar su orientación durante la marcha.

### Control y Seguimiento de la ruta
El seguimiento de los waypoints requiere transformar los objetivos globales en información útil para el robot. Para ello, convertí cada punto objetivo a coordenadas relativas al robot utilizando su posición y orientación actuales. Esta transformación permite evaluar si el waypoint está delante, detrás o a un lado del robot, algo esencial para generar comandos de velocidad coherentes. A partir de estos valores relativos, apliqué un controlador proporcional sencillo: la velocidad lineal depende de la distancia hacia delante, mientras que la velocidad angular depende del desplazamiento lateral del waypoint respecto al eje del robot.

Este enfoque, aunque básico, funciona bien en este tipo de simulaciones y permite que el robot avance de forma continua hacia cada waypoint sin necesidad de modelos complejos. Además, se incluye una lógica adicional para alinear el robot con la estantería antes de levantarla, asegurando que la aproximación final sea precisa y estable. En conjunto, el sistema de control combina sencillez y efectividad, logrando que el robot recorra la trayectoria planificada de manera estable y sin maniobras bruscas.

### Máquina de Estados
Utilicé una máquina de estados finita que divide la misión en etapas claramente diferenciadas. Cada estado se encarga de una parte específica del proceso, lo que permite mantener el control organizado y evitar decisiones ambiguas. La secuencia completa desde el inicio hasta la entrega final de la estantería queda estructurada de la siguiente forma:

- **PLANIFICAR →** el sistema calcula la ruta desde la posición actual del robot hasta el objetivo (ya sea la estantería o el punto de retorno). Aquí se selecciona el mapa inflado adecuado dependiendo de si el robot va vacío o cargado.  
- **NAVEGAR →** el robot sigue los waypoints generados por el planificador, avanzando hacia el objetivo mediante el controlador proporcional.  
- **ALINEAR →** una vez alcanzada la estantería, el robot ajusta su orientación para garantizar que la aproximación final sea precisa antes de ejecutar la acción de levantarla.  
- **REPLANIFICAR →** tras recoger la estantería, el sistema recalcula una nueva ruta, esta vez usando el mapa inflado correspondiente al volumen adicional que ocupa el robot cargado.  
- **FINALIZAR →** cuando el robot llega al punto de origen, detiene su movimiento, desciende la estantería y concluye la misión de forma segura.

## Dificultades y Decisiones de Diseño

Una de las principales dificultades fue conseguir una conversión precisa entre las coordenadas del mundo y las del mapa en píxeles. Cualquier pequeño desfase provocaba rutas erróneas o colisiones, así que tuve que ajustar escalas y límites varias veces hasta que la planificación y la visualización coincidieran. También el inflado de obstáculos requirió tiempo para encontrar un valor adecuado: demasiado inflado bloqueaba la planificación y demasiado poco hacía que el robot pasara demasiado cerca de las estanterías.

El controlador de seguimiento también presentó retos, especialmente para evitar oscilaciones al avanzar entre waypoints. Ajustar las ganancias y asegurar que la orientación del robot fuera correcta antes de levantar la estantería fue más delicado de lo esperado. La fase de alineación, en particular, dependía mucho de que la transformación a coordenadas relativas fuera precisa. En general, estas dificultades ayudaron a refinar la solución y a lograr un comportamiento más estable y predecible del robot.



## Video

Lo tuve que grabar con el ordenador de otra persona ya que mi ordenador no tiene la capacidad suficiente ejecutar la práctica, por eso hay una subida de calidad respecto a la práctica anterior.


**Estantería 1**  
[![Video Estantería 1](https://img.youtube.com/vi/L7cuVNzxEmY/0.jpg)](https://youtube.com/shorts/L7cuVNzxEmY)

**Estantería 2**  
[![Video Estantería 2](https://img.youtube.com/vi/gXQftRaNYyA/0.jpg)](https://youtube.com/shorts/gXQftRaNYyA)

**Estantería 3**  
[![Video Estantería 3](https://img.youtube.com/vi/mZyXeO4c4vs/0.jpg)](https://youtube.com/shorts/mZyXeO4c4vs)




