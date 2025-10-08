## Índice
<details>
<summary>Práctica 1</summary>

- [Objetivo](#objetivo)
- [Teoría](#teoría)
  * [Wave Front Algorithm](#wave-front-algorithm)
  * [Expansión de Obstáculos](#expansión-de-obstáculos)
  * [Métodos de Navegación](#métodos-de-navegación)
- [Funcionalidad del Código](#funcionalidad-del-código)
  * [Cálculo del Mapa de Coste](#cálculo-del-mapa-de-coste)
  * [Expansión de Costes en Obstáculos](#expansión-de-costes-en-obstáculos)
  * [Navegación Basada en Gradiente](#navegación-basada-en-gradiente)
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


