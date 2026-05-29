# Guion completo del video - Practica 2 (Navegacion autonoma con NAV2)

Texto **palabra por palabra** para leer en voz alta. **Todo esta ya ejecutado de antemano**:
en el video NO se lanzan comandos en directo. Cada bloque indica **a donde enfocar** la pantalla
**(ENFOCAR: ... en negrita)** y el texto a narrar.

> Antes de empezar: sustituir `[Nombre1]`, `[Nombre2]`, `[Nombre3]` por los nombres reales.
> Leer despacio, vocalizando. Pausa de 1 segundo entre escenas.
> Mover el raton lento y senalar con el cursor el dato del que se habla.
> Alcance: el video cubre SOLO el scope NAV2 (SLAM, EKF, AMCL, costmaps, planner/controller,
> mision de waypoints). El brazo UR5e, la vision YOLO y el comportamiento del TFM no se tratan
> aunque convivan en la rama.

---

## 0:00 - 0:25 -- Intro `[P1]`

**ENFOCAR: la ventana de Gazebo con el mundo cargado (plano de fondo durante la intro).**

> "Hola, somos [Nombre1], [Nombre2] y [Nombre3]. En este video presentamos la Practica 2:
> navegacion autonoma con NAV2 sobre ROS 2 Humble. El robot es el Husky A200 de Clearpath,
> el mismo de la Practica 1. Una aclaracion de alcance: este proyecto forma parte de un Trabajo
> de Fin de Master de inspeccion de torres eolicas, asi que en la rama hay tambien codigo de
> otras partes, el brazo, la percepcion y el comportamiento. En este video hablamos unicamente
> de la navegacion: SLAM, fusion de sensores, localizacion, costmaps, el controlador y una mision
> de waypoints."

---

## 0:25 - 1:00 -- Arquitectura del sistema `[P1]`

**ENFOCAR: el diagrama `architecture_diagram.png` a pantalla completa. Senalar cada capa al nombrarla.**

> "Esta es la arquitectura del stack, organizada en cuatro capas. La primera es Gazebo, que
> simula el robot y publica los sensores. La segunda es la fusion de odometria, que combina la IMU
> y la odometria de las ruedas con un filtro de Kalman extendido en dos dimensiones. La tercera es
> la localizacion: AMCL situa al robot sobre un mapa que hemos generado con SLAM Toolbox. Y la
> cuarta es NAV2, que ejecuta el planificador, el controlador y un arbol de comportamiento. Todo se
> arranca y se coordina con un gestor de ciclo de vida."

---

## 1:00 - 2:00 -- Launchers y mapeo SLAM `[P1]`

**ENFOCAR: RViz con la config de SLAM (y Gazebo al lado si cabe).**

> "Arrancamos primero la simulacion y, sobre ella, el mapeo con SLAM Toolbox. Una nota importante:
> nuestro LiDAR es un Velodyne 3D. Para mapear en 2D, convertimos la nube de puntos del Velodyne
> en un escaneo laser plano, que es lo que utiliza el algoritmo de mapeo."

**ENFOCAR: RViz mostrando el mapa de ocupacion construyendose (o el clip del robot recorriendo el entorno).**

> "Conduciendo el robot por el entorno, el mapa de ocupacion se va construyendo en tiempo real.
> Las zonas libres aparecen en claro y los obstaculos en oscuro. Cuando el mapa esta completo y
> estable, lo guardamos para usarlo despues como base de la localizacion y la navegacion."

---

## 2:00 - 3:00 -- Fusion de sensores con EKF `[P2]`

**ENFOCAR: RViz mostrando la trayectoria/pose filtrada del robot (o directamente el plot del EKF).**

> "Para estimar bien la pose del robot fusionamos dos fuentes: la odometria de las ruedas y la
> IMU. Lo hacemos con un filtro de Kalman extendido en modo dos dimensiones. El estado que
> estimamos es la posicion en X e Y, la orientacion, y las velocidades lineal y angular. El
> resultado es la odometria filtrada que alimenta a la navegacion."

**ENFOCAR: el plot `ekf_filtered_odom.png` (odometria cruda vs filtrada).**

> "En esta grafica comparamos la odometria cruda con la filtrada. La fusion suaviza el ruido y
> corrige la deriva de la odometria diferencial, que es la que mas sufre al patinar las ruedas.
> La salida es una estimacion continua y mas estable, que es justo lo que necesita la localizacion."

---

## 3:00 - 4:00 -- Localizacion con AMCL `[P2]`

**ENFOCAR: RViz con la config de navegacion, mostrando la nube de particulas recien dispersada (`amcl_initial`).**

> "Con el mapa ya generado, la localizacion global la hace AMCL, un filtro de particulas. Al dar
> una estimacion inicial de pose, las particulas se reparten alrededor de esa posicion
> representando la incertidumbre. Cada particula es una hipotesis de donde puede estar el robot."

**ENFOCAR: RViz tras conducir unos segundos, con la nube de particulas ya concentrada (`amcl_converged`).**

> "En cuanto el robot se mueve un poco, AMCL compara las lecturas del LiDAR con el mapa y las
> particulas convergen rapidamente en torno a la pose real. A partir de ahi el sistema corrige
> continuamente la deriva entre el mapa y la odometria. Lo hemos configurado con un minimo de 500
> y un maximo de 2000 particulas, con muestreo adaptativo."

---

## 4:00 - 4:30 -- Costmaps `[P3]`

**ENFOCAR: RViz mostrando el costmap global y el local sobre el mapa (`costmaps_rviz`).**

> "Sobre el mapa estatico NAV2 monta dos mapas de coste. El global cubre todo el escenario y
> sirve para planificar la ruta completa. El local es una ventana movil alrededor del robot, para
> reaccionar a lo inmediato. La capa de obstaculos incorpora lo que detecta el LiDAR, y la capa de
> inflado anade un margen de seguridad alrededor de cada obstaculo, para que el robot no roce las
> paredes ni las personas que se mueven por la nave."

---

## 4:30 - 5:45 -- Tuning iterativo del controlador `[P3]`

**ENFOCAR: la tabla de parametros antes/despues; de fondo o intercalado, el clip del Husky subiendo la rampa.**

> "Esta es una de las contribuciones principales de la entrega. La rampa de acceso al tubo, con
> bastante pendiente, nos obligo a afinar el controlador. Nuestra primera idea fue subir la
> velocidad y la aceleracion para vencer la cuesta. El resultado fue malo: el robot patinaba, la
> odometria daba saltos y AMCL perdia coherencia."

> "La segunda hipotesis fue que el problema no era falta de potencia, sino falta de traccion.
> Asi que cambiamos de estrategia: bajamos las aceleraciones del suavizador a un tercio, alargamos
> la distancia de anticipacion del Pure Pursuit y permitimos una aproximacion mas lenta. Como veis
> en la tabla, con esos cambios el Husky sube la rampa sin patinar y la trayectoria queda suave y
> estable."

---

## 5:45 - 6:30 -- Mision con waypoints `[P3]`

**ENFOCAR: clip de Gazebo + RViz lado a lado durante la mision (`nav_full_run`).**

> "Para validar todo el stack junto lanzamos una mision automatica con una lista de waypoints
> predefinida, que se envian a NAV2 en secuencia. El robot recorre los puntos uno a uno: el
> planificador traza la ruta, el controlador la sigue, y si aparece un obstaculo nuevo en el
> camino, como una de las personas que se mueven por la nave, replanifica y lo esquiva sin detener
> la mision."

---

## 6:30 - 7:00 -- Resultados y metricas `[P3]`

**ENFOCAR: la tabla de metricas a pantalla completa.**

> "Hemos medido varias metricas para cuantificar el comportamiento: el tiempo que tarda AMCL en
> converger, el error cuadratico medio de la trayectoria frente a la referencia, el tiempo medio
> en llegar a una meta, y la mas significativa para nosotros, el numero de abortos del arbol de
> comportamiento en la rampa antes y despues del tuning. Como se ve en la tabla, el tuning del
> controlador elimina practicamente esos abortos. Todos los valores estan detallados en la memoria."

---

## 7:00 - 7:30 -- Problemas resueltos y cierre `[P1]`

**ENFOCAR: vuelve a Gazebo, o a la tabla resumen de problemas.**

> "Para terminar, resumimos tres problemas que tuvimos que resolver. El primero, el agotamiento de
> recursos de la red de comunicaciones por el numero de nodos, que solucionamos ajustando la
> configuracion del middleware. El segundo, una incompatibilidad de calidad de servicio en el
> escaneo del laser, que arreglamos con un pequeno puente que lo republica con la QoS correcta. Y
> el tercero, el tuning del controlador, que ya hemos contado. Con esto cerramos: gracias por su
> atencion."

---

## Resumen de quien dice que

| Persona | Escenas que narra |
| --- | --- |
| **P1** | Intro, Arquitectura, Launchers + SLAM, Problemas + cierre |
| **P2** | EKF, AMCL |
| **P3** | Costmaps, Tuning del controlador, Mision con waypoints, Resultados |

**Duracion total estimada:** ~7 min 30 s.

---

## Artefactos que deben existir antes de grabar

Como todo se enfoca ya hecho, estas imagenes/datos tienen que estar listos:

| Escena | Necesita |
| --- | --- |
| Arquitectura | `figures/architecture_diagram.png` |
| SLAM | mapa `maps/wind_tower.pgm` + clip o captura del mapeo |
| EKF | plot `plots/ekf_filtered_odom.png` |
| AMCL | capturas `amcl_initial` y `amcl_converged` (o clip en vivo grabado) |
| Costmaps | captura `costmaps_rviz` |
| Tuning | tabla antes/despues + clip del Husky en la rampa |
| Mision | clip `nav_full_run` (Gazebo + RViz) |
| Resultados | tabla de metricas |
