# Guion completo del video - Practica 1

Texto **palabra por palabra** para leer en voz alta. **Todo esta ya ejecutado de antemano**:
en el video NO se lanzan comandos en directo. Cada bloque indica **a donde enfocar** la pantalla
**(ENFOCAR: ... en negrita)** y el texto a narrar.

> Antes de empezar: sustituir `[Nombre1]`, `[Nombre2]`, `[Nombre3]` por los nombres reales.
> Leer despacio, vocalizando. Hacer una pausa de 1 segundo entre escenas.
> Mover el raton lento y senalar con el cursor el dato del que se habla.

---

## 0:00 - 0:15 -- Intro `[P1]`

**ENFOCAR: la ventana de Gazebo con el mundo cargado (es el plano de fondo durante toda la intro).**

> "Hola, somos David, Dani y yo Javi. En este video presentamos la Practica 1
> de Robotica.
> A lo largo del video
> vamos a mostrar cuatro cosas: el mundo simulado, el robot y sus sensores, la integracion
> con ROS 2, y un experimento de fisica. Empezamos."


---

## 0:30 - 1:15 -- Mundo Gazebo (20%) `[P1]`

**ENFOCAR: la ventana de Gazebo Sim (ya cargada). Senalar con el cursor la torre,
los obstaculos y el suelo segun los vas nombrando.**

> "En la industria, muchas empresas inspeccionan tubos eolicos en busca de posibles fallos, y
> ese es justo el escenario que hemos recreado. Esto es el mundo, definido en gacebo. Representa una nave industrial pensada para el mantenimiento de tubos
> eolicos. En el centro tenemos un tramo de 30 m de tubo apoyado sobre rodillos, con sus rampas
> de acceso y barreras de seguridad alrededor. A los lados hay dos zonas de trabajo: una estacion
> de carga, donde el robot se recarga, y una estacion de mantenimiento. Tambien hay personas en
> movimiento por la nave, que actuan como obstaculos dinamicos. Ademas hemos anadido iluminacion
> realista."

---

## 1:15 - 2:00 -- Robot (25%) `[P2]`

**ENFOCAR: RViz con el RobotModel y las TF visibles (Gazebo al lado si cabe).
Senalar base_link, las ruedas y el top_plate al nombrarlos.**

> "Pasamos al robot. Es un Husky A200. Su descripcion URDF no la escribimos a mano: se
> genera a partir del fichero `robot.yaml` con el generador de Clearpath, que esta en la
> carpeta `wind_tower_bringup/config`. En RViz podemos ver la cadena cinematica completa:
> el `base_link`, las dos ruedas, izquierda y derecha, el `top_plate` y los distintos
> accesorios montados encima. Fijaos en que la pose inicial es estable: el robot no vibra,
> no penetra el suelo y se mantiene firme. Las transformadas, las TF, se publican
> correctamente y forman un arbol coherente."

---

## 2:00 - 3:00 -- Sensores (20%) `[P2]`

**ENFOCAR: el terminal con la salida del `ros2 topic echo /imu/data` ya impresa.
Senalar con el cursor el campo `linear_acceleration` y `angular_velocity`.**

> "Los sensores estan declarados en `robot.yaml`. Tenemos una IMU, el modelo `phidgets_spatial`,
> y un LiDAR 3D, un Velodyne VLP-16. Aqui esta la salida de la IMU: se ven las aceleraciones
> lineales y las velocidades angulares del giroscopio, asi que la IMU esta publicando."

**ENFOCAR: el terminal con `ros2 topic info /velodyne_points` (o `ros2 topic hz /velodyne_points`)
ya impreso. Senalar el tipo `sensor_msgs/msg/PointCloud2`.**

> "Para el LiDAR usamos solo el 3D: el Velodyne publica una nube de puntos `PointCloud2` en el
> topico `/velodyne_points`. No usamos un LiDAR 2D fisico; cuando necesitamos trabajar en dos
> dimensiones, proyectamos esa nube 3D a un plano. De hecho, nuestro nodo de localizacion toma
> `/velodyne_points`, se queda con una seccion transversal y ajusta un circulo para localizar el
> robot dentro del tubo."

**ENFOCAR: RViz, donde se ve la nube de puntos del Velodyne sobre los obstaculos y el tubo.**

> "En RViz vemos la nube de puntos en tiempo real. El Velodyne barre el entorno y los puntos
> dibujan la geometria del mundo y de la torre, asi que el sensor esta percibiendo correctamente."

---

## 3:00 - 3:45 -- Integracion ROS-Gazebo (20%) `[P3]`

**ENFOCAR: el terminal con la salida de `ros2 topic list` ya impresa.
Ir senalando con el cursor cada topico segun lo nombras.**

> "Ahora la integracion entre ROS 2 y Gazebo. El puente lo gestiona Clearpath
> internamente, asi que no necesitamos escribir nuestro propio `ros_gz_bridge`.
> En esta lista de topicos aparecen los que importan: `/cmd_vel` para enviarle velocidades al robot,
> `/odom` para leer su odometria, `/clock` para el reloj de la simulacion, y los topicos
> de los sensores que acabamos de ver. La comunicacion es bidireccional: ROS manda
> ordenes a Gazebo y Gazebo devuelve los datos de los sensores y la odometria."

---

## 3:45 - 4:15 -- Teleoperacion con mando `[P3]`

**ENFOCAR: el clip grabado del Husky moviendose en Gazebo mientras se maneja con el mando.**

> "Para el control usamos un mando. El nodo `joy` lee el mando y `teleop_twist_joy` traduce
> los movimientos del joystick en mensajes `Twist` que se publican en `/cmd_vel`. Manteniendo
> el boton de activacion, con el joystick hacemos que el robot avance, retroceda y gire.
> Como veis en la grabacion, el robot responde de inmediato. El plugin de traccion diferencial
> de Clearpath convierte ese `Twist` en velocidades para cada rueda, y el movimiento es suave
> y sin latencia apreciable."



---

## 5:30 - 5:50 -- Checklist y cierre `[P1]`

**ENFOCAR: vuelve a la ventana de Gazebo con el mundo y el robot (plano de cierre).**

> "Para terminar, repasamos lo que hemos demostrado y todo queda cubierto: el mundo simulado,
> el robot estable, la IMU, el LiDAR, el topico `/cmd_vel`, la odometria en `/odom`, la
> teleoperacion por teclado y el experimento D1. Todo el codigo esta en la rama `practica-1`
> del repositorio. Gracias por su atencion."

---

## Resumen de quien dice que

| Persona | Escenas que narra |
| --- | --- |
| **P1** | Intro, Nota Husky vs Tracer, Mundo, Checklist y cierre |
| **P2** | Robot, Sensores |
| **P3** | Integracion, Teleop, Experimento D1 |

**Duracion total estimada:** ~5 min 50 s.
