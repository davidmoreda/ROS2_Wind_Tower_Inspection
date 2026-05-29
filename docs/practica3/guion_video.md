# Guion completo del video - Practica 3 (Manipulacion con MoveIt 2)

Texto **palabra por palabra** para leer en voz alta. **Todo esta ya ejecutado de antemano**:
en el video NO se lanzan comandos en terminal. Cada bloque indica **a donde enfocar** la pantalla
**(ENFOCAR: ... en negrita)** y el texto a narrar.

> Antes de empezar: sustituir `[Nombre1]`, `[Nombre2]`, `[Nombre3]` por los nombres reales.
> Leer despacio, vocalizando. Pausa de 1 segundo entre escenas.
> Mover el raton lento y senalar con el cursor lo que se nombra.
> Alcance: el video cubre SOLO la manipulacion con MoveIt 2 (modelo del brazo, configuracion,
> planificacion articular y cartesiana). La navegacion, la vision y el comportamiento del TFM no
> se tratan aunque convivan en la rama.

---

## 0:00 - 0:20 -- Intro `[P1]`

**ENFOCAR: RViz (o Gazebo) con el Husky y el brazo UR5e visibles en el centro.**

> "Bueno, pues seguimos con la Practica 3, y aqui llega la guinda del proyecto. Esta es la parte
> donde el robot deja de limitarse a moverse por el suelo y empieza a trabajar de verdad. Os vamos
> a ensenar como nuestro brazo robotico planifica y ejecuta movimientos de forma autonoma con
> MoveIt 2, el mismo estandar que se usa hoy en la industria y en los laboratorios de robotica
> punteros. Dentro video."

---

## 0:20 - 0:50 -- Justificacion del brazo UR5e `[P1]`

**ENFOCAR: el robot en RViz; senalar el brazo con el cursor.**

> "El enunciado proponia un ABB IRB-120, pero nosotros hemos ido un paso mas alla: montamos un
> UR5e, un brazo colaborativo de Universal Robots, exactamente el tipo de brazo que ahora mismo
> esta desplegado en fabricas de medio mundo. Y no va suelto: lo llevamos integrado encima del
> Husky, formando una unica plataforma movil y manipuladora. Seis articulaciones, alcance de sobra
> y precision de nivel industrial, todo sobre la misma base con la que hemos hecho las practicas
> anteriores. Es el corazon de nuestra mision de inspeccion de torres eolicas."

---

## 0:50 - 1:20 -- Configuracion de MoveIt `[P1]`

**ENFOCAR: RViz con el panel de planificacion de movimiento visible.**

> "Detras de cada movimiento hay un stack de configuracion completo, montado por nosotros de cero.
> Tenemos el solucionador de cinematica inversa, que calcula como colocar cada articulacion para
> alcanzar un punto en el espacio; los limites de seguridad de cada junta; los controladores que
> ejecutan las trayectorias en el robot; y no uno, sino varios planificadores: uno para trayectorias
> libres esquivando obstaculos y otro para movimientos perfectamente rectilineos. Y como extra,
> hemos anadido nuestro propio nodo de inspeccion, pensado ya para la aplicacion real del TFM."

---

## 1:20 - 2:10 -- Modelo y configuracion semantica `[P2]`

**ENFOCAR: RViz con el robot; senalar las articulaciones del brazo al nombrarlas.**

> "El robot entero, Husky mas UR5e, se genera al vuelo en tiempo de ejecucion, y sobre el
> construimos toda su 'inteligencia' de movimiento. Le decimos al sistema que articulaciones forman
> el brazo, le damos poses predefinidas listas para usar, como reposo y posicion de trabajo, y
> generamos una matriz de colisiones afinada: el robot sabe que partes de su propio cuerpo pueden
> estar en contacto y cuales no, asi que planifica mas rapido y nunca se choca consigo mismo. En
> resumen, el robot conoce su propia anatomia al milimetro."

---

## 2:10 - 3:00 -- Carga del robot en RViz `[P2]`

**ENFOCAR: Gazebo + RViz lado a lado, con el robot ya cargado y quieto.**

> "Aqui lo teneis: levantamos la simulacion, el robot aparece firme y estable en el mundo, y al
> instante arranca MoveIt con su entorno de planificacion en RViz. Fijaos en que todo encaja a la
> primera, con la cinematica perfectamente alineada. Desde este panel tomamos el control del brazo
> y, a partir de aqui, todo lo que veais lo decide y lo ejecuta el robot por si mismo."

---

## 3:00 - 4:00 -- Planificacion: pose home `[P3]`

**ENFOCAR: clip en RViz del brazo moviendose a la pose home, con la camara apuntando al frente,
en sincronia con el robot en Gazebo.**

> "Hemos creado dos planificaciones para el brazo. La primera es la pose home. Es la posicion base:
> el brazo se coloca con la camara apuntando recta hacia la pared del tubo. Es la pose de partida y
> de reposo, desde la que arranca y a la que vuelve cualquier inspeccion. Seleccionamos la pose,
> planificamos, y al ejecutar el brazo se mueve hasta esa posicion en sincronia con el robot
> simulado."

---

## 4:00 - 5:00 -- Planificacion: barrido (sweep) `[P3]`

**ENFOCAR: clip del brazo haciendo el barrido alrededor de la pose base, girando paso a paso, y
volviendo a home; zoom opcional sobre el efector.**

> "La segunda planificacion es el barrido, o sweep. Partiendo de la pose home, el brazo recorre una
> serie de orientaciones girando paso a paso, en torno a 45 grados cada vez, para barrer la
> superficie de la pared, y al terminar vuelve a home. De esta forma la camara cubre toda la zona a
> inspeccionar de manera sistematica. Igual que antes, lo planificamos y lo ejecutamos, y el barrido
> completo se ve en RViz y en Gazebo."

---

## 5:00 - 5:30 -- Cierre y discrepancias `[P1]`

**ENFOCAR: vuelve al robot en RViz o Gazebo (plano de cierre).**

> "Y con esto lo tenemos: hemos cargado el robot, seleccionado el brazo, y planificado y ejecutado
> tanto movimientos articulares como trayectorias cartesianas milimetricas, cubriendo todas las
> pruebas del enunciado y subiendo el liston con un brazo colaborativo de nivel industrial integrado
> en una plataforma movil. Y esto es solo una pieza de un proyecto mucho mas grande: un robot capaz
> de navegar, manipular e inspeccionar torres eolicas de forma autonoma. Muchas gracias, nos vemos
> en el siguiente."

---

## Resumen de quien dice que

| Persona | Escenas que narra |
| --- | --- |
| **P1** | Intro, Justificacion UR5e, Configuracion de MoveIt, Cierre |
| **P2** | Modelo y configuracion semantica, Carga en RViz |
| **P3** | Planificacion pose home, Planificacion barrido (sweep) |

**Duracion total estimada:** ~5 min 30 s.

---

## Artefactos que deben existir antes de grabar

Como todo se enfoca ya hecho, estos clips/capturas tienen que estar listos:

| Escena | Necesita |
| --- | --- |
| Carga en RViz | captura del robot cargado y quieto (antes de planificar) |
| Planificacion pose home | clip: seleccionar home -> planificar -> ejecutar (RViz + Gazebo) + captura del brazo en home |
| Planificacion barrido (sweep) | clip: ejecutar el barrido completo home -> giros -> home + captura del recorrido |
