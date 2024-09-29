# Ejercicio 8

Se utiliza `ros-humble` para la resolución del ejercicio.

Primero se crea un paquete que contiene un nodo `cylinder_detector_node`. Su función es escuchar `/scan` y publicar en `visualization_marker` los markers representando los cilindros estimados.

También se provee un archivo `launch` para levantar todos los nodos.

## Ejecución Local

1. Instalar los paquetes de `ros-humble` requeridos:

```bash
apt-get update
apt-get install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3
```

2. Setear las variables de entorno:

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

3. Copiar el paquete `cylinder_detector_pkg` a un workspace y compilarlo. No olvidarse de realizar `source` para su instalación.

4. Modificar la variable `world_path` en el archivo `launch.py` con la ruta del archivo `world.model`.

5. Ejecutar el launch:

```bash
ros2 launch launch.py
```

## Ejecución en un container de Docker

1. Instalar xhost para permitir GUI forwarding:

```bash
sudo apt-get install x11-xserver-utils
```

2. Permitir a Docker el acceso al servidor X:

```bash
xhost +local:docker
```

3. Hacer `build` del container:

```bash
docker build -t ros2_cylinder_detector .
```

4. Ejecutar el container:

```bash
docker run -it --rm --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros2_cylinder_detector
```

5. (Opcional) Comandos para mover el robot:

Abrir otra terminal y ejecutar:

```bash
sudo docker exec -it <CONTAINER_ID> /bin/bash
```

Luego una vez dentro del mismo container se pueden publicar órdenes al robot mediante:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

## Configuración de RViz

Una vez corriendo RViz se deben realizar las siguientes configuraciones:

- En `Global Options` setear el `Fixed Frame` con valor `odom`.

- En la ventana display, hacer click en `Add`, luego cambiar a la pestaña `By topic` y hacer
doble click en el campo `Marker` dentro del tópico `/visualization_marker`. Esto mostrará en
el mapa los cilindros calculados.
