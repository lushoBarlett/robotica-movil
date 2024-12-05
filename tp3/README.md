# TP3

## Setup

Primeramente debe descargar el rosbag que se va a utilizar. Se probó el rosbag Machine Hall 01. Para utilizarlo se lo debe descargar junto con su trayectoria ground truth de [acá](https://docs.openvins.com/gs-datasets.html).
Colocar ambos archivos en la carpeta `data`. Puede utilizar los nombres `ros.bag2` y `ground_truth.csv`
respectivamente o ajustarlos en el archivo `main.cpp`.

Previamente se realizó una calibración de las cámaras con el rosbag cam_checkerboard. El resultado se encuentra en data también.

Finalmente debe compilar el código con los siguientes comandos:

```bash
mkdir build && cd build
cmake ..
make
```

## Overview del sistema

Hay 3 módulos principales:

### `stereoproc.cpp`
Su función es definir procesos o pipelines que hagan una cosa en particular.
Hay 3 definidos.
- `stereo_process_images` procesa 2 imágenes estéreo para triangular features y estimar
la pose de la cámara derecha a partir de la izquierda.
- `monocular_process_images` procesa 2 imágenes de la cámara izquierda. Estima el desplazamiento de ella.
- `stereo_process_images_dense` procesa 2 imágenes estéreo triangulando puntos de manera densa y mostrando
el mapa de disparidad.

### `mapping_node.cpp`
Define un nodo de ros2. Lee los mensajes de un rosbag y a medida que obtiene información de las 2 cámaras
la utiliza para triangular puntos de manera densa o simplemente los features. Además estima la pose de la cámara derecha a partir de la izquierda y publica todas las transformaciones.

### `mapping_node.cpp`
Define un nodo de ros2. Lee los mensajes de un rosbag y a partir de las 2 últimas imágenes de la cámara
izquierda, calcula el desplazamiento de ella usando `monocular_process_images`. Publica las transformaciones y además escribe las poses en `data/trajectory_estimation/body.csv` y `data/trajectory_estimation/cam.csv`. Estos archivos se pueden plottear utilizando el script de python `plot_poses.py`.

## Ejecución

Para poder correr el programa, se debe ejecutar lo siguiente:

```bash
./compvision -h
```

Esto mostrará las opciones de ejecución disponibles.

Por ejemplo:

```bash
./compvision 1
```

Los resultados se pueden visualizar con rviz, se publican las nubes de puntos y también las transformaciones. Además con opencv se muestran los matches en otra ventana.

### Debugging

Para debuggear o verificar los matches se puede utilizar la señal `SIGTSTP` o `ctrl+z` en la terminal.

Al enviar la señal el proceso se detendrá para poder visualizar mejor la imagen de los matches o lo que
haga falta.

Presionando cualquier tecla en la ventana donde se dibujan los matches se puede avanzar de
a una iteración.

Si se desea volver al modo normal se puede volver a mandar la misma señal al proceso
y al presionar alguna tecla en la ventana de los matches volverá a correr normalmente.

## Usando Docker

Para poder correr la imagen ejecutar los siguientes comandos:

```bash
xhost +local:docker
```

```bash
sudo docker build -t compvision .
```

Puede cambiar el número del final para probar otros modos de ejecución.

```bash
sudo docker run --rm -it --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="<path>/robotica-movil/tp3/data:/usr/src/app/data" compvision 1
```

Se puede debuggear de la misma manera comentada anteriormente.
