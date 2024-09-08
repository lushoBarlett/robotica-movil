# Trabajo práctico 1

## Instalación

Contar con `python3` e instalar los paquetes requeridos.

```shell
pip install -r requirements.txt
```

Descargar el dataset EuRoc disponible [acá](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip). Luego colocar la carpeta mav0 dentro de tp1/ejercicio5.

## Ejecución

En `gen_cam0_ground_truth.py` se calculan las poses de la camara con respecto a la pose de la cámara
inicial y se guardan los datos en `mav0/cam0/pose_data.csv`.

Para ello en el archivo `poses.py` se calculan las poses necesarias para pasar de una pose del body
a la cámara y reinterpretarla en el sistema de coordenadas de la pose de la cámara inicial (c0).

En `plot_cam0.py` se generan imágenes de la trayectoria de c y b con respecto a w.

Para el ejercicio 1 se debe ejecutar:

```shell
python3 gen_cam0_ground_truth.py
```

Para el ejercicio 2 se puede ejecutar el mismo comando con la flag `--ns-to-s`. Esto hará que
se escriban las poses de la cámara con 'timestamps' en segundos con precisión de nanosegundos.

```shell
python3 gen_cam0_ground_truth.py --ns-to-s
```

Para el ejercicio 3 se puede ejecutar:

```shell
python3 plot_cam0.py --num-poses 10 --step 1
```

donde se pueden pasar los argumentos `num-poses` y `step` para mayor control sobre el plot.