# TP Final: GLOMAP

Código utilizado para el cálculo de métricas y creación de gráficos a partir de las estimaciones de GLOMAP y COLMAP.

## Datos

En la carpeta `data` se encuentran los datos procesados de las estimaciones de GLOMAP y COLMAP, en formato de COLMAP, para las secuencias obtenidas a partir de los datasets Machine Hall 01 y Rosario Dataset v2. Además, allí se encuentran los _ground truths_ correspondientes a las mismas secuencias, en formato: `timestamp x y z qx qy qz qw`.

En la carpeta `evo_data` se encuentran las mismas trayectorias estimadas pero en el formato `tum` de `evo`.

## Algoritmos de alineamiento

En el archivo `aligners.py` se implementan los algoritmos de alineamiento RANSAC, Kabsch-Umeyama (al cual se llamó Kabsch) y Umeyama con _scaling_ (al cual se llamó Umeyama). Para RANSAC se utiliza la implementación de OpenCV.

## Métricas

Se calculan 2 medidas de error para la posición: APE (Absolute Pose Error) y AUC (Area Under the recall Curve). Además, se puede experimentar con otras, como RPE (Relative Pose Error) utilizando `evo`.

## Uso

Instalar las dependencias con: `pip install -r requirements.txt`.

Primero crear la carpeta `plots` donde se guardarán los gráficos: `mkdir plots`.

Se debe correr el archivo `main.py` con los siguientes argumentos:

- Para seleccionar un dataset: `--dataset {rd, mh01}`
- Para seleccionar la estimación de un algoritmo: `--method {glomap, colmap}`
- Para pasar una lista de thresholds para la métrica AUC: `--thresholds THRESHOLDS`
- Para seleccionar un método de alineamiento: `--alignment {ransac_inliers,ransac_all,kabsch,umeyama}`
- Para graficar las trayectorias, guardándolas en un archivo (opcional): `--save_plot FILENAME`

A continuación se muestran algunos ejemplos:

```shell
python3 main.py --dataset mh01 --method colmap --thresholds 0.5 1 2 3 --alignment ransac_inliers --save_plot plot.png
python3 main.py --dataset rd --method glomap --thresholds 0.5 1 2 3 4 --alignment umeyama
```

Además se puede utilizar el paquete [evo](https://github.com/MichaelGrupp/evo/) para calcular métricas de la siguiente manera:

```shell
evo_ape tum data/{dataset}.gt.txt evo_data/{dataset}.{method}.txt --plot --verbose -r {trans_part, angle_deg} --align --correct_scale
```

A continuación se muestran algunos ejemplos:

```shell
evo_ape tum data/rd.gt.txt evo_data/rd.glomap.txt --plot --verbose -r trans_part --align --correct_scale
evo_ape tum data/rd.gt.txt evo_data/rd.colmap.txt --plot --verbose -r trans_part --align --correct_scale
evo_ape tum data/mh01.gt.txt evo_data/mh01.glomap.txt --plot --verbose -r trans_part --align
```

## Uso con Docker

Para construir la imagen:
```shell
docker build -t glomap_metric .
```

Crear la carpeta `plots` para poder montarla al contenedor: `mkdir plots`.

Para correrlo, ejecutar comandos como el siguiente, pasando los argumentos de manera adecuada:
```shell
docker run --rm -v ./plots:/app/plots glomap_metric ARGS
```

A continuación se muestran algunos ejemplos:
```shell
docker run --rm -v ./plots:/app/plots glomap_metric --dataset rd --method glomap --alignment umeyama --thresholds 1 2 3 --save_plot myplot.png
docker run --rm -v ./plots:/app/plots glomap_metric --dataset mh01 --method colmap --alignment ransac_inliers --thresholds 0.5 1
```

Para correr `evo`. Entrar al contenedor, montar la carpeta `plots` para acceder a los resultados desde fuera, y ejecutar el comando de evo. A continuación se muestra un ejemplo:
```shell
docker run -v ./plots:/app/plots -it --entrypoint /bin/bash glomap_metric
evo_ape tum data/rd.gt.txt evo_data/rd.glomap.txt --verbose -r trans_part --align --correct_scale --save_plot plots/plot.png
```
