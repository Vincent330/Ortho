# 正射图生成

## · COLMAP重建

### （1）图像特征提取

```
$ DATASET_PATH=/path/to/dataset

$ colmap feature_extractor \
	--database_path $DATASET_PATH/database.db \
	--image_path $DATASET_PATH/images
```

相机内参已知则通过 *ImageReader.camera_params* 传给 COLMAP ，其中 *camera_model* 默认 *SIMPLE_RADIAL*，如：

```
$ colmap feature_extractor \
	--database_path $DATASET_PATH/database.db \
	--image_path $DATASET_PATH/images \
	--ImageReader.camera_model SIMPLE_PINHOLE \
	--ImageReader.camera_params "8141.6649310000003,4088.7069849999998,2744.4657299999999"
```

### （2）图像特征匹配

```
$ colmap exhaustive_matcher \
	--database_path $DATASET_PATH/database.db
```

### （3）稀疏重建

```
$ mkdir $DATASET_PATH/sparse

$ colmap mapper \
	--database_path $DATASET_PATH/database.db \
	--image_path $DATASET_PATH/images \
	--output_path $DATASET_PATH/sparse
```

### （4）地理配准

将 COLMAP 重建结果对齐到enu坐标系下。

```
colmap model_aligner \
    --input_path $DATASET_PATH/sparse/0 \
    --output_path $DATASET_PATH/sparse/enu \
    --database_path  $DATASET_PATH/database.db \
    --ref_is_gps 1 \
    --alignment_type enu \
    --robust_alignment 1 \
    --robust_alignment_max_error 3.0
```

### （5）模型格式转换

COLMAP 稀疏重建后模型默认导出为 bin 文件，通过 *model_converter* 转换成 txt 文件。

```
$ colmap model_converter \
    --input_path $DATASET_PATH/sparse/enu \
    --output_path $DATASET_PATH/sparse \
    --output_type TXT
```

生成文件包括：

*cameras.txt* ：相机内参，如：

```
# Camera list with one line of data per camera:
#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
# Number of cameras: 1
1 PINHOLE 8192 5460 8352.1833208403386 8349.8891564001406 4096 2730
```

*points3D.txt*：稀疏三维点云中三维空间点的信息，如：

```
# 3D point list with one line of data per point:
#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
# Number of points: 124409, mean track length: 7.3650378991873575
91192 123.44432022064703 47.062972718538376 -76.142005611126919 74 72 51 2.1808438076794063 35 944 36 1292 11 4766 10 4590 55 497
40663 209.9986755816106 35.173755586497045 -84.436714988318741 164 161 146 0.61142845101018317 74 5892 64 3986 26 5361 27 5279
...
```

*images.txt* ：相机外参，包含旋转和平移，以及每张图片中特征点在图像中的二维坐标及其对应的三维稀疏点索引，如：

```
# Image list with two lines of data per image:
#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
#   POINTS2D[] as (X, Y, POINT3D_ID)
# Number of images: 173, mean observations per image: 5296.3988439306358
1 0.0689472801192067 0.70560988468229136 -0.7048650158271963 0.022940633507678346 0.34149251662439523 -2.2758206445045892 -8.1782899938016342 1 DJI_20220908100430_0101.JPG
5759.45068359375 4.8832278251647949 -1 5846.328125 5.252079963684082 -1 ...
2 0.066063024981370136 0.70590061746532673 -0.7050147211347646 0.017153366306688324 0.38011396540043579 9.2268881037835175 -7.9378142950761745 1 DJI_20220908100431_0102.JPG
...
```

COLMAP输出的 *images.txt* 中的四元数 *Q* 和平移向量 *T* ，是其定义的相机坐标系下的 *R* 和 *t* ，需要首先进行坐标系的变换：
```math
$R’ = R^T$
$t’ = -R^Tt$
```
## · 输入文件准备

输入文件为 *images_w.txt* ，由 *get_tras.py* 从 *images.txt* 转化，每行分别为 *NAME, QW, QX, QY, QZ, TX, TY, TZ* ，如：

```
DJI_20220908100430_0101 0.06894728011920546 -0.705609884682304 0.704865015827209 -0.02294063350767862 -1.1987342798263498 0.879258405187668 -8.364827837490488
DJI_20220908100431_0102 0.06606302498136993 -0.705900617465329 0.7050147211347668 -0.017153366306688633 10.092607772563841 0.901536081989625 -6.753913497492001
...
```

