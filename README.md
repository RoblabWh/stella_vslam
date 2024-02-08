# stella_vslam_dense
Innovate your UAV-based USAR missions with our real-time dense 3D reconstruction method tailored for common 360° action cams. Building upon StellaVSLAM and ORB-SLAM, our approach introduces a PatchMatch-Stereo thread for dense correspondences, extending robust long-term localization on equirectangular video input. This GitHub repository hosts our novel massively parallel variant of the PatchMatch-Stereo algorithm, optimized for low latency and supporting the equirectangular camera model. Experience improved accuracy and completeness on consumer-grade laptops with recent mobile GPUs, surpassing traditional offline Multi-View-Stereo solutions in real-time dense 3D reconstruction tasks.

<b>Website:</b> https://roblabwh.github.io/stella_vslam_dense/ \
<b>Original Paper:</b> [PatchMatch-Stereo-Panorama, a fast dense reconstruction from 360° video images](https://arxiv.org/abs/2211.16266)

<img width="100%" src="demo.gif"/>

## Limitations of Dense Feature
- Only monocular equirectangular input is supported.
- Only msgpack format is supported.
- Visualization is limited to using socket_publisher exclusively.

## Dependencies
- Docker [(installation guide)](https://docs.docker.com/engine/install/)
- Nvidia Docker [(installation guide)](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Installation
```
git clone --recursive https://github.com/RoblabWh/stella_vslam_dense.git
cd stella_vslam_dense
docker build -t stella_vslam_dense -f Dockerfile.socket . --build-arg NUM_THREADS=$(nproc)
```

## Running
Start VSLAM container
```
docker run -it --rm --gpus all --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 -p 3001:3001 --name=stella_vslam_dense -v ${HOST_DATA_PATH:?Set path to directory on the host system to keep input and output data.}:/data stella_vslam_dense
```
Open the viewer in a browser: http://localhost:3001

Run VSLAM in container
```
./run_video_slam -v /data/orb_vocab.fbow -c "/data/${PATH_TO_CONFIG:?}" -m "/data/${PATH_TO_VIDEO:?}" --mask "/data/${PATH_TO_MASK:?}"
```
More options are available
```
./run_video_slam -h
```

### Running our examples
We present two illustrative examples to evaluate our method. The first utilizes a high-definition (HD) equirectangular video and boasts real-time processing capabilities. In contrast, the second example employs a 5.7k video, yielding a point cloud of higher quality. To execute the first example seamlessly, a computer equipped with 16GB of RAM and an NVIDIA graphics card is required. However, for the second example, a more robust system with 128GB of RAM (in addition to an NVIDIA graphics card) is necessary. While using swap space is an option, it significantly compromises performance speed.

#### HD / Real time example
Prepare dataset in container
```
mkdir -p /data/example_inspection_flight_drz_keyframes_hd

curl -L "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o /data/orb_vocab.fbow

curl -u "dF2wQbFNW2zuCpK:fire" "https://w-hs.sciebo.de/public.php/webdav/stella_vslam_dense/example_inspection_flight_drz_hd.mp4" -o /data/example_inspection_flight_drz_hd.mp4
```

Run VSLAM in container
```
./run_video_slam -v /data/orb_vocab.fbow -c /stella_vslam/example/dense/dense_hd.yaml -m /data/example_inspection_flight_drz_hd.mp4 --frame-skip 3 -o /data/example_inspection_flight_drz_hd.msg -p /data/example_inspection_flight_drz_hd.ply -k /data/example_inspection_flight_drz_keyframes_hd/
```

#### High quality example
Prepare dataset in container
```
mkdir -p /data/example_inspection_flight_drz_keyframes

curl -L "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o /data/orb_vocab.fbow

curl -u "dF2wQbFNW2zuCpK:fire" "https://w-hs.sciebo.de/public.php/webdav/stella_vslam_dense/example_inspection_flight_drz.mp4" -o /data/example_inspection_flight_drz.mp4
```

Run VSLAM in container
```
./run_video_slam -v /data/orb_vocab.fbow -c /stella_vslam/example/dense/dense.yaml -m /data/example_inspection_flight_drz.mp4 --frame-skip 3 -o /data/example_inspection_flight_drz.msg -p /data/example_inspection_flight_drz.ply -k /data/example_inspection_flight_drz_keyframes/
```

## Additional export scripts
Exporting point cloud from msgpack to ply
```
/stella_vslam/scripts/export_dense_msg_to_ply.py -i ${PATH_TO_MSGPACK:?} -o ${PATH_TO_PLY:?}
```
Exporting the project from msgpack for use with [nerfstudio](https://docs.nerf.studio/) or [instant-ngp](https://github.com/NVlabs/instant-ngp)
```
/stella_vslam/scripts/export_dense_msg_to_nerf.py ${PATH_TO_MSGPACK:?} ${PATH_TO_VIDEO:?} ${PATH_TO_OUTPUT:?}
```

## Citation of original PatchMatch integration for OpenVSLAM
```
@INPROCEEDINGS{surmann2022,
  author={Surmann, Hartmut and Thurow, Marc and Slomma, Dominik},
  booktitle={2022 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  title={PatchMatch-Stereo-Panorama, a fast dense reconstruction from 360° video images},
  year={2022},
  volume={},
  number={},
  pages={366-372},
  doi={10.1109/SSRR56537.2022.10018698}}
```
The paper can be found [here](https://arxiv.org/abs/2211.16266).\
The code can be found [here](https://github.com/RoblabWh/PatchMatch).


## Further reading
<details>
<summary>
This part is from the original StellaVSLAM readme, which remains fully functional for non-dense operations. Notably, the documentation related to the configuration YAML file is entirely applicable. It's important to emphasize that when utilizing the dense feature, only the socket_publisher, msgpack format, and monocular equirectangular inputs are supported.
</summary>

# stella_vslam

[![CI](https://github.com/stella-cv/stella_vslam/actions/workflows/main.yml/badge.svg)](https://github.com/stella-cv/stella_vslam/actions/workflows/main.yml)
[![Documentation Status](https://readthedocs.org/projects/stella-cv/badge/?version=latest)](https://stella-cv.readthedocs.io/en/latest/?badge=latest)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

---

> *NOTE:* This is a community fork of [xdspacelab/openvslam](https://github.com/xdspacelab/openvslam). It was created to continue active development of OpenVSLAM on Jan 31, 2021. The original repository is no longer available. Please read the [official statement of termination](https://github.com/xdspacelab/openvslam/wiki/Termination-of-the-release) carefully and understand it before using this. The similarities with ORB_SLAM2 in the original version have been removed by [#252](https://github.com/stella-cv/stella_vslam/pull/252). If you find any other issues with the license, please point them out. See [#37](https://github.com/stella-cv/stella_vslam/issues/37) and [#249](https://github.com/stella-cv/stella_vslam/issues/249) for discussion so far.

*Versions earlier than 0.3 are deprecated. If you use them, please use them as a derivative of ORB_SLAM2 under the GPL license.*

---

## Overview

[<img src="https://raw.githubusercontent.com/stella-cv/docs/main/docs/img/teaser.png" width="640px">](https://arxiv.org/abs/1910.01122)

<img src="https://j.gifs.com/81m1QL.gif" width="640px">

[**[PrePrint]**](https://arxiv.org/abs/1910.01122)

stella_vslam is a monocular, stereo, and RGBD visual SLAM system.

### Features

The notable features are:

- It is compatible with **various type of camera models** and can be easily customized for other camera models.
- Created maps can be **stored and loaded**, then stella_vslam can **localize new images** based on the prebuilt maps.
- The system is fully modular. It is designed by encapsulating several functions in separated components with easy-to-understand APIs.
- We provided **some code snippets** to understand the core functionalities of this system.

One of the noteworthy features of stella_vslam is that the system can deal with various type of camera models, such as perspective, fisheye, and equirectangular.
If needed, users can implement extra camera models (e.g. dual fisheye, catadioptric) with ease.
For example, visual SLAM algorithm using **equirectangular camera models** (e.g. RICOH THETA series, insta360 series, etc) is shown above.

We provided [documentation](https://stella-cv.readthedocs.io/) for installation and tutorial.
The repository for the ROS wrapper is [stella_vslam_ros](https://github.com/stella-cv/stella_vslam_ros).

### Acknowledgements

OpenVSLAM is based on an indirect SLAM algorithm with sparse features, such as [ORB-SLAM](https://arxiv.org/abs/1502.00956)/[ORB-SLAM2](https://arxiv.org/abs/1610.06475), [ProSLAM](https://arxiv.org/abs/1709.04377), and [UcoSLAM](https://arxiv.org/abs/1902.03729).
The core architecture is based on ORB-SLAM/ORB-SLAM2 and the code has been redesigned and written from scratch to improve scalability, readability, performance, etc.
UcoSLAM has implemented the parallelization of feature extraction, map storage and loading earlier.
ProSLAM has implemented a highly modular and easily understood system earlier.

### Examples

Some code snippets to understand the core functionalities of the system are provided.
You can employ these snippets for in your own programs.
Please see the `*.cc` files in `./example` directory or check [Simple Tutorial](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) and [Example](https://stella-cv.readthedocs.io/en/latest/example.html).

## Installation

Please see [**Installation**](https://stella-cv.readthedocs.io/en/latest/installation.html) chapter in the [documentation](https://stella-cv.readthedocs.io/).

[**The instructions for Docker users**](https://stella-cv.readthedocs.io/en/latest/docker.html) are also provided.

## Tutorial

Please see [**Simple Tutorial**](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) chapter in the [documentation](https://stella-cv.readthedocs.io/).

A sample ORB vocabulary file can be downloaded from [here](https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow).
Sample datasets are also provided at [here](https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4).

If you would like to run visual SLAM with standard benchmarking datasets (e.g. KITTI Odometry dataset), please see [**SLAM with standard datasets**](https://stella-cv.readthedocs.io/en/latest/example.html#slam-with-standard-datasets) section in the [documentation](https://stella-cv.readthedocs.io/).

## Community

Please contact us via [GitHub Discussions](https://github.com/stella-cv/stella_vslam/discussions) if you have any questions or notice any bugs about the software.

## Currently working on

- Refactoring
- Algorithm changes and parameter additions to improve performance
- Add tests
- Marker integration
- Implementation of extra camera models
- Python bindings
- IMU integration

The higher up the list, the higher the priority.
Feedbacks, feature requests, and contribution are welcome!

## License

**2-clause BSD license** (see [LICENSE](./LICENSE))

The following files are derived from third-party libraries.

- `./3rd/json` : [nlohmann/json \[v3.6.1\]](https://github.com/nlohmann/json) (MIT license)
- `./3rd/spdlog` : [gabime/spdlog \[v1.3.1\]](https://github.com/gabime/spdlog) (MIT license)
- `./3rd/tinycolormap` : [yuki-koyama/tinycolormap](https://github.com/yuki-koyama/tinycolormap) (MIT license)
- `./src/stella_vslam/solver/pnp_solver.cc` : part of [laurentkneip/opengv](https://github.com/laurentkneip/opengv) (3-clause BSD license)
- `./src/stella_vslam/feature/orb_extractor.cc` : part of [opencv/opencv](https://github.com/opencv/opencv) (3-clause BSD License)
- `./src/stella_vslam/feature/orb_point_pairs.h` : part of [opencv/opencv](https://github.com/opencv/opencv) (3-clause BSD License)

Please use `g2o` as the dynamic link library because `csparse_extension` module of `g2o` is LGPLv3+.

## Authors of the original version of OpenVSLAM

- Shinya Sumikura ([@shinsumicco](https://github.com/shinsumicco))
- Mikiya Shibuya ([@MikiyaShibuya](https://github.com/MikiyaShibuya))
- Ken Sakurada ([@kensakurada](https://github.com/kensakurada))

## Citation of original version of OpenVSLAM

OpenVSLAM **won first place** at **ACM Multimedia 2019 Open Source Software Competition**.

If OpenVSLAM helps your research, please cite the paper for OpenVSLAM. Here is a BibTeX entry:

```
@inproceedings{openvslam2019,
  author = {Sumikura, Shinya and Shibuya, Mikiya and Sakurada, Ken},
  title = {{OpenVSLAM: A Versatile Visual SLAM Framework}},
  booktitle = {Proceedings of the 27th ACM International Conference on Multimedia},
  series = {MM '19},
  year = {2019},
  isbn = {978-1-4503-6889-6},
  location = {Nice, France},
  pages = {2292--2295},
  numpages = {4},
  url = {http://doi.acm.org/10.1145/3343031.3350539},
  doi = {10.1145/3343031.3350539},
  acmid = {3350539},
  publisher = {ACM},
  address = {New York, NY, USA}
}
```

The preprint can be found [here](https://arxiv.org/abs/1910.01122).

## Reference

- Raúl Mur-Artal, J. M. M. Montiel, and Juan D. Tardós. 2015. ORB-SLAM: a Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics 31, 5 (2015), 1147–1163.
- Raúl Mur-Artal and Juan D. Tardós. 2017. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics 33, 5 (2017), 1255–1262.
- Dominik Schlegel, Mirco Colosi, and Giorgio Grisetti. 2018. ProSLAM: Graph SLAM from a Programmer’s Perspective. In Proceedings of IEEE International Conference on Robotics and Automation (ICRA). 1–9.
- Rafael Muñoz-Salinas and Rafael Medina Carnicer. 2019. UcoSLAM: Simultaneous Localization and Mapping by Fusion of KeyPoints and Squared Planar Markers. arXiv:1902.03729.
- Mapillary AB. 2019. OpenSfM. <https://github.com/mapillary/OpenSfM>.
- Giorgio Grisetti, Rainer Kümmerle, Cyrill Stachniss, and Wolfram Burgard. 2010. A Tutorial on Graph-Based SLAM. IEEE Transactions on Intelligent Transportation SystemsMagazine 2, 4 (2010), 31–43.
- Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, and Wolfram Burgard. 2011. g2o: A general framework for graph optimization. In Proceedings of IEEE International Conference on Robotics and Automation (ICRA). 3607–3613.

</details>
