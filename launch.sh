#!/bin/bash
echo "starting stella vslam dense"
docker run \
  -it --rm --ipc=host \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -p 3001:3001 \
  --name=stella_vslam_dense \
  -v $(pwd)/../dataset/test_data/orb_vocab.fbow:/orb_vocab.fbow \
  -v $(pwd)/../dataset/tube/mask.png:/tube/mask.png \
  -v $(pwd)/../dataset/video_drz.mp4:/video.mp4  \
  -v $(pwd)/../dataset/test_data/equirectangular.yaml:/config.yaml \
  -v $(pwd)/../dataset/output:/output/ \
  stella_vslam_dense
