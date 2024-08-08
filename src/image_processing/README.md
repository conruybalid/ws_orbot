# Nodes

## arm_location_service

This node creates a service that accepts an emtpy call and returns the relative location of the largest apple seen by the arm camera. In the process, it publishes a viewable version of the red mask it used to /masked_image_topic

## zed_location_service

This node creates a service that accepts an emtpy call and returns the absolute location (with respect to the arm base) of the largest apple seen by the zed camera. In the process, it publishes a viewable version of the red mask it used to /zed_mask_topic


# Other Functions

## ImageProcess.py

This function is used to process the image from the arm camera. It applies a red mask and identifies the location of the apple.


# Installing Torch/TorchVision for the Orin GPU on the Jetson

The version of torch and torchvision that yolov5 wants to use is different than what actually works on the jetson. If the torch.hub.load() line in AI_model.py attempts to replace torch and torchvision, follow these instruction to reinstall the working versions.

uninstall both torch and torchvision

```bash
pip uninstall torch
pip uninstall torchvision
```

install Torch 2.0.0

```bash
export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

python3 -m pip install --no-cache-dir $TORCH_INSTALL
```

install TorchVision 0.15.1 from source

```bash
cd ~/vision/
git checkout v0.15.1

python3 setup.py install
```

This may take a little while