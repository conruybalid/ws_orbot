import torch
print(torch.cuda.is_available())  # Should return True if CUDA is available
print(torch.cuda.get_device_name(0))  # Should return the name of your GPU

import torchvision

print(f"PyTorch version: {torch.__version__}")
print(f"torchvision version: {torchvision.__version__}")