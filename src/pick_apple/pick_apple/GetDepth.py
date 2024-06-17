import numpy as np

def GetDepth(depth):
    # Adjust boundaries based on where fruit is when arm centered
    # Note: OpenCV's crop uses start point and width/height, similar to MATLAB's imcrop
    x, y, w, h = 235, 155, 20, 20
    M = depth[y:y+h, x:x+w]

    # Find the 5 highest values
    max_vals = np.partition(M.flatten(), -5)[-5:]
    d = np.mean(max_vals)

    # Adjusting the distance calculation
    if d*0.001-0.13 < 0:
        # Find the 5 lowest values if the condition is met
        min_vals = np.partition(M.flatten(), 5)[:5]
        d = np.mean(min_vals)

    dist = d*0.001-0.13
    return dist