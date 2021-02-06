# Add_Noise_to_Point_Cloud
## Overview
- Add noise to point cloud

- We have the following two types of "noise targets"
   - Coordinates
      - Gaussian noise
      - Outlier noise
   - Color
      - Gaussian

## Visualization Results
### Noise in "coordinate" space
|Gaussian noise|Outlier noise|
|:-:|:-:|
|![gaussian](coords/figures/gaussian_10per_1e-2.bmp)|![outlier](coords/figures/outlier_10per_1e-2.bmp)|

### Noise in "color" space
|Original color|sigma=20|sigma=40|
|:-:|:-:|:-:|
|![color_noise1](color/figures/funehoko_original_color.bmp)|![color_noise2](color/figures/funehoko_50per_sigma20.bmp)|![color_noise3](color/figures/funehoko_50per_sigma40.bmp)|