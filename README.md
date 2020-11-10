# Add_Noise_to_Point_Cloud
## Overview
- Add noise to point cloud

- We have the following two types of "noise targets"
   - Coordinates
      - Gaussian noise
      - Poisson noise
      - Outlier noise
   - Color
      - Gaussian

## Visualization Results
### Noise in "coordinate" space
|Gaussian noise|Poisson noise|Outlier noise|
|:-:|:-:|:-:|
|![coords_noise1](coords/sample_images/gaussian_10per_1e-2.bmp)|![coords_noise2](coords/sample_images/poisson_1per_1e-3.bmp)|![coords_noise3](coords/sample_images/outlier_10per_1e-2.bmp)|

### Noise in "color" space
|Original color|sigma=20|sigma=40|
|:-:|:-:|:-:|
|![color_noise1](color/sample_images/funehoko_original_color.bmp)|![color_noise2](color/sample_images/funehoko_50per_sigma20.bmp)|![color_noise3](color/sample_images/funehoko_50per_sigma40.bmp)|