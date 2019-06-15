# Add_Noise_to_Point_Cloud
## Overview
- Add noise to point cloud

- We have the following "hree types of noises"
   - Gaussian
   - Poisson
   - Spike

- We have the following two "noise targets"
   - Coordinate
   - Color

## Result
### Coordinate Noise
|Gaussian|Poisson|Spike|
|:-:|:-:|:-:|
|![coords_noise1](resources/gaussian_LR1.bmp)|![coords_noise2](resources/poisson_LR1.bmp)|![coords_noise3](resources/spike_LR1.bmp)|

### Color Noise
|Original color|sigma=5|sigma=10|
|:-:|:-:|:-:|
|![color_noise1](resources/funehoko_original_color.bmp)|![color_noise2](resources/100per_sigma5.bmp)|![color_noise3](resources/100per_sigma10.bmp)|