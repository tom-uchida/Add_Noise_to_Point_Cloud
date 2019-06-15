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
|![coords_noise1](coords/sample_images/gaussian_LR1.bmp)|![coords_noise2](coords/sample_images/poisson_LR1.bmp)|![coords_noise3](coords/sample_images/spike_LR1.bmp)|

### Color Noise
|Original color|sigma=5|sigma=10|
|:-:|:-:|:-:|
|![color_noise1](color/sample_images/funehoko_original_color.bmp)|![color_noise2](color/sample_images/100per_sigma5.bmp)|![color_noise3](color/sample_images/100per_sigma10.bmp)|