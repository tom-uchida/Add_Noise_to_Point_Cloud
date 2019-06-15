# Add_Noise_to_Point_Cloud
## Overview
Add the Gaussian noise to color of each point

### Usage
`./addNoise2color [data_file] [output_file] [noise_ratio] [sigma(=standard deviation)]`

### For example
`./addNoise input.ply output.spbr 0.1 5.0`

## Result
|Original color|sigma=5|sigma=10|
|:-:|:-:|:-:|
|![color_noise1](sample_images/funehoko_original_color.bmp)|![color_noise2](sample_images/100per_sigma5.bmp)|![color_noise3](sample_images/100per_sigma10.bmp)|