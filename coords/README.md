# Add_Noise_to_Point_Cloud
## Overview
Add the following three types of noises to coordinates of points.
   - Gaussian
   - Poisson
   - Outlier

### Usage
`./addNoise [input_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise] [noise_option]`

### For example
`./addNoise [.ply] [.spbr] [0.1] [0.01] [-g]`

|argv|content|
|:--:|:--|
|argv[1]|Input file (.ply)|
|argv[2]|Output file (.spbr)|
|argv[3]|Probability of adding noise|
|argv[4]|Parameter (sigma, lamda, none)|
|argv[5]|Noise type (-g, -p, -o)|


## Result
### Figure
|Gaussian|Poisson|Spike|
|:-:|:-:|:-:|
|![gaussian](sample_images/gaussian_LR1.bmp)|![poisson](sample_images/poisson_LR1.bmp)|![spike](sample_images/spike_LR1.bmp)|

### Parameter
|Noise type|argv[3]|argv[4]|argv[5]|
|:--|:-:|:-|:-:|
|Gaussian|0.1 (10%)|0.01 (variance=0.01)|-g|
|Poisson|0.1 (10%)|0.01 (lamda=diagonal length of BB*0.01)|-p|
|Outlier|0.1 (10%)|none|-o|