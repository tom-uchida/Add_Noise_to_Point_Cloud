# Add Noise to "Coords" of Point Cloud
## Overview
- Add the following three types of noises to the coordinates of each point.
   - Gaussian noise
   - Poisson noise
   - Outlier noise

## Usage
```
$ ./addNoise2coords 

================================================
     Add Noise to "Coords" of Point Cloud
               Tomomasa Uchida
                 2020/06/21
================================================

  USAGE:
  ./addNoise2coords [input_file] [output_file] [noise_probability] [hyperparameter4noise] [noise_option]

  EXAMPLE:
  ./addNoise2coords input.ply output.spbr 0.1 0.01 -g

   [noise_probability]
    Add noise with 10(=0.1*100) percent.

   [hyperparameter4noise]
    Gaussian : sigma = 0.01
    Poisson  : lamda = (diagonal length of BB) * 0.01
    Outlier  : none

   [noise_option]
    -g : Add Gaussian noise
    -p : Add Poisson noise
    -o : Add Outlier noise
```

## Visualization Result
### Figure
|Gaussian noise|Poisson noise|Outlier noise|
|:-:|:-:|:-:|
|![gaussian](sample_images/gaussian_10per_1e-2.bmp)|![poisson](sample_images/poisson_1per_1e-3.bmp)|![spike](sample_images/outlier_10per_1e-2.bmp)|