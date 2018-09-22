# Add_Noise_v001
Add the following three types of noises to point cloud
- Gaussian
- Poisson
- Spike

## How to execute this program
`./addNoise [input_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise]`

`./addNoise [.ply] [.spbr] [0.1] [0.001]`

## Three types of noises
1. Gaussian (AddNoise::Gaussian)

2. Poisson (AddNoise::Poisson)

3. Spike (AddNoise::Spike)
