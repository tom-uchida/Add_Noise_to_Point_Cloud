// 2019/6/15 : Supported adding Gaussian noise to color of each point

#ifndef _AddNoise_H__
#define _AddNoise_H__

#include <kvs/PolygonObject>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector>

class AddNoise {

public:
    enum NoiseType {
        Gaussian = 0,
        Poisson  = 1,
        Spike    = 2
    };

    enum NoiseTarget {
        Coords = 0,
        Color  = 1,
    };

public:
    AddNoise( void );
    AddNoise( double _ratio_of_adding_noise, double _param_spec_to_noise);

    void setNoiseType( NoiseType _type );
    void addNoise2Coords( kvs::PolygonObject* _ply );
    void addNoise2Color( kvs::PolygonObject* _ply );
    std::vector<bool> getNoisePoints( void ) { return m_is_noise_points; }

    std::vector<float> getNoiseIntensities( void ) { return m_noise_intensities; }
    double getMaxNoiseIntensity( void ) { return m_max_noise_intensity; }
    double getMinNoiseIntensity( void ) { return m_min_noise_intensity; }
    void normalizeNoiseIntensities( void );

private:
    NoiseType           m_noise_type;
    NoiseTarget         m_noise_target;
    size_t	            m_number;
    double	            m_sigma2;
    double              m_lamda;
    double	            m_ratio_of_adding_noise;
    double              m_param_spec_to_noise;
    std::vector<bool>   m_is_noise_points;

    std::vector<float>  m_noise_intensities;
    double	            m_max_noise_intensity;
    double              m_min_noise_intensity;

private:
    // Gaussian
    //  coords
    void setSigma( double _ratio_for_sigma, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax );
    void addGaussianNoise( kvs::PolygonObject* _ply );
    //  color
    void addGaussianNoise2Color( kvs::PolygonObject* _ply );

    // Poisson
    void setLamda( double _ratio_for_lamda, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax );
    void applyPoissonNoise( kvs::PolygonObject* _ply );

    // Spike
    void addSpikeNoise( kvs::PolygonObject* _ply );    
};

#endif