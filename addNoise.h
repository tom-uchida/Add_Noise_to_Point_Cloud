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

public:
    AddNoise( void );
    AddNoise( const FeatureType _type, double _ratio_of_adding_noise );

    void setNoiseType( NoiseType _type );
    void setSigma( double _ratio4sigma, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax  );
    void addNoise( kvs::PolygonObject* _ply );

    std::vector<float> getNoiseIntensities( void ) { return m_noise_intensities; }
    double getMaxNoiseIntensity( void ) { return m_max_noise_intensity; }
    double getMinNoiseIntensity( void ) { return m_min_noise_intensity; }

private:
    NoiseType   m_type;
    size_t	    m_number;
    double	    m_sigma;
    double	    m_ratio_of_adding_noise;

    std::vector<float> m_noise_intensities;
    double	    m_max_noise_intensity;
    double      m_min_noise_intensity;

private:
    void addGaussianNoise( std::vector<pcl::PointXYZ> &point );
    void applyPoissonNoise( std::vector<pcl::PointXYZ> &point );
    void addSpikeNoise( std::vector<pcl::PointXYZ> &point );
};

#endif