#ifndef _AddNoise_H__
#define _AddNoise_H__

#include <kvs/PolygonObject>
#include <vector>

class AddNoise {

public:
    enum NoiseType {
        Gaussian = 0,
        Outlier  = 1
    };

public:
    AddNoise( void );
    AddNoise( const double _noise_prob, const double _sigma );

    void setNoiseType( const NoiseType _type );
    void addNoise2Coords( kvs::PolygonObject* _ply );
    void addNoise2Color( kvs::PolygonObject* _ply );
    std::vector<bool> getIsNoisePoints( void ) const { return m_is_noise_points; }

private:
    void addGaussianNoise( kvs::PolygonObject* _ply );
    void addOutlierNoise( kvs::PolygonObject* _ply );

private:
    NoiseType           m_noise_type;
    size_t	            m_number_of_points;
    double	            m_noise_probability;
    double	            m_sigma;
    std::vector<bool>   m_is_noise_points;

};

#endif