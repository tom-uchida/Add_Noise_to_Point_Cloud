#include "add_noise.h"

#include <vector>
#include <time.h>
#include <kvs/BoxMuller> 
#include <kvs/MersenneTwister>
#include <fstream>

AddNoise::AddNoise( void ): 
    m_noise_type( Gaussian ),
    m_number_of_points( 0 ),
    m_noise_probability( 0.0f ),
    m_sigma( 0.0f ),
    m_is_noise_points()
{}

AddNoise::AddNoise( const double _noise_prob, const double _sigma ): 
    m_number_of_points( 0 ),
    m_noise_probability( _noise_prob ),
    m_sigma( _sigma ),
    m_is_noise_points()
{}

void AddNoise::setNoiseType( const NoiseType _type ) {
    m_noise_type = _type;
}

void AddNoise::addNoise( kvs::PolygonObject* _ply ) {
    const clock_t start = clock();

    // Gaussian noise
    if ( m_noise_type == Gaussian ) {
        this->addGaussianNoise( _ply );
        
    // Outlier noise
    } else if ( m_noise_type == Outlier ) {
        this->addOutlierNoise( _ply );
    }

    const clock_t end = clock();
    std::cout << "\n";
    std::cout << "Done adding noise!";
    std::cout << " ( " << (double)( end - start ) / CLOCKS_PER_SEC << " [sec] )\n";
} // end of addNoise( kvs::PolygonObject* _ply )

void AddNoise::addGaussianNoise( kvs::PolygonObject* _ply ) {
    kvs::BoxMuller        gaussRand;
    kvs::MersenneTwister  uniRand;

    m_number_of_points = _ply->numberOfVertices();
    std::cout << "\n";
    std::cout << "Adding \"Gaussian noise\" with " << m_noise_probability*100 << "(%)...\n";
    std::cout << " sigma: " << m_sigma << "\n";
    std::cout << " Number of original points: " << m_number_of_points << " (points)" << "\n";

    // Add Gaussian noise
    kvs::ValueArray<kvs::Real32> coords  = _ply->coords();
    size_t noise_counter = 0;
    for ( size_t i = 0; i < m_number_of_points; i++ ) {
        
        if ( uniRand() < m_noise_probability ) {
            coords[3*i  ] += gaussRand.rand( 0.0f, m_sigma );
            coords[3*i+1] += gaussRand.rand( 0.0f, m_sigma );
            coords[3*i+2] += gaussRand.rand( 0.0f, m_sigma );

            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
        } // end if

    } // end for

    _ply->setCoords( coords );
    std::cout << " Number of noise points: " << noise_counter << " (points)\n";
} // End of addGaussianNoise()

void AddNoise::addOutlierNoise( kvs::PolygonObject* _ply ) {
    kvs::MersenneTwister uniRand;
    
    m_number_of_points = _ply->numberOfVertices();
    std::cout << "\n";
    std::cout << "Adding \"Outlier noise\" with " << m_noise_probability*100 << "(%)...\n";
    std::cout << " Number of original points: " << m_number_of_points << " (points)\n";

    // Preprocess for generating 3D random points
    kvs::Vector3f BB_min = _ply->minObjectCoord();
    kvs::Vector3f BB_max = _ply->maxObjectCoord();
    const float BB_x = BB_max.x() - BB_min.x();
    const float BB_y = BB_max.y() - BB_min.y();
    const float BB_z = BB_max.z() - BB_min.z();

    // Add Outlier noise
    kvs::ValueArray<kvs::Real32> coords = _ply->coords(); 
    size_t noise_counter = 0;
    for ( size_t i = 0; i < m_number_of_points; i++ ) {

        if ( uniRand() < m_noise_probability ) {
            coords[3*i  ] = BB_x*uniRand() + BB_min.x();
            coords[3*i+1] = BB_y*uniRand() + BB_min.y();
            coords[3*i+2] = BB_z*uniRand() + BB_min.z();
            
            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
        } // end if

    } // end for

    _ply->setCoords( coords );
    std::cout << " Number of noise points: " << noise_counter<< " (points)\n";
} // End of addOutlierNoise()