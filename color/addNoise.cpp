#include "addNoise.h"

#include <vector>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <kvs/BoxMuller> 
#include <kvs/MersenneTwister>
#include <random> // for poisson distribution
#include <fstream>

AddNoise::AddNoise( void ): 
    m_noise_type( Gaussian ),
    m_number( 0 ),
    m_sigma( 0.0 ),
    m_noise_probability( 0.0 ),
    m_hyperparameter4noise( 0.0 )
{}

AddNoise::AddNoise( double _noise_probability, double _hyperparameter4noise ): 
    m_number( 0 ),
    m_sigma( 0.0 ),
    m_noise_probability( _noise_probability ),
    m_hyperparameter4noise( _hyperparameter4noise )
{}

void AddNoise::setNoiseType( NoiseType _type ) {
    m_noise_type = _type;
}

void AddNoise::addNoise2Color( kvs::PolygonObject* _ply ) {
    // Set sigma(=standard deviation)
    m_sigma = m_hyperparameter4noise;

    std::cout << std::endl;
    std::cout << "Hyperparameters about Gaussian noise" << std::endl;
    std::cout << " - Noise probability        : " << m_noise_probability*100 << "(%)"<< std::endl;
    std::cout << " - sigma(standard deviation): " << m_sigma << std::endl;
    std::cout << std::endl;
    std::cout << "Adding Gaussian noise..." << std::endl;
    
    // Add Gaussian noise to color of each point
    addGaussianNoise2Color( _ply );
}

void AddNoise::addGaussianNoise2Color( kvs::PolygonObject* _ply ) {
    kvs::BoxMuller              gaussRand;
    kvs::MersenneTwister        uniRand;

    // std::vector<unsigned char>  colors;
    kvs::ValueArray<kvs::UInt8> colors = _ply->colors();

    m_number = _ply->numberOfVertices();
    std::cout << " - Number of points: " << m_number << "(points)" << std::endl;

    int noise_counter = 0;
    for ( size_t i = 0; i < m_number; i++ ) {
        kvs::UInt8 r  = colors[3*i];
        kvs::UInt8 g  = colors[3*i+1];
        kvs::UInt8 b  = colors[3*i+2];

        // Add Gaussian noise
        if ( uniRand() < m_noise_probability ) {
            // Check
            // if ( i < 10 ) {
            //     std::cout << "R: " << +r << std::endl;
            //     std::cout << "G: " << +g << std::endl;
            //     std::cout << "B: " << +b << std::endl;
            // }

            // N(μ, σ)
            // Generate Gaussian noise
            int r_noise = gaussRand.rand(r, m_sigma);
            int g_noise = gaussRand.rand(g, m_sigma);
            int b_noise = gaussRand.rand(b, m_sigma);
            if (r_noise < 0)    r_noise = 0;
            if (r_noise > 255)  r_noise = 255;
            if (g_noise < 0)    g_noise = 0;
            if (g_noise > 255)  g_noise = 255;
            if (b_noise < 0)    b_noise = 0;
            if (b_noise > 255)  b_noise = 255;

            // Check
            // if ( i < 10 ) {
            //     std::cout << "R_noised: " << +r_noise << std::endl;
            //     std::cout << "G_noised: " << +g_noise << std::endl;
            //     std::cout << "B_noised: " << +b_noise << std::endl;
            //     std::cout << std::endl;
            // }

            // Add Gaussian noise
            r = kvs::UInt8(r_noise);
            g = kvs::UInt8(g_noise);
            b = kvs::UInt8(b_noise);

            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
        } // end if
    
        // Apply change
        colors[3*i]     = r;
        colors[3*i+1]   = g;
        colors[3*i+2]   = b;
    } // end for

    // Set color after adding Gaussian noise
    _ply->setColors( kvs::ValueArray<kvs::UInt8>( colors ) );

    std::cout << "\n";
    std::cout << "Done adding Gaussian noise to color!" << std::endl;
    std::cout << " - Number of noised points : " << noise_counter << "(points)" << std::endl;
} // End of addGaussianNoise2Color()

void AddNoise::normalizeNoiseIntensities() {
    for (int i = 0; i < m_noise_intensities.size(); i++)
        m_noise_intensities[i] = m_noise_intensities[i] / m_max_noise_intensity;
    // end for

    // double max = *max_element( m_noise_intensities.begin(), m_noise_intensities.end() );
    // double min = *min_element( m_noise_intensities.begin(), m_noise_intensities.end() );
    // std::cout << max << "," << min << std::endl;
}