#include "addNoise.h"

#include <vector>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <kvs/BoxMuller> 
#include <kvs/MersenneTwister>
#include <random>   // for poisson distribution


AddNoise::AddNoise( void ): 
    m_type( Gaussian ),
    m_number( 0 ),
    m_sigma( 0.0 ),
    m_ratio_of_adding_noise( 0.0 ),
    m_param_spec_to_noise( 0.0 )
{}

AddNoise::AddNoise( double _ratio_of_adding_noise, double _param_spec_to_noise): 
    m_number( 0 ),
    m_sigma( 0.0 ),
    m_ratio_of_adding_noise( _ratio_of_adding_noise ),
    m_param_spec_to_noise( _param_spec_to_noise )
{}

void AddNoise::setNoiseType( NoiseType _type ) {
    m_type = _type;
}

void AddNoise::setSigma( double _ratio_for_sigma, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax  ) {
    kvs::Vector3f diagonal_vector   = _bbmax - _bbmin;
    double diagonal_length          = diagonal_vector.length();
    m_sigma                         = diagonal_length * _ratio_for_sigma; // Calc sigma

    std::cout << "\n\n----- Calc. sigma -----"      << std::endl;
    std::cout << "Diagonal length"                  << std::endl;
    std::cout << "> " << diagonal_length << "\n"    << std::endl;
    std::cout << "Sigma(standard deviation)"        << std::endl;
    std::cout << "> " << m_sigma << " ( = " << diagonal_length << " * " << _ratio_for_sigma << "(argv[4]) )" << std::endl;
}

void AddNoise::setLamda( double _ratio_for_lamda, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax  ) {
    kvs::Vector3f diagonal_vector   = _bbmax - _bbmin;
    double diagonal_length          = diagonal_vector.length();
    m_lamda                         = diagonal_length * _ratio_for_lamda; // Calc lamda

    std::cout << "\n\n----- Calc. lamda -----"      << std::endl;
    std::cout << "Diagonal length"                  << std::endl;
    std::cout << "> " << diagonal_length << "\n"    << std::endl;
    std::cout << "Lamda(average)"                   << std::endl;
    std::cout << "> " << m_lamda << " ( = " << diagonal_length << " * " << _ratio_for_lamda << "(argv[4]) )" << std::endl;
}

void AddNoise::addNoise( kvs::PolygonObject* _ply ) {
    // Start time count
    std::cout << "\n\n=======================" << std::endl;
    std::cout << "===== Clock_start =====" << std::endl;
    std::cout << "=======================" << std::endl;
    clock_t start = clock();


    // ---------------------
    // ----- Add noise -----
    // ---------------------
    // Gaussian Noise
    if ( m_type == Gaussian ) {
        setSigma(  /* ratio4sigma  */ m_param_spec_to_noise, 
                   /* BBmin        */ _ply->minObjectCoord(), 
                   /* BBmax        */ _ply->maxObjectCoord() );
        addGaussianNoise( _ply );
        
    // Poisson Noise
    } else if ( m_type == Poisson ) {
        setLamda(  /* ratio4sigma  */ m_param_spec_to_noise, 
                   /* BBmin        */ _ply->minObjectCoord(), 
                   /* BBmax        */ _ply->maxObjectCoord() );
        applyPoissonNoise( _ply );
        
    // Spike Noise
    } else if ( m_type == Spike ) {
        addSpikeNoise( _ply );
    }


    // End time count
    clock_t end = clock();
    std::cout << "\n\n===========================" << std::endl;
    std::cout << "===== Time : " <<(double)(end - start) / CLOCKS_PER_SEC << " =====" << std::endl;
    std::cout << "===========================" << std::endl;
}

void AddNoise::addGaussianNoise( kvs::PolygonObject* _ply ) {
    std::vector<pcl::PointXYZ>  points;
    kvs::BoxMuller              gaussRand;
    kvs::MersenneTwister        uniRand;

    kvs::ValueArray<kvs::Real32> coords  = _ply->coords(); 
    kvs::ValueArray<kvs::Real32> normals = _ply->normals();
    size_t num      = _ply->numberOfVertices();
    m_number        = num;
    bool hasNormal  = false;
    if ( num == _ply->numberOfNormals() ) hasNormal = true;

    std::cout << "\n\nNumber of points" << std::endl;
    std::cout << "> " << m_number                   << std::endl;

    std::cout << "\n\n==============================================" << std::endl;
    std::cout << "     Add Gaussian noise with " << m_ratio_of_adding_noise*100 << " percent."    << std::endl;
    std::cout << "==============================================" << std::endl;

    float tmp_max       = 0.0f;
    float tmp_min       = 1.0e3;
    int noise_counter   = 0;
    // coords[]（KVS） → pcl::PointXYZ(x,y,z)（PCL）
    for ( size_t i = 0; i < num; i++ ) {
        float x  = coords[3*i];
        float y  = coords[3*i+1];
        float z  = coords[3*i+2];
        float nx = 0.0;
        float ny = 0.0;
        float nz = 0.0; 
        if ( hasNormal ) {
            nx = normals[3*i];
            ny = normals[3*i+1];
            nz = normals[3*i+2];
        }

        // Add Gaussian noise
        if ( uniRand() < m_ratio_of_adding_noise ) {
            // N(μ, σ^2)
            // Generate Gaussian noise
            float x_noise = gaussRand.rand(0.0, m_sigma*m_sigma);
            float y_noise = gaussRand.rand(0.0, m_sigma*m_sigma);
            float z_noise = gaussRand.rand(0.0, m_sigma*m_sigma);

            kvs::Vector3f origin_point(x, y, z);
            kvs::Vector3f noised_point(x+x_noise, y+y_noise, z+z_noise);

            // Add Gaussian noise
            x += x_noise;
            y += y_noise;
            z += z_noise;

            // Calc noise intensity
            kvs::Vector3f distance  = noised_point - origin_point;
            float noise_intensity   = distance.length();
            m_noise_intensities.push_back(noise_intensity);

            // Update max & min noise intensity
            if( tmp_max < noise_intensity ) tmp_max = noise_intensity;
            if( tmp_min > noise_intensity ) tmp_min = noise_intensity;

            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
            m_noise_intensities.push_back( 0.0f );
        }
    
      
        // After adding noise
        coords[3*i]     = x;
        coords[3*i+1]   = y;
        coords[3*i+2]   = z;
        points.push_back( pcl::PointXYZ( x, y, z ) );
        _ply->setCoords( coords );     
    }

    // Save max & min noise intensity
    m_max_noise_intensity = tmp_max;
    m_min_noise_intensity = tmp_min;
    std::cout << "\nMax noise intensity" << std::endl;
    std::cout << "> " << m_max_noise_intensity << std::endl;
    std::cout << "\nMin noise intensity" << std::endl;
    std::cout << "> " << m_min_noise_intensity << std::endl;

    std::cout << "\nNumber of noised points" << std::endl;
    std::cout << "> " << noise_counter       << std::endl;
}

void AddNoise::applyPoissonNoise( kvs::PolygonObject* _ply ) {
    std::vector<pcl::PointXYZ>      points;
    std::random_device              rd;
    std::mt19937                    gen(rd());
    std::poisson_distribution<>     poissonRand(m_lamda);
    kvs::MersenneTwister            uniRand;

    kvs::ValueArray<kvs::Real32> coords  = _ply->coords(); 
    kvs::ValueArray<kvs::Real32> normals = _ply->normals();
    size_t num      = _ply->numberOfVertices();
    m_number        = num;
    bool hasNormal  = false;
    if ( num == _ply->numberOfNormals() ) hasNormal = true;

    std::cout << "\n\nNumber of points" << std::endl;
    std::cout << "> " << m_number                   << std::endl;

    std::cout << "\n\n==============================================" << std::endl;
    std::cout << "     Apply Poisson noise with " << m_ratio_of_adding_noise*100 << " percent."    << std::endl;
    std::cout << "==============================================" << std::endl;

    float scale = m_lamda;
    int noise_counter = 0;
    // coords[]（KVS） → pcl::PointXYZ(x,y,z)（PCL）
    for ( size_t i = 0; i < num; i++ ) {
        float x  = coords[3*i];
        float y  = coords[3*i+1];
        float z  = coords[3*i+2];
        float nx = 0.0;
        float ny = 0.0;
        float nz = 0.0; 
        if ( hasNormal ) {
            nx = normals[3*i];
            ny = normals[3*i+1];
            nz = normals[3*i+2];
        }

        // Apply Poisson noise
        if ( uniRand() < m_ratio_of_adding_noise ) {
            // P(λ)
            if ( uniRand() < 0.5 )  x += poissonRand(gen)*scale;
            else                    x -= poissonRand(gen)*scale;
            
            if ( uniRand() < 0.5 )  y += poissonRand(gen)*scale;
            else                    y -= poissonRand(gen)*scale;

            if ( uniRand() < 0.5 )  z += poissonRand(gen)*scale;
            else                    z -= poissonRand(gen)*scale;

            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
        }
      
        // After applying noise
        coords[3*i]     = x;
        coords[3*i+1]   = y;
        coords[3*i+2]   = z;
        points.push_back( pcl::PointXYZ( x, y, z ) );
        _ply->setCoords( coords );     
    }

    std::cout << "\nNumber of noised points" << std::endl;
    std::cout << "> " << noise_counter       << std::endl;
}

void AddNoise::addSpikeNoise( kvs::PolygonObject* _ply ) {
}