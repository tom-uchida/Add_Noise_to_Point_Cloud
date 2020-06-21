#include "addNoise.h"

#include <vector>
#include <time.h>
// #include <pcl/point_cloud.h>
// #include <pcl/kdtree/kdtree_flann.h> 
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
    m_type = _type;
}

void AddNoise::setLamda( double _ratio4lamda, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax ) {
    kvs::Vector3f bb_diagonal_vector = _bbmax - _bbmin;
    double bb_diagonal_length        = bb_diagonal_vector.length();
    m_lamda                          = bb_diagonal_length*bb_diagonal_length * _ratio4lamda;

    std::cout << "Diagonal length of BB: " << bb_diagonal_length << std::endl;
    std::cout << "Lamda(average): " << m_lamda << " (= " << bb_diagonal_length*bb_diagonal_length << " * " << _ratio4lamda << "(argv[4]) )" << std::endl;
}

void AddNoise::addNoise( kvs::PolygonObject* _ply ) {
    clock_t start = clock(); // Start time count

    // Add noise
    // Gaussian noise
    if ( m_type == Gaussian ) {
        addGaussianNoise( _ply );
        
    // Poisson noise
    } else if ( m_type == Poisson ) {
        setLamda(  /* ratio4lamda  */ m_hyperparameter4noise, 
                   /* BBmin        */ _ply->minObjectCoord(), 
                   /* BBmax        */ _ply->maxObjectCoord() );
        applyPoissonNoise( _ply );
        
    // Outlier noise
    } else if ( m_type == Outlier ) {
        addOutlierNoise( _ply );
    } // end if

    clock_t end = clock(); // End time count
    std::cout << "Done! (" <<(double)(end - start) / CLOCKS_PER_SEC << " sec)" << std::endl;
}

void AddNoise::addGaussianNoise( kvs::PolygonObject* _ply ) {
    // std::vector<pcl::PointXYZ>  points;
    kvs::BoxMuller              gaussRand;
    kvs::MersenneTwister        uniRand;

    kvs::ValueArray<kvs::Real32> coords  = _ply->coords(); 
    kvs::ValueArray<kvs::Real32> normals = _ply->normals();
    size_t num      = _ply->numberOfVertices();
    m_number        = num;
    bool hasNormal  = false;
    if ( num == _ply->numberOfNormals() ) hasNormal = true;

    std::cout << "Number of points     : " << m_number << " (points)" << std::endl;
    std::cout << "\n";
    std::cout << "Adding Gaussian noise with " << m_ratio_of_adding_noise*100 << "(%)..."    << std::endl;

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
            // N(μ, σ)
            // Generate Gaussian noise
            float x_noise = gaussRand.rand(0.0, m_sigma);
            float y_noise = gaussRand.rand(0.0, m_sigma);
            float z_noise = gaussRand.rand(0.0, m_sigma);

            kvs::Vector3f origin_point(x, y, z);
            kvs::Vector3f noised_point(x+x_noise, y+y_noise, z+z_noise);

            // Add Gaussian noise
            x += x_noise;
            y += y_noise;
            z += z_noise;

            // Calculate noise intensity
            kvs::Vector3f distance = noised_point - origin_point;
            float noise_intensity  = distance.length();
            m_noise_intensities.push_back(noise_intensity);

            noise_counter++;
            m_is_noise_points.push_back( true );

        } else {
            m_is_noise_points.push_back( false );
        }
    
        // After adding noise
        coords[3*i]     = x;
        coords[3*i+1]   = y;
        coords[3*i+2]   = z;
        // points.push_back( pcl::PointXYZ( x, y, z ) );
        _ply->setCoords( coords );
    }

    // Save max noise intensity
    m_max_noise_intensity = *std::min_element(m_noise_intensities.begin(), m_noise_intensities.end());
    std::cout << " ** Max noise intensity   : " << m_max_noise_intensity << std::endl;

    std::cout << " ** Number of noise points: " << noise_counter<< " (points)" << std::endl;
}

void AddNoise::applyPoissonNoise( kvs::PolygonObject* _ply ) {
    // std::vector<pcl::PointXYZ>      points;
    // std::random_device              rd;
    // std::mt19937                    gen(rd());
    std::random_device              seed_gen;
    std::default_random_engine      engine(seed_gen());
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

    std::cout << "\n==============================================" << std::endl;
    std::cout << "     Apply poisson noise with " << m_ratio_of_adding_noise*100 << " percent."    << std::endl;
    std::cout << "==============================================" << std::endl;

    float scale       = m_lamda;
    float tmp_max     = 0.0f;
    float tmp_min     = 1.0e3;
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
            kvs::Vector3f origin_point(x, y, z);

            // P(λ)
            if ( uniRand() < 0.5 )  x += poissonRand(engine)*scale;
            else                    x -= poissonRand(engine)*scale;
            
            if ( uniRand() < 0.5 )  y += poissonRand(engine)*scale;
            else                    y -= poissonRand(engine)*scale;

            if ( uniRand() < 0.5 )  z += poissonRand(engine)*scale;
            else                    z -= poissonRand(engine)*scale;

            kvs::Vector3f noised_point(x, y, z);

            // Calculate noise intensity
            kvs::Vector3f distance = noised_point - origin_point;
            float noise_intensity  = distance.length();
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

        // After applying noise
        coords[3*i]     = x;
        coords[3*i+1]   = y;
        coords[3*i+2]   = z;
        // points.push_back( pcl::PointXYZ( x, y, z ) );
        _ply->setCoords( coords );     
    }

    // Save max & min noise intensity
    m_max_noise_intensity = tmp_max;
    m_min_noise_intensity = tmp_min;
    std::cout << "\n";
    std::cout << "Max noise intensity" << std::endl;
    std::cout << "> " << m_max_noise_intensity << std::endl;
    std::cout << "\n";
    std::cout << "Min noise intensity" << std::endl;
    std::cout << "> " << m_min_noise_intensity << std::endl;

    std::cout << "\n";
    std::cout << "Number of noised points" << std::endl;
    std::cout << "> " << noise_counter       << std::endl;
}

void AddNoise::addOutlierNoise( kvs::PolygonObject* _ply ) {
    // std::vector<pcl::PointXYZ>      points;
    kvs::MersenneTwister            uniRand;

    kvs::ValueArray<kvs::Real32> coords  = _ply->coords(); 
    kvs::ValueArray<kvs::Real32> normals = _ply->normals();
    size_t num      = _ply->numberOfVertices();
    m_number        = num;
    bool hasNormal  = false;
    if ( num == _ply->numberOfNormals() ) hasNormal = true;

    std::cout << "Number of points     : " << m_number << " (points)" << std::endl;
    std::cout << "\n";
    std::cout << "Adding outlier noise with " << m_ratio_of_adding_noise*100 << "(%)..."    << std::endl;

    // Pre-Process for generating 3D random point
    kvs::Vector3f min_BB = _ply->minObjectCoord();
    kvs::Vector3f max_BB = _ply->maxObjectCoord();
    float x_BB = max_BB.x() - min_BB.x();
    float y_BB = max_BB.y() - min_BB.y();
    float z_BB = max_BB.z() - min_BB.z();

    // std::ofstream fout_outlier_noise_point("outlier_noise_point.txt");
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

        // Add outlier noise
        if ( uniRand() < m_ratio_of_adding_noise ) {
            kvs::Vector3f origin_point(x, y, z);

            // Random 3D point in BB
            x = x_BB*uniRand() + min_BB.x(); // x coord is a random number [min_BB.x, max_BB.x]
            y = y_BB*uniRand() + min_BB.y(); // y coord is a random number [min_BB.y, max_BB.y]
            z = z_BB*uniRand() + min_BB.z(); // z coord is a random number [min_BB.z, max_BB.z]

            kvs::Vector3f noised_point(x, y, z);

            // Write to .txt file
            //fout_outlier_noise_point << x << " " << y << " " << z << std::endl;

            // Calculate noise intensity
            kvs::Vector3f distance = noised_point - origin_point;
            float noise_intensity  = distance.length();
            m_noise_intensities.push_back(noise_intensity);
            
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
        // points.push_back( pcl::PointXYZ( x, y, z ) );
        _ply->setCoords( coords );     
    }

    // Save max noise intensity
    m_max_noise_intensity = *std::min_element(m_noise_intensities.begin(), m_noise_intensities.end());
    std::cout << " ** Max noise intensity   : " << m_max_noise_intensity << std::endl;

    std::cout << " ** Number of noise points: " << noise_counter<< " (points)" << std::endl;
}

void AddNoise::normalizeNoiseIntensities() {
    for (int i = 0; i < m_noise_intensities.size(); i++)
        m_noise_intensities[i] /= m_max_noise_intensity;

    // double max = *max_element( m_noise_intensities.begin(), m_noise_intensities.end() );
    // double min = *min_element( m_noise_intensities.begin(), m_noise_intensities.end() );
    // std::cout << max << "," << min << std::endl;
}