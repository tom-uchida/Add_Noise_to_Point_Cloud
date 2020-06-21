// Uchida-Tomomasas-MacBook-Pro:coords uchidatomomasa$ ./addNoise2coords 

// ================================================
//      Add Noise to "Coords" of Point Cloud
//                Tomomasa Uchida
//                  2020/06/21
// ================================================

//   USAGE:
//   ./addNoise2coords [input_file] [output_file] [noise_probability] [hyperparameter4noise] [noise_option]

//   EXAMPLE:
//   ./addNoise2coords input.ply output.spbr 0.1 0.01 -g

//    [noise_probability]
//     Add noise with 10(=0.1*100) percent.

//    [hyperparameter4noise]
//     Gaussian : sigma = 0.01
//     Poisson  : lamda = (diagonal length of BB) * 0.01
//     Outlier  : none

//    [noise_option]
//     -g : Add Gaussian noise
//     -p : Add Poisson noise
//     -o : Add Outlier noise

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "addNoise.h"
#include "writeSPBR.h"
#include "noise_option.h"
#include "tinycolormap.h"

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

#define NOISE_ORIGINAL_COLOR
//#define NOISE_INTENSITY
//#define NOISE_ARTIFICIAL_PLANE

const char OUT_FILE[] = "SPBR_DATA/out_coords_noise.spbr";

void message() {
    std::cout << "\n";
    std::cout << "================================================" << std::endl;
    std::cout << "     Add Noise to \"Coords\" of Point Cloud"      << std::endl;
    std::cout << "               Tomomasa Uchida"                   << std::endl;
    std::cout << "                 2020/06/21"                      << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "\n";
}

int main( int argc, char** argv ) {
    char outSPBRfile[512];
    strcpy( outSPBRfile, OUT_FILE ); 

    if ( argc != 6 ) {
        message();
        std::cout << "  USAGE:\n  ";
        std::cout << argv[0] << " [input_file] [output_file] [noise_probability] [hyperparameter4noise] [noise_option]";
        std::cout << "\n\n  EXAMPLE:\n  ";
        std::cout << argv[0] << " input.ply output.spbr 0.1 0.01 -g"
                  << "\n\n"
                  << "   [noise_probability]\n"
                  << "    Add noise with 10(=0.1*100) percent."
                  << "\n\n"
                  << "   [hyperparameter4noise]\n"
                  << "    Gaussian : sigma = 0.01\n"
                  << "    Poisson  : lamda = (diagonal length of BB) * 0.01\n"
                  << "    Outlier  : none"
                  << "\n\n"
                  << "   [noise_option]\n"
                  << "    -g : Add Gaussian noise\n"
                  << "    -p : Add Poisson noise\n"
                  << "    -o : Add Outlier noise\n"
                  << std::endl;
        exit(1);

    } else if ( argc >= 3 ) {
        strcpy( outSPBRfile, argv[2] );
    } // end if

    // Display message
    message();
    
    // Import "point cloud data（.ply, argv[1]）" that user selected
    // Inheritance of KVS::PolygonObject -----
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "\nPLY Min, Max Coords:" << std::endl;
    std::cout << "Min : " << ply->minObjectCoord() << std::endl;
    std::cout << "Max : " << ply->maxObjectCoord() << std::endl;

    // Set up for adding noise
    AddNoise *an = new AddNoise( /* noise probability    */ atof(argv[3]),
                                 /* hyperparameter4noise */ atof(argv[4]) );

    // Apply noise type ( Gaussian or Poisson or Outlier )
    for ( int i = 1; i < argc; i++ ) {
        // Gaussian
        if ( !strncmp( GAUSSIAN_OPTION, argv[i], strlen( GAUSSIAN_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Gaussian );
            // std::cout << "\n";
            // std::cout << "Noise Type : Gaussian noise"   << std::endl;
            i++;

        // Poisson
        } else if ( !strncmp( POISSON_OPTION, argv[i], strlen( POISSON_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Poisson );
            // std::cout << "\n";
            // std::cout << "Noise Type : Poisson noise"  << std::endl;
            i++;

        // Outlier
        } else if ( !strncmp( OUTLIER_OPTION, argv[i], strlen( OUTLIER_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Outlier );
            // std::cout << "\n";
            // std::cout << "Noise Type : Outlier noise" << std::endl;
            i++;
        }
    } // end for

    // Add Noise
    an->addNoise( /* kvs::PolygonObject* */ ply );

    // Get noise indexes
    std::vector<bool> is_noise_points = an->getIsNoisePoints();

    // Get noise intensity
#ifdef NOISE_INTENSITY
    // const float max_noise_intensity = (float)an->getMaxNoiseIntensity();
    // const float min_noise_intensity = (float)an->getMinNoiseIntensity();
    an->normalizeNoiseIntensities();
    const std::vector<float> normalized_noise_intensities = an->getNoiseIntensities();
#endif

    //kvs::ColorMap cmap( 256, min_noise_intensity, max_noise_intensity );
    // kvs::ColorMap cmap( 256, 0.0, max_noise_intensity );
    // cmap.create();

    // Apply color
    std::vector<unsigned char>  colors;
    kvs::ValueArray<kvs::UInt8> original_colors = ply->colors();
    kvs::RGBColor white(255, 255, 255), red(255, 0, 0), black(0, 0, 0);
    for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
#ifdef NOISE_INTENSITY
        // not noised points (white)
        if ( !is_noise_points[i] ) {
            // colors.push_back( white.r() );
            // colors.push_back( white.g() );
            // colors.push_back( white.b() );
            colors.push_back( original_colors[3*i]);
            colors.push_back( original_colors[3*i+1] );
            colors.push_back( original_colors[3*i+2] );


        // noised points (colormap)
        } else {
            // ----- Get the mapped color -----
            tinycolormap::Color color = tinycolormap::GetColor(normalized_noise_intensities[i], tinycolormap::ColormapType::Viridis);
            //tinycolormap::Color color = tinycolormap::GetColor(normalized_noise_intensities[i], tinycolormap::ColormapType::Jet);
            //tinycolormap::Color color = tinycolormap::GetColor(normalized_noise_intensities[i], tinycolormap::ColormapType::Heat);
            colors.push_back( color.r()*255 );
            colors.push_back( color.g()*255 );
            colors.push_back( color.b()*255 );
            // colors.push_back( original_colors[3*i]);
            // colors.push_back( original_colors[3*i+1] );
            // colors.push_back( original_colors[3*i+2] );
        }
#endif

#ifdef NOISE_ARTIFICIAL_PLANE
        // Noise points(red)
        if ( is_noise_points[i] ) {
            colors.push_back( red.r() );
            colors.push_back( red.g() );
            colors.push_back( red.b() );

        // Not noised points(white)
        } else {
            colors.push_back( white.r() );
            colors.push_back( white.g() );
            colors.push_back( white.b() );
        }
#endif

#ifdef NOISE_ORIGINAL_COLOR
        colors.push_back( original_colors[3*i]);
        colors.push_back( original_colors[3*i+1] );
        colors.push_back( original_colors[3*i+2] );
#endif
    }
    ply->setColors( kvs::ValueArray<kvs::UInt8>( colors ) );

    // Write to spbr file
    WritingDataType type = Ascii; // Writing data as ascii
    writeSPBR( ply,                    /* kvs::PolygonObject *_ply        */  
               //noise_intensities,    /* std::vector<float> &_ni         */  
               outSPBRfile,            /* char*              _filename    */  
               type );                 /* WritingDataType    _type        */  

    // Convert "PolygonObject（KVS）" to "PointObject（KVS）"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    return 0;
} // end main()