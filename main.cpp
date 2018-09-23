// ===================================================
//      Add three types of noises to point cloud
// ===================================================

// USAGE:
// ./addNoise [data_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise] [noise_option]

// [For example]
// $ ./addNoise input.ply output.spbr 0.1 0.001 -g
// > Gaussian : sigma = b_leng * 0.001
// > Poisson  : lamda = b_leng * 0.001
// > Spike    : none
// > Add noise with (0.1*100=)10 percent.

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "addNoise.h"
#include "writeSPBR.h"
#include "noise_option.h"

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

#define EXEC_VIA_SPBR
//#define EXEC_VIA_KVS

const char OUT_FILE[] = "out_noised.spbr";

int main( int argc, char** argv ) {
    char outSPBRfile[512];
    strcpy( outSPBRfile, OUT_FILE ); 

    if ( argc != 6 ) {
        std::cout << "\n----- USAGE -----\n" << argv[0] << " [input_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise]\n"
                  << "\n----- For example -----\n" 
                  << "$ " << argv[0] << " input.ply output.xyz 0.1 0.001\n"
                  << "> Gaussian : sigma = b_leng * 0.001\n"
                  << "> Poisson  : lamda = b_leng * 0.001\n"
                  << "> Spike    : none\n"
                  << "> Add noise with (0.1*100=)10 percent.\n"
                  << std::endl;
        exit(1);

    } else if ( argc >= 3 ) {
        strcpy( outSPBRfile, argv[2] );
    }
    
    // ----- Import "point cloud data（argv[1]）" that user decided
    // ----- Inheritance of KVS::PolygonObject -----
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "PLY Min, Max Coords:" << std::endl;
    std::cout << "Min : " << ply->minObjectCoord() << std::endl;
    std::cout << "Max : " << ply->maxObjectCoord() << std::endl;

    // ----- Set up for adding noise -----
    AddNoise *an = new AddNoise( atof(argv[3]), atof(argv[4]) );

    // ----- Apply noise type ( Gaussian or Poisson or Spike ) -----
    for ( int i = 1; i < argc; i++ ) {
        // Gaussian
        if ( !strncmp( GAUSSIAN_OPTION, argv[i], strlen( GAUSSIAN_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Gaussian );
            std::cout << "\n\nNoise Type"   << std::endl;
            std::cout << "> Gaussian Noise" << std::endl;
            i++;

        // Poisson
        } else if ( !strncmp( POISSON_OPTION, argv[i], strlen( POISSON_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Poisson );
            std::cout << "\n\nNoise Type"  << std::endl;
            std::cout << "> Poisson Noise" << std::endl;
            i++;

        // Spike
        } else if ( !strncmp( SPIKE_OPTION, argv[i], strlen( SPIKE_OPTION ) ) ) {
            an->setNoiseType( AddNoise::Spike );
            std::cout << "\n\nNoise Type" << std::endl;
            std::cout << "> Spike Noise"  << std::endl;
            i++;
        }
    }


    // ---------------------
    // ----- Add Noise -----
    // ---------------------
    an->addNoise( /* kvs::PolygonObject* */ ply );


    // ----- Get noise indices -----
    std::vector<bool> is_noise_points = an->getIsNoisePoints();

    // ----- Get noise intensity -----
    // std::vector<float> noise_intensities = an->getNoiseIntensities();

    // ----- ColorMap -----
    // float max_noise_intensity = (float)an->getMaxNoiseIntensity();
    // float min_noise_intensity = (float)an->getMinNoiseIntensity();
    //kvs::ColorMap cmap( 256, min_noise_intensity, max_noise_intensity );
    // kvs::ColorMap cmap( 256, 0.0, max_noise_intensity );
    // cmap.create();

    std::vector<unsigned char> colors;
    kvs::RGBColor white(255, 255, 255);
    kvs::RGBColor red(255, 0, 0);
    for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
        // noise point( = red )
        if ( is_noise_points[i] ) {
            colors.push_back( red.r() );
            colors.push_back( red.g() );
            colors.push_back( red.b() );

        } else {
            colors.push_back( white.r() );
            colors.push_back( white.g() );
            colors.push_back( white.b() );
        }
    }
    // for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
    //     // white
    //     if ( noise_intensities[i] == 0.0 ) {
    //         colors.push_back( white.r() );
    //         colors.push_back( white.g() );
    //         colors.push_back( white.b() );

    //     // color map
    //     } else {
    //         kvs::RGBColor color( cmap.at( noise_intensities[i] ) );
    //         colors.push_back( color.r() );
    //         colors.push_back( color.g() );
    //         colors.push_back( color.b() );
    //     }
    // }
    ply->setColors( kvs::ValueArray<kvs::UInt8>( colors ) );

    // ----- Write .spbr file -----
    WritingDataType type = Ascii;   // Writing data as ascii
    writeSPBR( ply,                    /* kvs::PolygonObject *_ply        */  
               //noise_intensities,    /* std::vector<float> &_ni         */  
               outSPBRfile,            /* char*              _filename    */  
               type );                 /* WritingDataType    _type        */  
  
    // ----- Convert "PolygonObject（KVS）" to "PointObject（KVS）" -----
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 



    // ----- Exec. SPBR -----
#ifdef EXEC_VIA_SPBR
    std::string out_noised_spbr( outSPBRfile );
    std::string EXEC("./spbr ");
    EXEC += out_noised_spbr;
    system( EXEC.c_str() );

    return 0;
#endif

    // ----- Exec. KVS -----
#ifdef EXEC_VIA_KVS
    kvs::glut::Application app( argc, argv );
    kvs::glut::Screen screen( &app );

    kvs::PointRenderer* renderer = new kvs::PointRenderer(); 
    renderer->enableTwoSideLighting(); 
    screen.setTitle( "Point Object" );
    
    kvs::Vector3f cam_pos(0, 10, 0);
    kvs::Vector3f cam_up(0, 0, 1);
    
    screen.setBackgroundColor( kvs::RGBColor(0, 0, 0) );
    screen.scene()->camera()->setPosition(cam_pos);
    screen.scene()->camera()->setUpVector(cam_up);
    screen.registerObject( object, renderer);
    screen.show();

    return app.run();
#endif
}
