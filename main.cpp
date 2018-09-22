// Program to add noise to point cloud

// USAGE:
// ./addNoise [data_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise]

// [For example]
// $ ./addNoise input.ply output.xyz 0.1 0.002
// > sigma = b_leng * 0.002 (Gaussian)
// > lamda = b_leng * 0.002 (Poisson)
// > Add noise with 0.1*100=10 percent.

#include <iostream>
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "addNoise.h"
#include "writeNoiseIntensity.h"

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

const char OUT_FILE[] = "out_noised.xyz";

int main( int argc, char** argv ) {
    char outXYZfile[512];
    strcpy( outXYZfile, OUT_FILE ); 

    if ( argc != 5 ) {
        std::cout << "\n----- USAGE -----\n" << argv[0] << " [input_file] [output_file] [ratio_of_adding_noise] [param_spec_to_noise]\n"
                  << "\n----- For example -----\n" 
                  << "$ " << argv[0] << " input.ply output.xyz 0.1 0.002\n"
                  << "> sigma = b_leng * 0.002\n"
                  << "> Add noise with 0.1*100=10 percent.\n"
                  << std::endl;

        exit(1);

    } else if ( argc == 5 ) {
        strcpy( outXYZfile, argv[2] );
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

    // ----- Select noise type ( Gaussian or Poisson or Spike )-----
    //an->setNoiseType( AddNoise::Gaussian );
    //an->setNoiseType( AddNoise::Poisson );
    an->setNoiseType( AddNoise::Spike );


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

    // ----- Output File for "xyzrgbf" -----
    WritingDataType type = Ascii;   // Writing data as ascii
    writeNoiseIntensity( ply,                  /* kvs::PolygonObject *_ply        */  
                         //noise_intensities,    /* std::vector<float> &_ni         */  
                         outXYZfile,           /* char*              _filename    */  
                         type );               /* WritingDataType    _type        */  
  
    // ----- Convert "PolygonObject（KVS）" to "PointObject（KVS）" -----
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    // ----- KVS screen -----
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
}
