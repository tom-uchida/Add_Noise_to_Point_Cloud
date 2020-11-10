#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "addNoise.h"
#include "writeSPBR.h"
#include "noise_type.h"
#include "tinycolormap.h"

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

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
        std::cout << argv[0] << " [input_file] [output_file] [noise_probability] [hyperparameter4noise] [noise_type]";
        std::cout << "\n\n  EXAMPLE:\n  ";
        std::cout << argv[0] << " input.ply output.spbr 0.1 0.1 -g"
                  << "\n\n"
                  << "   [noise_probability]\n"
                  << "    Add noise with 10(=0.1*100) percent."
                  << "\n\n"
                  << "   [hyperparameter4noise]\n"
                  << "    Gaussian: sigma = 0.1\n"
                  << "    Outlier : none"
                  << "\n\n"
                  << "   [noise_type]\n"
                  << "    -g: Gaussian noise\n"
                  << "    -o: Outlier noise\n"
                  << std::endl;
        exit(1);

    } else if ( argc >= 3 ) {
        strcpy( outSPBRfile, argv[2] );
    } // end if

    // Display message
    message();
    
    // Import input point cloud
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "\nPLY Min, Max Coords:" << std::endl;
    std::cout << "Min : " << ply->minObjectCoord() << std::endl;
    std::cout << "Max : " << ply->maxObjectCoord() << std::endl;

    // Set up for adding noise
    AddNoise *an = new AddNoise(
        atof( argv[3] ),  /* double _noise_probability    */ 
        atof( argv[4] )   /* double _hyperparameter4noise */
    );

    // Determine noise type ( Gaussian or Poisson or Outlier )
    for ( int i = 1; i < argc; i++ ) {
        // Gaussian
        if ( !strncmp( GAUSSIAN_TYPE , argv[i], strlen( GAUSSIAN_TYPE ) ) ) {
            an->setNoiseType( AddNoise::Gaussian );
            // std::cout << "\n";
            // std::cout << "Noise Type : Gaussian noise"   << std::endl;
            i++;

        // Poisson
        } else if ( !strncmp( POISSON_TYPE, argv[i], strlen( POISSON_TYPE ) ) ) {
            an->setNoiseType( AddNoise::Poisson );
            // std::cout << "\n";
            // std::cout << "Noise Type : Poisson noise"  << std::endl;
            i++;

        // Outlier
        } else if ( !strncmp( OUTLIER_TYPE, argv[i], strlen( OUTLIER_TYPE ) ) ) {
            an->setNoiseType( AddNoise::Outlier );
            // std::cout << "\n";
            // std::cout << "Noise Type : Outlier noise" << std::endl;
            i++;
        } // end if
    } // end for

    // Add noise
    an->addNoise( ply );

    // Apply color
    std::vector<unsigned char>  colors;
    const kvs::ValueArray<kvs::UInt8> original_colors = ply->colors();
    for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
        colors.push_back( original_colors[3*i]);
        colors.push_back( original_colors[3*i+1] );
        colors.push_back( original_colors[3*i+2] );
    }
    ply->setColors( kvs::ValueArray<kvs::UInt8>( colors ) );

    // Write to spbr file
    const WritingDataType type = Ascii; // Writing data as ascii
    writeSPBR(
        ply,            /* kvs::PolygonObject *_ply     */
        outSPBRfile,    /* char*              _filename */
        type            /* WritingDataType    _type     */
    );

    // Convert "PolygonObject" to "PointObject"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    return 0;
} // End main()