// ============================================
//      Add Noise to Coords of Point Cloud
//               Tomomasa Uchida
//                 2021/02/06
// ============================================

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "import_point_clouds.h"
#include "add_noise.h"
#include "write_spbr.h"
#include "noise_type.h"

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

const char OUT_FILE[] = "SPBR_DATA/out_coords_noise.spbr";

inline void message() {
    std::cout << "\n";
    std::cout << "=============================================" << std::endl;
    std::cout << "     Add Noise to \"Coords\" of Point Cloud"   << std::endl;
    std::cout << "               Tomomasa Uchida"                << std::endl;
    std::cout << "                 2021/02/06"                   << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "\n";
}

inline void display_usage( char* _argv0 ) {
    std::cout   << "  USAGE:\n  "
                << _argv0
                << " [input_file] [output_file] [noise_probability] [sigma] [noise_type]"
                << "\n\n  EXAMPLE:\n  "
                << _argv0
                << " input.ply output.spbr 0.1 0.02 -g"
                << "\n\n"
                << "   [noise_probability]\n"
                << "    Add noise with 10(=0.1*100) percent."
                << "\n\n"
                << "   [sigma]\n"
                << "    Gaussian: sigma = 0.02\n"
                << "    Outlier : none(skip the sigma.)"
                << "\n\n"
                << "   [noise_type]\n"
                << "    -g: Gaussian noise\n"
                << "    -o: Outlier noise\n"
                << std::endl;
}

int main( int argc, char** argv ) {
    char outSPBRfile[512];
    strcpy( outSPBRfile, OUT_FILE ); 
    message();

    if ( argc != 6 ) {    
        display_usage( argv[0] );
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[2] );
    } // end if
    
    // Import the input point cloud
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "\n";
    std::cout << "Bounding Box:" << "\n";
    std::cout << " Min: " << ply->minObjectCoord() << "\n";
    std::cout << " Max: " << ply->maxObjectCoord() << "\n";

    // Instantiate "AddNoise" class
    AddNoise *an = new AddNoise(
        atof( argv[3] ),  /* double _noise_prob */ 
        atof( argv[4] )   /* double _sigma      */
    );

    // Set noise type
    for ( size_t i = 1; i < argc; i++ ) {
        // Gaussian
        if ( !strncmp( GAUSSIAN_TYPE, argv[i], strlen( GAUSSIAN_TYPE ) ) ) {
            an->setNoiseType( AddNoise::Gaussian );
            i++;

        // Outlier
        } else if ( !strncmp( OUTLIER_TYPE, argv[i], strlen( OUTLIER_TYPE ) ) ) {
            an->setNoiseType( AddNoise::Outlier );
            i++;
        }
    } // end for

    // Add noise
    an->addNoise2Coords( ply );

    // Write to spbr file
    const WritingDataType type = Ascii;
    writeSPBR(
        ply,            /* kvs::PolygonObject *_ply     */
        outSPBRfile,    /* char*              _filename */
        type            /* WritingDataType    _type     */
    );

    // Convert "PolygonObject" to "PointObject"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    // Exec. SPBR
    const std::string output_file( outSPBRfile );
    std::string EXEC( "spbr " );
    EXEC += output_file;
    system( EXEC.c_str() );

    return 0;
}