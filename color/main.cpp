// ===================================================
//    Add Gaussian noise to the color of each point
// ===================================================

// USAGE:
// ./addNoise2color [data_file] [output_file] [noise_ratio] [sigma(=standard deviation)]

// EXAMPLE:
// ./addNoise input.ply output.spbr 0.1 5.0

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

const char OUT_FILE[] = "SPBR_DATA/out_noised.spbr";

int main( int argc, char** argv ) {
    char outSPBRfile[512];
    strcpy( outSPBRfile, OUT_FILE ); 

    if ( argc != 5 ) {
        std::cout   << "\n  ";
        std::cout   << "USAGE   : " << argv[0] << " [input_file] [output_file] [noise_ratio] [sigma(=standard deviation)]";
        std::cout   << "\n  ";
        std::cout   << "Example : " << argv[0] << " input.ply output.spbr 0.1 5.0\n" << std::endl;
        exit(1);

    } else if ( argc >= 3 ) {
        strcpy( outSPBRfile, argv[2] );
    }
    
    // Import "point cloud data（.ply, argv[1]）" that user selected
    // Inheritance of KVS::PolygonObject
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "PLY Min, Max Coords:" << std::endl;
    std::cout << "Min : " << ply->minObjectCoord() << std::endl;
    std::cout << "Max : " << ply->maxObjectCoord() << std::endl;

    // Set up for adding noise
    AddNoise *an = new AddNoise( /* noise ratio                 */ atof(argv[3]),
                                 /* sigma(=standard deviation)  */ atof(argv[4]) );

    // Apply Gaussian noise
    an->setNoiseType( AddNoise::Gaussian );

    // Add noise to color
    an->addNoise2Color( /* kvs::PolygonObject* */ ply );

    // ----- Get noise indices -----
    std::vector<bool> is_noise_points = an->getNoisePoints();

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
    
    return 0;
}
