// Uchida-Tomomasas-MacBook-Pro:color uchidatomomasa$ ./addNoise2color 

// =============================================
//      Add Noise to "Color" of Point Cloud
//               Tomomasa Uchida
//                 2020/06/21
// =============================================

//   USAGE:
//   ./addNoise2color [input_file] [output_file] [noise_probability] [sigma(=standard deviation)]

//   EXAMPLE:
//   ./addNoise2color input.ply output.spbr 0.1 5.0

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

const char OUT_FILE[] = "SPBR_DATA/out_color_noise.spbr";

void message() {
    std::cout << "\n";
    std::cout << "=============================================" << std::endl;
    std::cout << "     Add Noise to \"Color\" of Point Cloud"    << std::endl;
    std::cout << "              Tomomasa Uchida"                 << std::endl;
    std::cout << "                2020/06/21"                    << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "\n";
}

int main( int argc, char** argv ) {
    char outSPBRfile[512];
    strcpy( outSPBRfile, OUT_FILE ); 

    // Display message
    message();

    if ( argc != 5 ) {
        std::cout << "  ";
        std::cout << "USAGE:\n  ";
        std::cout << argv[0] << " [input_file] [output_file] [noise_probability] [sigma(=standard deviation)]";
        std::cout << "\n\n  ";
        std::cout << "EXAMPLE:\n  ";
        std::cout << argv[0] << " input.ply output.spbr 0.1 5.0\n" << std::endl;
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
    AddNoise *an = new AddNoise( /* noise probability           */ atof(argv[3]),
                                 /* sigma(=standard deviation)  */ atof(argv[4]) );

    // Apply Gaussian noise
    an->setNoiseType( AddNoise::Gaussian );

    // Add noise to color
    an->addNoise2Color( /* kvs::PolygonObject* */ ply );

    // Get noise indices
    std::vector<bool> is_noise_points = an->getNoisePoints();

    // Write to spbr file
    WritingDataType type = Ascii;   // Writing data as ascii
    writeSPBR( ply,                    /* kvs::PolygonObject *_ply        */  
               //noise_intensities,    /* std::vector<float> &_ni         */  
               outSPBRfile,            /* char*              _filename    */  
               type );                 /* WritingDataType    _type        */  

    // Convert "PolygonObject（KVS）" to "PointObject（KVS）"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 
    
    return 0;
}
