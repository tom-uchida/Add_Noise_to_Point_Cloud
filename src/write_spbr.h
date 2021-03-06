#ifndef _writeSPBR_H__
#define _writeSPBR_H__

#include <kvs/PolygonObject>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <time.h>

enum WritingDataType {
    Ascii = 0,
    Binary = 1
};

// default
const float NORMAL[3]   = { 0.0, 0.0, 0.0 };
const int   COLOR[3]    = { 255, 0, 0 };

/*===========================================================================*/
/**
 *  @brief  Write noise intensity to outfile.
 *  @param  *ply        point cloud
 *  @param  filename    output file name
 *  @param  type        Ascii or Binary
 **/
/*===========================================================================*/
void writeSPBR(
    kvs::PolygonObject *_ply,
    char* _filename,
    WritingDataType _type = Ascii )
{

    size_t  num         = _ply->numberOfVertices(); 
    bool    hasNormal   = false;
    bool    hasColor    = false;
    if( num == _ply->numberOfNormals() ) hasNormal   = true; 
    if( num == _ply->numberOfColors() )  hasColor    = true;
    kvs::ValueArray<kvs::Real32>         coords      = _ply->coords();  
    kvs::ValueArray<kvs::Real32>         normals     = _ply->normals();
    kvs::ValueArray<kvs::UInt8>          colors      = _ply->colors();

    
    std::ofstream fout( _filename );
    if ( _type == Ascii ) {
        fout << "#/SPBR_ASCII_Data"       << std::endl;
        fout << "#/RepeatLevel 1"         << std::endl;
        fout << "#/BGColorRGBByte 0 0 0"  << std::endl;
        fout << "#/ImageResolution 512"   << std::endl;
        fout << "#/Shading 0"             << std::endl;
        fout << "#/EndHeader"             << std::endl;
    }

    // Write to ouput spbr file
    std::cout << "\n";
    std::cout << "Writing spbr file (" << _filename << ")..." << "\n";
    const clock_t start = clock();
    for ( int i = 0; i < num; i++ ) {
        // coords                                               
        float x = coords[3*i];
        float y = coords[3*i+1];
        float z = coords[3*i+2];

        // normal(default)
        float nx = NORMAL[0];
        float ny = NORMAL[1];
        float nz = NORMAL[2];
        if ( hasNormal ) {
            nx = normals[ 3*i  ];
            ny = normals[ 3*i+1];
            nz = normals[ 3*i+2];
        }

        // color(default)
        int r = COLOR[0];
        int g = COLOR[1];
        int b = COLOR[2];
        if ( hasColor ) {
            r = (int)colors[3*i];
            g = (int)colors[3*i+1];
            b = (int)colors[3*i+2];
        }

        if( _type == Binary ) {
            fout.write( (char*)&x, sizeof(float) );
            fout.write( (char*)&y, sizeof(float) );
            fout.write( (char*)&z, sizeof(float) );
            fout.write( (char*)&nx, sizeof(float) );
            fout.write( (char*)&ny, sizeof(float) );
            fout.write( (char*)&nz, sizeof(float) );

            unsigned char cl = (unsigned char)r;
            fout.write( (char*)&cl, sizeof(unsigned char) );
            cl = (unsigned char)g;
            fout.write( (char*)&cl, sizeof(unsigned char) );
            cl = (unsigned char)b;
            fout.write( (char*)&cl, sizeof(unsigned char) );
        
        // Ascii
        } else {     
            fout    << x   << " " << y  << " " << z  << " "
                    << nx  << " " << ny << " " << nz << " "
                    << r   << " " << g  << " " << b  << " " 
                    << "\n";
        } // end if

        // Display progress
        const double processing_ratio = 100.0 * (double)i / (double)num;
        if ( !(i % 1000000) && i > 0 ) { 
            std::cout << "*** Num. of processed points: " << i;
            std::cout << " [" << processing_ratio << " %]\n";
        }
    } // end for

    const clock_t end = clock();
    std::cout << "Done writing spbr file!";
    std::cout << " ( " << (double)(end - start) / CLOCKS_PER_SEC << " [sec] )\n";

    fout.close();
}


#endif
