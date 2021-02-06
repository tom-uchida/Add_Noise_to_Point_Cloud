#include <kvs/PolygonImporter>
#include "ply_read.h"
#include <fstream>
#include <cstring>

const int BUF_SIZE = 512;

plyRead::plyRead( void): m_filename( NULL ) {
    hasFace = false;    hasColor = false;  hasAlpha = false;
    hasNormal = false;  isAscii= false;
}
  
plyRead::plyRead( char* filename, bool &hface ): m_filename( filename ) {
    //-- Initializing Flags 
    hasFace = false;    hasColor = false; hasAlpha = false;
    hasNormal = false;  isAscii= false;

    //--- Check File Type and Header
    checkHasFace( );

    //---- Reading Data
    if( hasFace ) {		//-- With KVS Importer
        //    std::cout << "Using KVS Importer " << std::endl;
        kvs::PolygonObject *polygon = new kvs::PolygonImporter( m_filename );
        SuperClass::setCoords( polygon->coords() );            
        SuperClass::setColors( polygon->colors() );
        SuperClass::setNormals( polygon->normals() );          
        SuperClass::setConnections( polygon->connections() );  
        SuperClass::setOpacities( polygon->opacities() );      
        SuperClass::setPolygonType( polygon->polygonType() );  
        SuperClass::setColorType( polygon->colorType() );      
        SuperClass::setNormalType( polygon->normalType() );
        SuperClass::setMinMaxObjectCoords( polygon->minObjectCoord(), polygon->maxObjectCoord() );                       
        SuperClass::setMinMaxExternalCoords( polygon->minExternalCoord(), polygon->maxExternalCoord() );
    
        delete polygon;
    
    } else {			//--- Without KVS Importer
        if( isAscii )
            execReadAscii( filename );
        else
            execReadBinary( filename );
    }
    
    hface = hasFace;
}

void plyRead::checkHasFace( void ) {
    std::ifstream fin( m_filename );
    if( !fin ) {
        std::cerr << "ERROR: Cannot Open ;" << m_filename << std::endl;
        exit(1);
    }

    char buf[BUF_SIZE];
  
    fin.getline( buf, BUF_SIZE, '\n' );
    if( strncmp( buf, "ply", 3 ) != 0 ) {
        std::cerr << "ERROR: " << m_filename <<" is not PLY Format "<< std::endl;
        exit(1);
    }
  
    char words[ 20 ][BUF_SIZE];
    char *p;
    int numT;
    int ifElem = 0, numData = 0, numNom = 0, numCol = 0; 

    while( fin.getline( buf, BUF_SIZE, '\n' ) ) {
        numT = 0;
        p = buf;
        //--- Devide Token
        while( (p=strtok(p, " \t") ) !=NULL ) {
            strcpy( words[numT], p );
            numT++;
            p = NULL;
    }
    
    //--- is ASCii
    if( !strncmp( words[0], "format", 6 ) ) 
        if( !strncmp( words[1], "ascii", 5 )  ) isAscii = true;
    
    //--- Number of Vertices and Faces
    if( !strncmp( words[0], "element", 7 ) )  {
        if( !strncmp( words[1], "vertex", 6 ) )
	       numVert = atoi( words[2] ); 
        
        else if( !strncmp( words[1], "face", 4 ) ) { 
	       numFace = atoi( words[2] ); 
	       ifElem = 1;
           if( numFace != 0 ) hasFace = true;
        }
    }
    
    //--- Check Property
    if( !strncmp( words[0], "property", 8 ) && ( ifElem == 0 ) ) {
        numData++;
        if( !strncmp( words[2], "nx", 2 ) ) {
	       numNom = numData -1;
	       hasNormal = true;
        
        } else if( !strncmp( words[2], "red", 3 ) ) {
	       numCol = numData -1; 
	       hasColor = true;
        
        } else if ( !strncmp( words[2], "alpha", 5 ) ) 
	       hasAlpha = true;
    }
    
    //--- End Header
    if( !strncmp( words[0], "end_header", 10 ) ){ 
        break;
    }
  }
}

void plyRead::execReadAscii( char* filename)
{
    std::cout << "ASCII Data " << std::endl;
    std::ifstream fin( m_filename );
    if( !fin ) {
        std::cerr << "ERROR: Cannot Open ;" << m_filename << std::endl;
        exit(1);
    }
    
    //--- Skip Header
    char buf[BUF_SIZE];
    while( fin.getline( buf, BUF_SIZE, '\n' ) ) 
        if( !strncmp( buf, "end_header", 10 ) ){ 
        break;
    }
    std::vector<kvs::Real32> coords; 
    std::vector<kvs::Real32> normals;
    std::vector<kvs::UInt8>  colors;  
    float p[3];
    float np[3];
    unsigned char c[3];
    int slide = 0;
    char words[ 20 ][BUF_SIZE];
    char *ps;
    int numT;
    kvs::Vector3f minCoord( 1.0e6, 1.0e6, 1.0e6 );
    kvs::Vector3f maxCoord( -1.0e6, -1.0e6, -1.0e6 );
    
    //--- Reading Data
    while( fin.getline( buf, BUF_SIZE, '\n' ) ) {
        numT = 0;
        ps = buf;
    
    //--- Devide Token
    while( (ps=strtok(ps, " \t") ) !=NULL ) {
        strcpy( words[numT], ps );
        numT++;
        ps = NULL;
    }
    p[0] = atof( words[0] );
    p[1] = atof( words[1] );
    p[2] = atof( words[2] );
    coords.push_back( p[0] );
    coords.push_back( p[1] );
    coords.push_back( p[2] );
    if( minCoord.x() > p[0] ) minCoord[0] = p[0];
    if( minCoord.y() > p[1] ) minCoord[1] = p[1];
    if( minCoord.z() > p[2] ) minCoord[2] = p[2];
    if( maxCoord.x() < p[0] ) maxCoord[0] = p[0];
    if( maxCoord.y() < p[1] ) maxCoord[1] = p[1];
    if( maxCoord.z() < p[2] ) maxCoord[2] = p[2];
    if( hasNormal ) {
        np[0] = atof( words[3] );
        np[1] = atof( words[4] );
        np[2] = atof( words[5] );
        normals.push_back( np[0] );
        normals.push_back( np[1] );
        normals.push_back( np[2] );
        slide = 3;
    }    
    if( hasColor ) {
        c[0] = static_cast<unsigned char>( atoi( words[ slide + 3 ] ) );
        c[1] = static_cast<unsigned char>( atoi( words[ slide + 4 ] ) );
        c[2] = static_cast<unsigned char>( atoi( words[ slide + 5 ] ) );
        colors.push_back( c[0] );      
        colors.push_back( c[1] );      
        colors.push_back( c[2] );      
    }
  }

  //  std::cout << "min: " << minCoord << " max: " << maxCoord <<std::endl;
  SuperClass::setCoords( kvs::ValueArray<kvs::Real32>( coords ));
  SuperClass::setColors( kvs::ValueArray<kvs::UInt8>( colors ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>(normals ) );
  SuperClass::setMinMaxObjectCoords( minCoord, maxCoord );
}

void plyRead::execReadBinary( char* filename) {
    std::ifstream fin( m_filename );
    if( !fin ) {
        std::cerr << "ERROR: Cannot Open ;" << m_filename << std::endl;
        exit(1);
    }
    
    //--- Skip Header  
    char buf[BUF_SIZE];
    while( fin.getline( buf, BUF_SIZE, '\n' ) ) 
        if( !strncmp( buf, "end_header", 10 ) ){ 
            break;
    }  
    std::vector<kvs::Real32> coords; 
    std::vector<kvs::Real32> normals;
    std::vector<kvs::UInt8>  colors;
    
    float p[3];
    float np[3];
    unsigned char c[3];
    unsigned char al;
    kvs::Vector3f minCoord( 1.0e6, 1.0e6, 1.0e6 );
    kvs::Vector3f maxCoord( -1.0e6, -1.0e6, -1.0e6 );
    
    for(int i=0; i<numVert; ++i ) {
        fin.read( (char*) p, sizeof(float)*3 );
        coords.push_back( p[0] );
        coords.push_back( p[1] );
        coords.push_back( p[2] );
        if( minCoord.x() > p[0] ) minCoord[0] = p[0];
        if( minCoord.y() > p[1] ) minCoord[1] = p[1];
        if( minCoord.z() > p[2] ) minCoord[2] = p[2];
        if( maxCoord.x() < p[0] ) maxCoord[0] = p[0];
        if( maxCoord.y() < p[1] ) maxCoord[1] = p[1];
        if( maxCoord.z() < p[2] ) maxCoord[2] = p[2];
      
        //---- Normal 
        if( hasNormal ) {
            fin.read( (char*)np, sizeof(float)*3 );  
            normals.push_back( np[0] );
            normals.push_back( np[1] );
            normals.push_back( np[2] );
            std::cout << np[0] << std::endl;
        }
    
        //---- Color
        if( hasColor ) {
            fin.read( (char*)c, sizeof(unsigned char)*3 );
            colors.push_back( c[0] );      
            colors.push_back( c[1] );      
            colors.push_back( c[2] );      
        }
        
        if( hasAlpha )
            fin.read( (char*)&al, sizeof(unsigned char) );
    
    }
    SuperClass::setCoords( kvs::ValueArray<kvs::Real32>( coords ));
    SuperClass::setColors( kvs::ValueArray<kvs::UInt8>( colors ) );
    SuperClass::setNormals( kvs::ValueArray<kvs::Real32>(normals ) );
    SuperClass::setMinMaxObjectCoords( minCoord, maxCoord );
}
    
