#include <fstream>
#include <cstring>
#include <vector>
#include <algorithm> 
#include <iterator>

#include "xyz_binary_reader.h"
#include "spcomment_xyz.h"

const int BUF_MAX = 1024;
const float NORM_DATA[3] = {0.0, 0.0, 0.0};
const unsigned char COLOR_DATA[3] = {0, 200, 200};

xyzBinaryReader::xyzBinaryReader( void ) :
  m_filename( NULL ) 
{ }

xyzBinaryReader::xyzBinaryReader( char* filename ) :
  m_filename( filename ) 
{
  m_fin.open( m_filename );
  if( !m_fin ) {
    std::cerr << "ERROR: Cannot Open ;" << m_filename << std::endl;
    exit(1);
  }

  execReadHeader( );
  execReadData( );

  m_fin.close();
}

void xyzBinaryReader::execReadHeader( void ) 
{

  char buf[ BUF_MAX ];
  std::string word[ 15 ];  
  int numPoints = 0;

  m_fin.getline( buf, BUF_MAX - 1, '\n');
  if( strncmp( buf, XYZ_BINARY, strlen( XYZ_BINARY ) ) ) {
    std::cout << m_filename << " is not xyz Binary Data " << std::endl; 
  }

  while( m_fin.getline( buf, BUF_MAX - 1, '\n') ) { 
    breakWord( buf, word ); 
    if( !strlen (buf) ) { 
      continue;
    }
    else if( buf[0] == '#' ) { 
      if( !strncmp( word[0].c_str(), XYZ_NUM_PARTICLES, strlen( XYZ_NUM_PARTICLES ) ) ) {
	  numPoints = atoi( word[1].c_str() );
	  m_numVert = numPoints;
	}
	else if( !strncmp( word[0].c_str(), XYZ_DATA_TYPE, strlen( XYZ_DATA_TYPE ) ) ) {
	  if(  !strncmp( word[1].c_str(), XYZ_NCF, strlen( XYZ_NCF ) ) ) {
	    m_numData = 10;
	  }
	  else if( !strncmp( word[1].c_str(), XYZ_N, strlen( XYZ_NC ) ) ) {
	    m_numData = 9;
	  }
	  else if( !strncmp( word[1].c_str(), XYZ_N, strlen( XYZ_N ) ) ) {
	    m_numData = 6;
	  }
	  else if( !strncmp( word[1].c_str(), XYZ, strlen( XYZ ) ) ) {
	    m_numData = 3; 
	  }
	  else {
	    std::cout << "Out of Range in xyzBinaryReader" << std::endl;
	    exit(1);
	  }
	}
	else if( !strncmp( word[0].c_str(), XYZ_END_HEADER, strlen( XYZ_END_HEADER ) ) )  {
	  return;
	}
    } // end else if( buf[0] == '#' )  
  } // end while

}

void xyzBinaryReader::execReadData( void ) 
{

  std::vector<kvs::Real32> coords; 
  std::vector<kvs::Real32> normals;
  std::vector<kvs::UInt8>  colors;  
  
  kvs::Vector3f minCoord( 1.0e6, 1.0e6, 1.0e6 );
  kvs::Vector3f maxCoord( -1.0e6, -1.0e6, -1.0e6 );

  int num = 0;
  for( int i=0; i<m_numVert; i++ ) {
    float x = 0.0, y = 0.0, z = 0.0; 
    float nx =  NORM_DATA[0], ny =  NORM_DATA[1], nz =  NORM_DATA[2];
    float f = 0.0;
    unsigned char r = COLOR_DATA[0], g = COLOR_DATA[1], b = COLOR_DATA[2]; 
    if( m_numData < 3 ) {
      std::cout << "Out of Reagion" << std::endl;
      exit(1);
    }
    if( m_numData >=3 ) {
      m_fin.read( (char*)&x, sizeof(kvs::Real32) );
      m_fin.read( (char*)&y, sizeof(kvs::Real32) );
      m_fin.read( (char*)&z, sizeof(kvs::Real32) );
      if( minCoord.x() > x ) minCoord[0] = x;
      if( minCoord.y() > y ) minCoord[1] = y;
      if( minCoord.z() > z ) minCoord[2] = z;
      if( maxCoord.x() < x ) maxCoord[0] = y;
      if( maxCoord.y() < y ) maxCoord[1] = y;
      if( maxCoord.z() < z ) maxCoord[2] = z;
      if ( m_numData >= 6 ) {
	m_fin.read( (char*)&nx, sizeof(kvs::Real32) );
	m_fin.read( (char*)&ny, sizeof(kvs::Real32) );
	m_fin.read( (char*)&nz, sizeof(kvs::Real32) );
	if( m_numData >= 9 ) {
	  m_fin.read( (char*)&r, sizeof(kvs::UInt8) );
	  m_fin.read( (char*)&g, sizeof(kvs::UInt8) );
	  m_fin.read( (char*)&b, sizeof(kvs::UInt8) );	  
	  if( m_numData >=10 ) {
	    m_fin.read( (char*)&f, sizeof(kvs::Real32) );
	  }
	} 
      }
    }
    coords.push_back( x );
    coords.push_back( y );
    coords.push_back( z );
    normals.push_back( nx );
    normals.push_back( ny );
    normals.push_back( nz );
    colors.push_back( r );
    colors.push_back( g );
    colors.push_back( b );
    m_ft.push_back( f );
    num++;    

  }

  SuperClass::setCoords( kvs::ValueArray<kvs::Real32>( coords ));
  SuperClass::setColors( kvs::ValueArray<kvs::UInt8>( colors ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>(normals ) );
  SuperClass::setMinMaxObjectCoords( minCoord, maxCoord );

}

int xyzBinaryReader::breakWord( char* buf, std::string *str )
{                                                     
  char* data;                                         
  int n = 0;                                          
  data = strtok( buf, " \t" );                        
  while( data != NULL ) {                             
    str[n] = data;                                    
    data = strtok( NULL, " \t" );                     
    n++;                                              
  }                                                   
  return n;                                           
}                                                     
