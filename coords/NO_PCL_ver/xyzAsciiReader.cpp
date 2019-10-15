#include <fstream>
#include <cstring>
#include <vector>
#include <algorithm> 
#include <iterator>

#include  "xyzAsciiReader.h"

const int BUF_MAX = 1024;
const float NORM_DATA[3] = {0.0, 0.0, 0.0};
const unsigned char COLOR_DATA[3] = {0, 200, 200};

xyzAsciiReader::xyzAsciiReader( void ) : m_filename( NULL ) {
}

xyzAsciiReader::xyzAsciiReader( char* filename ) : m_filename( filename ) {
    execRead( filename );
}

void xyzAsciiReader::execRead(char* filename) {
    std::ifstream fin( m_filename );
    if( !fin ) {
        std::cerr << "ERROR: Cannot Open ;" << m_filename << std::endl;
        exit(1);
    }

  std::vector<kvs::Real32> coords; 
  std::vector<kvs::Real32> normals;
  std::vector<kvs::UInt8>  colors;  
  
  char buf[ BUF_MAX];      
  std::string word[ 15 ];  
  int num = 0;
  kvs::Vector3f minCoord( 1.0e6, 1.0e6, 1.0e6 );
  kvs::Vector3f maxCoord( -1.0e6, -1.0e6, -1.0e6 );

  while( fin.getline( buf, BUF_MAX - 1, '\n') ) { 
    float x, y, z;
    float nx =  NORM_DATA[0], ny =  NORM_DATA[1], nz =  NORM_DATA[2];
    float f = 0.0;
    unsigned char r = COLOR_DATA[0], g = COLOR_DATA[1], b = COLOR_DATA[2]; 

    if( buf[0] != '#' ) {
      int nw = breakWord( buf, word ); 
      if( nw < 3 ) {
	std::cout << "Out of Reagion" << std::endl;
	exit(1);
      }
      if( nw >=3 ) {
	x = atof( word[0].c_str() );
	y = atof( word[1].c_str() );
	z = atof( word[2].c_str() );
	if( minCoord.x() > x ) minCoord[0] = x;
	if( minCoord.y() > y ) minCoord[1] = y;
	if( minCoord.z() > z ) minCoord[2] = z;
	if( maxCoord.x() < x ) maxCoord[0] = y;
	if( maxCoord.y() < y ) maxCoord[1] = y;
	if( maxCoord.z() < z ) maxCoord[2] = z;
          
	
	if ( nw >= 6 ) {
	  nx = atof( word[3].c_str() );
	  ny = atof( word[4].c_str() );
	  nz = atof( word[5].c_str() );
	    
	  if( nw >= 9 ) {
	    r = (unsigned char)( atoi( word[6].c_str() ) );
	    g = (unsigned char)( atoi( word[7].c_str() ) );
	    b = (unsigned char)( atoi( word[8].c_str() ) );

	    if( nw>=10 ) {
	      f = atof( word[9].c_str() ); 
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
  }
  std::cout << "Number of points: " << num << std::endl;

  m_numVert = num;

  SuperClass::setCoords( kvs::ValueArray<kvs::Real32>( coords ));
  SuperClass::setColors( kvs::ValueArray<kvs::UInt8>( colors ) );
  SuperClass::setNormals( kvs::ValueArray<kvs::Real32>(normals ) );
  SuperClass::setMinMaxObjectCoords( minCoord, maxCoord );

}

int xyzAsciiReader::breakWord( char* buf, std::string *str )
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


