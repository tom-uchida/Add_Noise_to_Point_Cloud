////////////////////
///// spbr.cpp /////
////////////////////

#include <fstream>
#include <iostream>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "spcomment.h"
#include "spbr.h"
#include "single_inputfile.h" 

//#define DEBUG
//#define DEBUG_COLOR


const unsigned int DEFAULT_REPEAT_LEVEL     = 1 ;
const unsigned int DEFAULT_IMAGE_RESOLUTION = 512 ;
const double       DEFAULT_VIEW_ANGLE       = 45.0 ;
const unsigned int DEFAULT_GRAY_LEVEL       = 128 ;
const unsigned int DEFAULT_BG_GRAY_LEVEL    = 255 ;


//-----
SPBR::SPBR( const char* input_file )  : 
    m_cameraPosition (0.0, 0.0, DEFAULT_CAMERA_DISTANCE) ,
    m_lookAt         (0.0, 0.0, 0.0 )             ,
    m_viewAngle      ( DEFAULT_VIEW_ANGLE )       ,
    m_flagCameraFar  ( false )                    , 
    m_repeatLevel(DEFAULT_REPEAT_LEVEL)           ,
    m_Rb(DEFAULT_GRAY_LEVEL)                      , 
    m_Gb(DEFAULT_GRAY_LEVEL)                      , 
    m_Bb(DEFAULT_GRAY_LEVEL)                      , 
    m_flagReverseNormals   (false)                ,
    m_flagUseNormals       (true)                 , 
    m_flagForcedColor      (false)                , 
    m_flagParticleZoom     (true)                 , 
    m_numParticles(0)                             , 
    m_pointSize(1)                                ,
    m_flagDrawWireframeBox (false)                , 
    m_minWBPosition        ( 0.0, 0.0, 0.0 )      , 
    m_maxWBPosition        ( 0.0, 0.0, 0.0 )      ,  
    m_flagFPS              (false)                , 
    m_BG_Rb(DEFAULT_BG_GRAY_LEVEL)                , 
    m_BG_Gb(DEFAULT_BG_GRAY_LEVEL)                , 
    m_BG_Bb(DEFAULT_BG_GRAY_LEVEL)                ,
    m_imageResolution ( DEFAULT_IMAGE_RESOLUTION ),
    m_flagCoordTransformed (false)                ,
    m_O  (0.0, 0.0, 0.0)                          , 
    m_e1 (1.0, 0.0, 0.0)                          , 
    m_e2 (0.0, 1.0, 0.0)                          , 
    m_e3 (0.0, 0.0, 1.0)                          ,
    m_flagLOD              (true)                 ,
    m_flagObjZXRot         (false)                ,
    m_flagForcedBoundingBox(false)                , 
    m_forcedMinPoint(-1.0, -1.0, -1.0 )           ,
    m_forcedMaxPoint(+1.0, +1.0, +1.0 )           ,
    m_flagShuffle          (true)                 ,
    m_flagForcedShuffle    (false)                ,
    m_flagCameraProjectionType (kvs::Camera::Perspective), 
    m_flagLambertShading (true), 
    m_kd(DEFAULT_PHONG_KD), m_ka(DEFAULT_PHONG_KA), 
    m_ks(DEFAULT_PHONG_KS), m_shininess(DEFAULT_PHONG_SHININESS), 
    m_unirand()
{
  // Input file name
  strcpy( m_input_file, input_file );  
  SingleInputFile* p  = SingleInputFile::GetInstance();
  p->SetName( m_input_file ); 

  // Initialize ZX rotation parameters
  m_objZXRotAngle[0] = 0.0;  m_objZXRotAngle[1] = 0.0;  

  // Ceate a point object
  generatePointObject( this );
}


//-----
void SPBR::generatePointObject( kvs::PointObject* point_object )
{
  // Read header (first reading)
  readHeader_and_countParticles() ; 

  // Read point data (second reading) 
  readPointData ()                ; 

  // Set bounding box of the pointObject
  setMinMaxCoords();
}


//-----
void 
SPBR::readHeader_and_countParticles ( void )
{
  // Open input data file
  std::ifstream     reader ; reader.open ( m_input_file );
  if( !reader ) {
    std::cerr << "ERROR: Cannot open " << m_input_file << std::endl;
    exit(1);
  }

  // Read data
  m_numParticles = 0 ; // clear particle counter
  char       buf[ SPBR_BUF_SIZE ] ;
  char       dummy [64] ;

  // Read commands and point data
  while ( reader.getline( buf, SPBR_BUF_SIZE - 1 , '\n' ) ) {

#if defined DEBUG
    std::cout << "READ_PARAM: " << buf << std::endl;
#endif

    // Blank line
    if( !strlen (buf) ) { 
      continue ; // skip a blank line
    } else 
    // Commmand line (#/...) or comment line (#...)
    if( buf[0] == '#' ) {

      // Command
      if(buf[1] == '/') { 
	//===== Commands skipped when reading the header part =====// //TANAKA
        //----- SPBR_ASCII_DATA -----
        if ( !strncmp( buf, SPBR_ASCII_DATA_COMMAND, strlen(SPBR_ASCII_DATA_COMMAND) ) ) { 
          // Do nothing (ignored)
          std::cout << "** SPBR ascii data: " << m_input_file << std::endl;
        } else
        //----- ColorRGBByte (renamed from ColorByteRGB) -----
        // NOTE: The ColorRGBByte block must be written before the ColorRGB block
        if ( !strncmp( buf, COLOR_BYTE_COMMAND, strlen(COLOR_BYTE_COMMAND) ) ) { 
          // Do nothing (to be considered in readPointData())
        } else
        //----- ColorRGB -----
        if ( !strncmp( buf, COLOR_COMMAND, strlen(COLOR_COMMAND) ) ) { 
          // Do nothing (to be considered in readPointData())
        }  else
        //----- ReverseNormals -----
        if ( !strncmp( buf, REVERSE_NORMALS_COMMAND, strlen(REVERSE_NORMALS_COMMAND) ) ) { 
          // Do nothing (to be considered in readPointData())
        } else
        //----- Origin -----
        if ( !strncmp( buf, ORIGIN_COMMAND, strlen(ORIGIN_COMMAND) ) ) { 
          // Do nothing (to be considered in readPointData())
        }  else
        //----- BaseVector -----
        if ( !strncmp( buf, BASE_VECTOR_COMMAND, strlen(BASE_VECTOR_COMMAND) ) ) { 
          // Do nothing (to be considered in readPointData())
        }  else
        //=================================================================//
        //----- Point size (for OPBR) -----
        if ( !strncmp( buf, POINT_SIZE_COMMAND, strlen(POINT_SIZE_COMMAND) ) ) { 
            unsigned int point_size;
            sscanf ( buf, "%s %u", dummy, &point_size );
            this->setPointSize ( point_size ) ;
        } else 
        //----- BGColorRGBByte (renamed from BGColorByteRGB)-----
        if ( !strncmp( buf, BG_COLOR_BYTE_COMMAND, strlen(BG_COLOR_BYTE_COMMAND) ) ) { 
          unsigned int R, G, B;
          sscanf ( buf, "%s %u %u %u", dummy, &R, &G, &B );
	  //          m_BG_Rb = R;  m_BG_Gb = G; m_BG_Bb = B ;
          setBackGroundColor (R, G, B);
        }  else
        //----- Shading (renamed from UseNormals)-----
        if ( !strncmp( buf, SHADING_COMMAND, strlen(SHADING_COMMAND) ) ) { 
	  int flag = 1 ; // default
          sscanf ( buf, "%s %d", dummy, &flag );
          setFlagUseNormals ( flag ) ;
        } else
        //----- FPS -----
        if ( !strncmp( buf, FPS_COMMAND, strlen(FPS_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
          setFlagFPS ( flag ) ;
        } else
        //----- LOD -----
        if ( !strncmp( buf, LOD_COMMAND, strlen(LOD_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
          setFlagLOD ( flag ) ;
        } else
        //----- ImageResolution -----
        if ( !strncmp( buf, IMAGE_RESOLUTION_COMMAND, strlen(IMAGE_RESOLUTION_COMMAND) ) ) { 
          unsigned int resolution;
          sscanf ( buf, "%s %u", dummy, &resolution );
          m_imageResolution = resolution;
        }  else
        //----- ParticleZoom -----
        if ( !strncmp( buf, PARTICLE_ZOOM_COMMAND, strlen(PARTICLE_ZOOM_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
	  setFlagParticleZoom ( flag ) ;
        } else
        //----- RepeatLevel -----
        if ( !strncmp( buf, REPEAT_LEVEL_COMMAND, strlen(REPEAT_LEVEL_COMMAND) ) ) { 
            unsigned int repeat_level;
            sscanf ( buf, "%s %u", dummy, &repeat_level );
            this->setRepeatLevel ( repeat_level ) ;
        } else 
        //----- WireframeBox -----
        if ( !strncmp( buf, WIREFRAME_BOX_COMMAND, strlen(WIREFRAME_BOX_COMMAND) ) ) { 
          double xmin, ymin, zmin, xmax, ymax, zmax ;
          sscanf ( buf, "%s %lg %lg %lg %lg %lg %lg", dummy, &xmin, &ymin, &zmin, &xmax, &ymax, &zmax ) ;
	  setWireframeBox ( xmin, ymin, zmin, xmax, ymax, zmax ) ;
          m_flagDrawWireframeBox = true ;
        } else 
        //----- BoundingBox -----
        if ( !strncmp( buf, BOUNDING_BOX_COMMAND, strlen(BOUNDING_BOX_COMMAND) ) ) { 
          double xmin, ymin, zmin, xmax, ymax, zmax ;
          sscanf ( buf, "%s %lg %lg %lg %lg %lg %lg", dummy, &xmin, &ymin, &zmin, &xmax, &ymax, &zmax ) ;
	  kvs::Vector3f minPoint ( xmin, ymin, zmin );
	  kvs::Vector3f maxPoint ( xmax, ymax, zmax );
	  setBoundingBox ( minPoint, maxPoint ) ;
        } else 
        //----- CameraPosition -----
        if ( !strncmp( buf, CAMERA_POSITION_COMMAND, strlen(CAMERA_POSITION_COMMAND) ) ) { 
          double Ex, Ey, Ez ;
          sscanf ( buf, "%s %lg %lg %lg", dummy, &Ex, &Ey, &Ez );
	  m_cameraPosition.set( Ex, Ey, Ez );
        } else 
        //----- LookAt -----
        if ( !strncmp( buf, LOOK_AT_COMMAND, strlen(LOOK_AT_COMMAND) ) ) { 
          double LAx, LAy, LAz ;
          sscanf ( buf, "%s %lg %lg %lg", dummy, &LAx, &LAy, &LAz );
	  m_lookAt.set( LAx, LAy, LAz );
        } else 
        //----- ViewAngle -----
        if ( !strncmp( buf, VIEW_ANGLE_COMMAND, strlen(VIEW_ANGLE_COMMAND) ) ) { 
          double angle_deg ;
          sscanf ( buf, "%s %lg", dummy, &angle_deg );
          setViewAngle ( angle_deg );
        }  else
        //----- CameraFar ----- 
        if ( !strncmp( buf, CAMERA_FAR_COMMAND, strlen(CAMERA_FAR_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
	  setFlagCameraFar ( flag ) ;
        } else 
        //----- CameraZoom ----- 
        if ( !strncmp( buf, CAMERA_ZOOM_COMMAND, strlen(CAMERA_ZOOM_COMMAND) ) ) { 
          double f_zoom ;
          sscanf ( buf, "%s %lf", dummy, &f_zoom );
	  setCameraZoom ( f_zoom ) ;
        } else 
        //----- ObjectZXRot ----- 
        if ( !strncmp( buf, OBJECT_ZX_ROT_COMMAND, strlen(OBJECT_ZX_ROT_COMMAND) ) ) { 
	  double zrot_angle_deg, xrot_angle_deg ;
          sscanf( buf, "%s %lg %lg", dummy, &zrot_angle_deg, &xrot_angle_deg );

          setObjZXRot ( zrot_angle_deg, xrot_angle_deg ) ;
        } else 
        //----- Shuffle ----- 
        if ( !strncmp( buf, SHUFFLE_COMMAND, strlen(SHUFFLE_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
	  setFlagShuffle ( flag ) ;
        } else 
        //----- ForcedShuffle ----- 
        if ( !strncmp( buf, FORCED_SHUFFLE_COMMAND, strlen(FORCED_SHUFFLE_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
	  setFlagForcedShuffle ( flag ) ;
        } else 
        //----- OrhtogonalCamera ----- 
        if ( !strncmp( buf, ORTHOGONAL_CAMERA_COMMAND, strlen(ORTHOGONAL_CAMERA_COMMAND) ) ) { 
          setOrthogonalCamera () ;
        } else 
        //----- PerspectiveCamera ----- 
        if ( !strncmp( buf, PERSPECTIVE_CAMERA_COMMAND, strlen(PERSPECTIVE_CAMERA_COMMAND) ) ) { 
          setPerspectiveCamera () ;
        } else 
        //----- LambertShading
        if ( !strncmp( buf, LAMBERT_SHADING_COMMAND, strlen(LAMBERT_SHADING_COMMAND) ) ) { 
	  double kd, ka ;
          int num_words = sscanf ( buf, "%s %lg %lg", dummy, &kd, &ka );
          if( num_words == 1 ) { kd = DEFAULT_LAMBERT_KD; ka = DEFAULT_LAMBERT_KA; }
          setLambertShading ( kd, ka ) ;
        } else 
        //----- PhongShading
        if ( !strncmp( buf, PHONG_SHADING_COMMAND, strlen(PHONG_SHADING_COMMAND) ) ) { 
	  double kd, ka, ks;
          int     shininess;
          int num_words = sscanf ( buf, "%s %lg %lg %lg %d", dummy, &kd, &ka, &ks, &shininess );
          if( num_words == 1 ) { 
            kd = DEFAULT_PHONG_KD; ka = DEFAULT_PHONG_KA; 
            ks = DEFAULT_PHONG_KS; shininess = DEFAULT_PHONG_SHININESS;
          }
          setPhongShading ( kd, ka, ks, shininess ) ;
        } else 
        //----- NumParticles
        if ( !strncmp( buf, NUM_PARTICLES_COMMAND , strlen(NUM_PARTICLES_COMMAND) ) ) { 
          unsigned int n_tmp;
          sscanf ( buf, "%s %u", dummy, &n_tmp );
          m_numParticles = (size_t)n_tmp; // TANAKA_SPBR
          std::cout << "** #/NumParticles command is found: ";
          std::cout <<  m_numParticles << " points." << std::endl;
        } else 
        //----- EndHeadr
        if ( !strncmp( buf, END_HEADER_COMMAND, strlen(END_HEADER_COMMAND) ) ) { 
          std::cout << "** #/EndHeader command is found." << std::endl;

          // If m_numPoints have already been set by the #/NumPoints command,
          // exit from readHeader().
          if( m_numParticles > 1) { 
	    std::cout << "**   Number of points is not counted. " << std::endl;
	    std::cout << "     The value of the #/NumParticle is used. " << std::endl;
            break; // get out of the while loop to read lines
          }

        } else 
        //-----------------------
        {
	  std::cerr << "!!! WARNING (in reading the header):\n    Unknown command \"" << buf << "\" ";
	  std::cerr << "is ignored. " << std::endl;
	  //          exit(1);
	}


      } // if(buf[1] == '/')  
      else {
        // Comment line ("#..." do nothing)
      }

    } // if( buf[0] == '#' ) 

    // Point data
    else {
      m_numParticles++ ;  // count num particles
    } // if-else

  } // while (reader.getline())

  // Close files
  //.. input SPBR file
  reader.close ();

  // Message
  std::cout << "** Reading the header part is completed:" << std::endl;
  std::cout << "**   Number of points     : " << m_numParticles << std::endl;
  std::cout << "**   Use of normal vectors: " ;
  std::cout << (m_flagUseNormals ? "Yes": "No") ;
  std::cout << std::endl;
  std::cout << "**   Repeat level         : " << m_repeatLevel << std::endl;

} // readHeader_and_countParticles()


//-----
void 
SPBR::readPointData( void )
{
  // Open input data file
  std::ifstream     reader ; reader.open ( m_input_file );
  if( !reader ) {
    std::cerr << "ERROR: Cannot open " << m_input_file << std::endl;
    exit(1);
  }


  // ValueArrays for coords, normals
  kvs::ValueArray<kvs::Real32> coords ( m_numParticles * 3 );
  kvs::Real32* pcoords = coords.pointer(); //point top of the array initially

  kvs::ValueArray<kvs::Real32> normals( m_numParticles * 3 );
  kvs::Real32* pnormals = normals.pointer(); //point top of the array initially

  kvs::ValueArray<kvs::UInt8> colors  ( m_numParticles * 3 );
  kvs::UInt8* pcolors = colors.pointer(); //point top of the array initially

  // Read data
  char       buf[ SPBR_BUF_SIZE ] ;
  char       dummy [64] ;

  // initialize point counter 
  unsigned int point_counter = 0 ;

  // Read commands and point data
  while ( reader.getline( buf, SPBR_BUF_SIZE - 1 , '\n' ) ) {

#if defined DEBUG
    std::cout << "READ: " << buf << std::endl;
#endif

    // Blank line
    if( !strlen (buf) ) { 
      continue ; // skip a blank line
    } else 
    // Commmand line (#/...) or comment line (#...)
    if( buf[0] == '#' ) {

      // Command line
      if(buf[1] == '/') { 

        //----- Origin -----
        if ( !strncmp( buf, ORIGIN_COMMAND, strlen(ORIGIN_COMMAND) ) ) { 
          double Ox, Oy, Oz ;
          sscanf ( buf, "%s %lg %lg %lg", dummy, &Ox, &Oy, &Oz );
          setBodyCoordOrigin( kvs::Vector3d( Ox, Oy, Oz ) );
          m_flagCoordTransformed = true ;
        }  else
        //----- BaseVector -----
        if ( !strncmp( buf, BASE_VECTOR_COMMAND, strlen(BASE_VECTOR_COMMAND) ) ) { 
          double e1x, e1y, e1z, e2x, e2y, e2z ;
          sscanf ( buf, "%s %lg %lg %lg %lg %lg %lg", dummy, 
                         &e1x, &e1y, &e1z, &e2x, &e2y, &e2z );
          setBodyCoordBaseVector( kvs::Vector3d( e1x, e1y, e1z ), 
                                  kvs::Vector3d( e2x, e2y, e2z )  );
          m_flagCoordTransformed = true ;
        }  else
        //----- ColorRGBByte (renamed from ColorByteRGB) ----- 
        // NOTE: The ColorRGBByte block must be written before the ColorRGB block
        if ( !strncmp( buf, COLOR_BYTE_COMMAND, strlen(COLOR_BYTE_COMMAND) ) ) { 
	    unsigned int Rb, Gb, Bb;
            sscanf ( buf, "%s %u %u %u", dummy, &Rb, &Gb, &Bb );
            setColor( Rb, Gb, Bb ); // set m_Rb, m_Gb, m_Bb
        } else
        //----- ColorRGB -----
        if ( !strncmp( buf, COLOR_COMMAND, strlen(COLOR_COMMAND) ) ) { 
          double R, G, B;
          sscanf ( buf, "%s %lg %lg %lg", dummy, &R, &G, &B );
          setColor( R, G, B ); // set m_Rb, m_Gb, m_Bb
        }  else
        //----- ReverseNormals -----
        if ( !strncmp( buf, REVERSE_NORMALS_COMMAND, strlen(REVERSE_NORMALS_COMMAND) ) ) { 
	  int flag ;
          sscanf ( buf, "%s %d", dummy, &flag );
          setFlagReverseNormals ( flag ) ;
        } else
        //----------------------- Unknown command
        {
	  // Do Nothing         
	}

      } // if(buf[1] == '/')  
      else {
        // Comment line ("#..." do nothing)
      }

    } // if( buf[0] == '#' ) 

    // Point data 
    else {
      // increment point counter
      point_counter++;  

      // Read too many points?
      if ( point_counter > m_numParticles ) {
	std::cout << "**   Reading point data is forcibly terminated." << std::endl;
	std::cout << "     ";
	std::cout << (point_counter -1) << " points are read." << std::endl;  
	break; // get out of the while loop to read lines
      }

      // Scan position, normal vector, and color from the read line
      double x  = 0.0, y  = 0.0, z  = 0.0 ;
      double nx = 0.0, ny = 0.0, nz = 0.0 ;
      unsigned int  Rb = m_Rb, Gb = m_Gb, Bb = m_Bb ;
      int num_buf_words = sscanf ( buf, "%lg %lg %lg %lg %lg %lg %u %u %u", &x, &y, &z, &nx, &ny, &nz, &Rb, &Gb, &Bb );

#if defined DEBUG_COLOR
      std::cout << "### num_buf_words = " << num_buf_words << std::endl;
#endif

      // Post processing of sscanf
      //...A: Tune the normal vector data
      if ( num_buf_words == 3 ) { // 3-word line (XYZ format)
        // Shading is off, since no normal vector is given
        nx = 0.0;   ny = 0.0;  nz = 0.0; 
        setFlagUseNormals ( false ) ; 
      }
      if( isNormalsReversed() ) { 
        // Invert orientation
        nx *= -1.0;   ny *= -1.0;  nz *= -1.0; 
      }
        // normalization
      normalizeVector ( &nx, &ny, &nz ); 

      //...B: Tune the color data 
      //      Special treatment for 
      //      (1) XYZ format, (2) XYZNxNyNz format, (3) Forced color mode
      if ( num_buf_words <= 6 || isForcedColor () ) { 
         Rb = m_Rb, Gb = m_Gb, Bb = m_Bb ;
      }

      //...C: Coordinate transformation if required
      if( m_flagCoordTransformed == true ) {
        doCoordTransformation( &x, &y, &z,  &nx, &ny, &nz );
      }

      // Store the read data 
      *(pcoords++) = x;
      *(pcoords++) = y;
      *(pcoords++) = z;

      *(pnormals++) = nx;
      *(pnormals++) = ny;
      *(pnormals++) = nz;

      *(pcolors++) = Rb;
      *(pcolors++) = Gb;
      *(pcolors++) = Bb;

    } // if point-data line

  } // while loop to read each line (readPointData())


  // Error recovery for the case that 
  //  value of #/NumParticles command is larger than 
  //  the real number of particles
  if ( point_counter < m_numParticles ) {
    m_numParticles = point_counter;
  }
  std::cout << "** Real number of points: " << m_numParticles << std::endl;


  // Set data to PointObject
  SuperClass::setCoords( coords );
  SuperClass::setNormals( normals );
  SuperClass::setColors ( colors  );
  SuperClass::setSize( m_pointSize );
  SuperClass::updateMinMaxCoords ();

  // Close files
  //.. input SPBR file
  reader.close ();

  // Message
  std::cout << "** PointObject is ready." << std::endl;
  std::cout << "** Point-set range: \n  " ;
  std::cout << BOUNDING_BOX_COMMAND << "  ";
  std::cout << minCoord() << "  ";
  std::cout << maxCoord() << std::endl;

} // readPointData()


//-----
void 
SPBR::setCameraPosition ( double Ex, double Ey, double Ez )
{
  m_cameraPosition.set( Ex, Ey, Ez );
  std::cout << "** Camera position : (" << Ex << "," << Ey << "," << Ez ;
  std::cout << ")" << std::endl;
}


//-----
void 
SPBR::setLookAt ( double LAx, double LAy, double LAz )
{
  m_lookAt.set( LAx, LAy, LAz );
  std::cout << "** Look-at position: (" << LAx << "," << LAy << "," << LAz;
  std::cout << ")" << std::endl;
}


//-----
void 
SPBR::setViewAngle ( double angle_deg )
{
  m_viewAngle = angle_deg ;
  std::cout << "** View-angle      : " << m_viewAngle << " [deg]" << std::endl;
}


//----- 
void 
SPBR::setFlagCameraFar ( int flag ) 
{
  if( flag ) { 
    m_flagCameraFar = true ;  
    setCameraPosition ( 0.0, 0.0, CAMERA_FAR_DISTANCE )  ;  
    setViewAngle ( CAMERA_FAR_VIEW_ANGLE );
    std::cout << "** Camera distance is made very far." << std::endl;
  }
  else       { 
    m_flagCameraFar = false ;  
  }

}


//-----
void 
SPBR::setCameraZoom ( double f_zoom ) 
{
  double d0 = DEFAULT_CAMERA_DISTANCE; 
  double d  = d0 / fabs(f_zoom) ;

  setCameraPosition ( 0.0, 0.0, d )  ;  
  std::cout << "** Camera distance is reset to " << d << std::endl;
}


//-----
void 
SPBR::setColor ( double R, double G, double B )
{
  this->setColor( (unsigned int)(255.0 * R), 
                  (unsigned int)(255.0 * G), 
                  (unsigned int)(255.0 * B) ) ;
}

//-----
void 
SPBR::setColor ( unsigned int Rb, unsigned int Gb, unsigned int Bb )
{
  setFlagForcedColor ( true ); //FORCED_COLOR

  m_Rb = Rb ; if ( m_Rb > 255 ) { m_Rb = 255 ;}
  m_Gb = Gb ; if ( m_Gb > 255 ) { m_Gb = 255 ;}
  m_Bb = Bb ; if ( m_Bb > 255 ) { m_Bb = 255 ;}

#if defined DEBUG_COLOR
  std::cout << "** Color (byte) is set to (" ;
  std::cout << m_Rb << " " << m_Gb << " " << m_Bb;
  std::cout << ")."  << std::endl;
#endif

}

//-----
void 
SPBR::setFlagUseNormals ( int flag ) 
{
  if( flag ) { 
    m_flagUseNormals = true;  
    std::cout << "** Use normal vectors for shading." ;
    std::cout << std::endl;
  }
  else       { 
    m_flagUseNormals = false; 
    std::cout << "** Do not use normal vectors for shading." ;
    std::cout << std::endl;
  }

}

//-----
void 
SPBR::setFlagFPS ( int flag ) 
{
  if( flag ) { 
    m_flagFPS = true;  
    std::cout << "** Display FPS on screen." ;
    std::cout << std::endl;
  }
  else       { 
    m_flagFPS = false; 
    std::cout << "** Do not display FPS on screen." ;
    std::cout << std::endl;
  }

}


//-----
void 
SPBR::setFlagLOD ( int flag ) 
{
  if( flag ) { 
    m_flagLOD = true;  
    std::cout << "** LOD is ON" << std::endl;
  }
  else       { 
    m_flagLOD = false ;  
    std::cout << "** LOD is OFF" << std::endl;
  }

}


//-----
void 
SPBR::setFlagShuffle ( int flag ) 
{
  if( flag ) { 
    m_flagShuffle = true;  
    std::cout << "** Particle shuffling is ON" << std::endl;
  }
  else       { 
    m_flagShuffle = false ;  
    std::cout << "** Particle shuffling is OFF" << std::endl;
  }

}


//----- //TANAKA
void 
SPBR::setFlagForcedShuffle ( int flag ) 
{
  if( flag ) { 
    m_flagForcedShuffle = true;  
    std::cout << "** Forced particle shuffling is ON" << std::endl;
  }
  else       { 
    m_flagForcedShuffle = false ;  
    std::cout << "** Forced particle shuffling is OFF" << std::endl;
  }

}


//-----
void 
SPBR::setPointSize ( unsigned int size ) 
{ 
  m_pointSize = size ;
  if ( m_pointSize < 1 ) {  m_pointSize = 1 ; }

  std::cout << "** Point size is set to \"" ;
  std::cout << m_pointSize << "\"." << std::endl;
  std::cout << "   (Used in OPBR and ignored in SPBR.)" <<  std::endl;
}


//-----
void 
SPBR::setRepeatLevel  ( unsigned int level ) 
{ 
  m_repeatLevel = level; 
  if ( m_repeatLevel < 1 ) {  m_repeatLevel = 1 ; }

  std::cout << "** Repeat level is set to \"" ;
  std::cout << m_repeatLevel << "\"." <<  std::endl;

}

//-----
void 
SPBR::setWireframeBox   ( double xmin, double ymin, double zmin, 
                              double xmax, double ymax, double zmax  )
{
  m_minWBPosition.set( xmin, ymin, zmin );
  m_maxWBPosition.set( xmax, ymax, zmax );
  std::cout << "** Wireframe box is set to: " ;
  std::cout << m_minWBPosition << "  " << m_maxWBPosition << std::endl;
  std::cout << "   (z=zmin: red, z=zmax: blue, z-parallel: green)" << std::endl; 
}




//-----
void 
SPBR::normalizeVector( double* x_p, double* y_p, double* z_p  )
{
  // x, y, z, and norm
  double x = *x_p; 
  double y = *y_p; 
  double z = *z_p; 
  double norm = sqrt( x*x + y*y + z*z );

  // normalization
  if( norm > 0 ) {
    *x_p /= norm;  *y_p /= norm ;  *z_p /= norm ;  
  } 

}

//-----
void 
SPBR::setBodyCoordOrigin ( const kvs::Vector3d& origin ) 
{
  m_O = origin ;
} 

//-----
void   
SPBR::setBodyCoordBaseVector ( const kvs::Vector3d& e1, 
                                     const kvs::Vector3d& e2 )
{
  // e1, e1
  m_e1 = e1       ; m_e2 = e2;
  m_e1.normalize(); m_e2.normalize();

  m_e3 = m_e1.cross (m_e2) ;
  m_e3.normalize();
} 


//-----
kvs::Vector3d 
SPBR::doCoordTransformation( const kvs::Vector3d& P )
{
  // calc 
  kvs::Vector3d PP = (P.x() * m_e1) + (P.y() * m_e2) + (P.z() * m_e3) + m_O ;

  // end
  return PP ;
}  

//-----
void  
SPBR::doCoordTransformation( double *x, double* y, double* z,  
                                  double *nx, double* ny, double* nz )
{
  //Original body-coord orgin, position, and normal vectors
  kvs::Vector3d O  ( 0.0, 0.0, 0.0 );
  kvs::Vector3d P  ( *x , *y , *z  ); 
  kvs::Vector3d N  ( *nx, *ny, *nz ); 

  //Transformed body-coord orgin, position, and normal vectors
  kvs::Vector3d OO = doCoordTransformation ( O ) ; 
  kvs::Vector3d PP = doCoordTransformation ( P ) ; 
  kvs::Vector3d NN = doCoordTransformation ( N ) ; 
  NN -= OO ; 

  //Results
  // ... Position
  *x = PP.x(); 
  *y = PP.y(); 
  *z = PP.z(); 

  // ... Normal vectors
  *nx = NN.x(); 
  *ny = NN.y(); 
  *nz = NN.z(); 
}  


//-----
void 
SPBR::setObjZXRot ( double zrot_angle_deg, double xrot_angle_deg ) 
{
  // set angle
  m_objZXRotAngle[0] = zrot_angle_deg ; 
  m_objZXRotAngle[1] = xrot_angle_deg ; 

  // set flag
  m_flagObjZXRot = true ;

  // Message
  std::cout << "** RotateZ(" <<  zrot_angle_deg << " deg) ==> ";
  std::cout << "RotateX(" <<  xrot_angle_deg << " deg)" << std::endl;
}


//-----
void   
SPBR::setBoundingBox ( const kvs::Vector3f& minPoint, 
                             const kvs::Vector3f& maxPoint )   

{
  m_flagForcedBoundingBox = true ;
  m_forcedMinPoint = minPoint ;
  m_forcedMaxPoint = maxPoint ;  

  // Message 
  std::cout << "** Forced bounding box is set to: ";
  std::cout << "(" <<   m_forcedMinPoint << ") - ";
  std::cout << "(" <<   m_forcedMaxPoint << ")" << std::endl;
}  


//-----
void 
SPBR::setMinMaxCoords( void )
{
  if( isForcedBoundingBox() ) { 
    kvs::Vector3f min = forcedMinPoint() ;
    kvs::Vector3f max = forcedMaxPoint() ;

    kvs::PointObject::setMinMaxObjectCoords   ( min, max );
    kvs::PointObject::setMinMaxExternalCoords ( min, max );
  }
}

//-----
void 
SPBR::setFlagParticleZoom ( int flag ) 
{
  // Set flag  m_flagParticleZoom
  if( flag ) { 
    m_flagParticleZoom = true;  
    std::cout << "** Particle zoom is ON." ;
    std::cout << std::endl;
  }
  else       { 
    m_flagParticleZoom = false; 
    std::cout << "** Particle zoom is OFF." ;
    std::cout << std::endl;
  }

}


//-----
void 
SPBR::setLambertShading ( double kd, double ka ) 
{ 
  m_flagLambertShading = true ;
  m_kd = kd ; m_ka = ka;

  if ( m_kd < 0.0 ) { m_kd = 0.0 ;}
  if ( m_kd > 1.0 ) { m_kd = 1.0 ;}

  if ( m_ka < 0.0 ) { m_ka = 0.0 ;}
  if ( m_ka > 1.0 ) { m_ka = 1.0 ;}
}


//-----
void 
SPBR::setPhongShading( double kd, double ka, double ks, int shininess )  

{ 
  m_flagLambertShading = false ;
  m_kd = kd ; m_ka = ka;
  m_ks = ks ; m_shininess = shininess;

  if ( m_kd < 0.0 ) { m_kd = 0.0 ;}
  if ( m_kd > 1.0 ) { m_kd = 1.0 ;}

  if ( m_ka < 0.0 ) { m_ka = 0.0 ;}
  if ( m_ka > 1.0 ) { m_ka = 1.0 ;}

  if ( m_ks < 0.0 ) { m_ks = 0.0 ;}
  if ( m_ks > 1.0 ) { m_ks = 1.0 ;}

  if ( m_shininess < 1 ) { m_shininess = 1;}  

}


// end of spbr.cpp
