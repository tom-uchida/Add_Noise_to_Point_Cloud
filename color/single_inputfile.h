//////////////////////////////
///// single_inputfile.h /////
/////   (Singleton)      /////
//////////////////////////////

#if !defined  INPUT_FILE_HH
#define       INPUT_FILE_HH

#include <cstring>

//---------------//
class SingleInputFile{
//---------------//
public:
  static SingleInputFile* GetInstance()
  {
    static SingleInputFile instance;  // only instance
    return &instance;
  } 

  void SetName( const char* filename ) ;

  void GetName( char name[] ) { std::strcpy( name, m_name ) ; }
  void GetNameBody ( char body[]) ;
  bool GetNameExt  ( char ext[] ) ; // RETURN: success / fail

  void GetBMPName( char name[] ) { std::strcpy( name, m_bmp_name ) ; }
  void GetPGMName( char name[] ) { std::strcpy( name, m_pgm_name ) ; }
  void GetPPMName( char name[] ) { std::strcpy( name, m_ppm_name ) ; }

 private:
  // private constructor 
  SingleInputFile()
  { 
     std::strcpy(m_name, ""); 
     m_name_length = 0 ;
    
     std::strcpy(m_bmp_name, ""); 
     std::strcpy(m_pgm_name, ""); 
     std::strcpy(m_ppm_name, ""); 
  }

  char m_name [256];
  int  m_name_length ;

  char m_bmp_name [256];
  char m_pgm_name [256];
  char m_ppm_name [256];

}; // SingleInputFile


//-----
inline 
void SingleInputFile::SetName( const char* filename ) 
{ 
  char body[252];

  std::strcpy( m_name, filename ); 
  m_name_length = strlen ( m_name ) ;

  GetNameBody ( body ) ;
  strcpy( m_bmp_name, body); 
  strcat( m_bmp_name, "."); 
  strcat( m_bmp_name, "bmp"); 

  strcpy( m_pgm_name, body); 
  strcat( m_pgm_name, "."); 
  strcat( m_pgm_name, "pgm"); 

  strcpy( m_ppm_name, body); 
  strcat( m_ppm_name, "."); 
  strcat( m_ppm_name, "ppm"); 

}



//-----
inline 
void SingleInputFile::GetNameBody ( char body[] )
{
  for ( int i = 0; i < m_name_length; i++ ) {
    if( m_name[i] == '.' ) { 
      body[i] = '\0';
      break; 
    }
    else {
      body[i] = m_name[i] ;
    }
    
  }

}


//-----
inline 
bool SingleInputFile::GetNameExt ( char body[] )
{
  // local variables
  char* p ;
  bool flag_found = false ;

  // Calc
  for ( p = m_name; p != NULL ; p++ ) {
    if( *p == '.' ) { 
      flag_found = true ;
      p++ ;  // go to the head of ext
      break; 
    }
  }

  // Set 
  if( flag_found ) { std::strcpy( body, p ); }

  // EndFn
  return flag_found ;

}


#endif
