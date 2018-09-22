#ifndef _xyzAsciiReader_H__
#define _xyzAsciiReader_H__

#include <kvs/PolygonObject>
#include <kvs/Module> 
#include <vector>

class xyzAsciiReader: public kvs::PolygonObject
{
  kvsModuleSuperClass( kvs::PolygonObject ); 
  
 public:
  xyzAsciiReader(void);
  xyzAsciiReader( char* filname );
  std::vector<float> featureData( void ) { return m_ft; }

 private:
  void execRead( char* filename);
  int breakWord( char* buf, std::string *str );

 private:
  char* m_filename;
  int m_numVert;
  std::vector<float> m_ft;

};

#endif
