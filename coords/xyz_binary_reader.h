#ifndef _xyzBinaryReader_H__
#define _xyzBinaryReader_H__

#include <kvs/PolygonObject>
#include <kvs/Module> 
#include <vector>
#include <fstream>

class xyzBinaryReader: public kvs::PolygonObject
{
  kvsModuleSuperClass( kvs::PolygonObject ); 
  
 public:
  xyzBinaryReader(void);
  xyzBinaryReader( char* filname );
  std::vector<float> featureData( void ) { return m_ft; }

 private:
  void execReadHeader( void );
  void execReadData( void );
  int breakWord( char* buf, std::string *str );

 private:
  char* m_filename;
  int m_numVert;
  int m_numData;
  std::vector<float> m_ft;
  std::ifstream m_fin;

};

#endif
