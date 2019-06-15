#ifndef _ImportPointClouds_H__
#define _ImportPointClouds_H__

#include <kvs/Module>
#include <kvs/PolygonObject>

class ImportPointClouds: public kvs::PolygonObject {
	kvsModuleSuperClass( kvs::PolygonObject );

public:
	ImportPointClouds( void );
	ImportPointClouds( char* filename );

private:
	bool m_hasFace;
	void classification( char* filename );
	int	breakWord( char* buf, std::string *str );
	std::vector<float> m_ft;

public:
	bool isFase( void ) { return m_hasFace; }
	std::vector<float> featureData( void ) { return m_ft; }
};


#endif
