#ifndef _plyRead_H__
#define _plyRead_H__

#include <kvs/PolygonObject>
#include <kvs/Module> 

class plyRead: public kvs::PolygonObject {
    kvsModuleSuperClass( kvs::PolygonObject );   

public:
    plyRead(void);
    plyRead( char* filname, bool &hface );

private:
    void checkHasFace(void);
    void execReadAscii( char* filename);
    void execReadBinary( char* filename);

private:
    char* m_filename;
    bool hasFace;
    bool hasColor;		/* For Vertex */
    bool hasNormal;		/* For Vertex */
    bool hasAlpha;		/* For Vertex */
    bool isAscii;
    int numVert;
    int numFace;
};
#endif
