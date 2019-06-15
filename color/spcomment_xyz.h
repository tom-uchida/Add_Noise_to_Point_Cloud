#ifndef _spcomment_xyz_H_
#define _spcomment_xyz_H_

//---- BinaryData for xyz format
const char XYZ_BINARY [] = "#/XYZ_BinaryData" ;

//---- Number of Points
const char XYZ_NUM_PARTICLES [] = "#/NumParticles";

// #/EndHeader 
const char XYZ_END_HEADER [] = "#/EndHeader" ;  

//--- Data Type
const char XYZ_DATA_TYPE [] = "#/XYZDataType" ;  
//--- Datatype : Vertex only
const char XYZ [] = "XYZ" ;  
//--- Datatype : Vertex + Normal 
const char XYZ_N [] = "XYZNormal" ;  
//--- Datatype : Vertex + Normal + Color
const char XYZ_NC [] = "XYZNormalColor" ;  
//--- Datatype : Vertex + Normal + Color + Feature
const char XYZ_NCF [] = "XYZNormalColorFeature" ;

#endif
