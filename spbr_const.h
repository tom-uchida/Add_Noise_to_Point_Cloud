////////////////////////
///// spbr_const.h /////
////////////////////////

#if !defined  SPBR_CONST_HH
#define       SPBR_CONST_HH

const double DEFAULT_CAMERA_DISTANCE = 12.0;
const double CAMERA_FAR_SCALE      = 84.0                   ; 
const double CAMERA_FAR_DISTANCE   = DEFAULT_CAMERA_DISTANCE * CAMERA_FAR_SCALE; // 1008
const double CAMERA_FAR_VIEW_ANGLE = 45.0 / CAMERA_FAR_SCALE; // 0.54

const char   SPBR_FILE_EXTENSION[] = "spbr";

#endif // end of spbr_const.h
