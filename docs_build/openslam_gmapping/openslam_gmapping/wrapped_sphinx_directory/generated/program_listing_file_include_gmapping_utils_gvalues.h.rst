
.. _program_listing_file_include_gmapping_utils_gvalues.h:

Program Listing for File gvalues.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_utils_gvalues.h>` (``include/gmapping/utils/gvalues.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef _GVALUES_H_
   #define _GVALUES_H_
   
   #define MAXDOUBLE 1e1000
   #ifdef LINUX
       #include <values.h>
   #endif
   #ifdef MACOSX
       #include <limits.h>
       #include <math.h>
       //#define isnan(x) (x==FP_NAN)
   #endif
   #ifdef _WIN32
     #include <limits>
     #ifndef __DRAND48_DEFINED__
        #define __DRAND48_DEFINED__
        inline double drand48() { return double(rand()) / RAND_MAX;}
     #endif
     #ifndef M_PI
       #define M_PI 3.1415926535897932384626433832795
     #endif
     #define round(d) (floor((d) + 0.5))
     typedef unsigned int uint;
     #define isnan(x) (_isnan(x))
   #endif
   
   #endif 
   
