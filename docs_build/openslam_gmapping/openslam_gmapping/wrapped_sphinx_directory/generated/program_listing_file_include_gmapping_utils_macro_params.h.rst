
.. _program_listing_file_include_gmapping_utils_macro_params.h:

Program Listing for File macro_params.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_utils_macro_params.h>` (``include/gmapping/utils/macro_params.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MACRO_PARAMS_H
   #define MACRO_PARAMS_H
   
   #define PARAM_SET_GET(type, name, qualifier, setqualifier, getqualifier)\
   qualifier: type m_##name;\
   getqualifier: inline type get##name() const {return m_##name;}\
   setqualifier: inline void set##name(type name) {m_##name=name;}
   
   #define PARAM_SET(type, name, qualifier, setqualifier)\
   qualifier: type m_##name;\
   setqualifier: inline void set##name(type name) {m_##name=name;}
   
   #define PARAM_GET(type, name, qualifier, getqualifier)\
   qualifier: type m_##name;\
   getqualifier: inline type get##name() const {return m_##name;}
   
   #define MEMBER_PARAM_SET_GET(member, type, name, qualifier, setqualifier, getqualifier)\
   getqualifier: inline type get##name() const {return member.get##name();}\
   setqualifier: inline void set##name(type name) { member.set##name(name);}
   
   #define MEMBER_PARAM_SET(member, type, name, qualifier, setqualifier, getqualifier)\
   setqualifier: inline void set##name(type name) { member.set##name(name);}
   
   #define MEMBER_PARAM_GET(member, type, name, qualifier, setqualifier, getqualifier)\
   getqualifier: inline type get##name() const {return member.get##name();}
   
   #define STRUCT_PARAM_SET_GET(member, type, name, qualifier, setqualifier, getqualifier)\
   getqualifier: inline type get##name() const {return member.name;}\
   setqualifier: inline void set##name(type name) {member.name=name;}
   
   #define STRUCT_PARAM_SET(member, type, name, qualifier, setqualifier, getqualifier)\
   setqualifier: inline void set##name(type name) {member.name=name;}
   
   #define STRUCT_PARAM_GET(member, type, name, qualifier, setqualifier, getqualifier)\
   getqualifier: inline type get##name() const {return member.name;}\
   
   #define convertStringArgument(var,val,buf) if (!strcmp(buf,#val)) var=val
   #endif
