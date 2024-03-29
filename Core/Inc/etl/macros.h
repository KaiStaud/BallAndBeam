///\file

/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2018 jwellbelove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#ifndef ETL_MACROS_INCLUDED
#define ETL_MACROS_INCLUDED

#define ETL_CONCAT2(X, Y)  X##Y
#define ETL_CONCAT(X, Y)   ETL_CONCAT2(X, Y)
#define ETL_STRINGIFY2(X)  #X
#define ETL_STRINGIFY(X)   ETL_STRINGIFY2(X)
#define ETL_STRING(X)      ETL_CONCAT(  , ETL_STRINGIFY(X))
#define ETL_WIDE_STRING(X) ETL_CONCAT( L, ETL_STRINGIFY(X))
#define ETL_U8_STRING(X)   ETL_CONCAT(u8, ETL_STRINGIFY(X))
#define ETL_U16_STRING(X)  ETL_CONCAT( u, ETL_STRINGIFY(X))
#define ETL_U32_STRING(X)  ETL_CONCAT( U, ETL_STRINGIFY(X))

#endif

