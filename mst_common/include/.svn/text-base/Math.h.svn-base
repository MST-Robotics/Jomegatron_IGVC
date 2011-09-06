//----------------------------------------------------------------------
//  Math.h - Math header
//
// Description: Contains common math functions used by any module
//----------------------------------------------------------------------
#ifndef MATH_H
#define MATH_H

#include <list>
#include <math.h>

double deg_to_rad( const double degrees )
{
    return ( M_PI / 180.0 ) * degrees;
} /* deg_to_rad() */

double rad_to_deg( const double radians )
{
    return radians * ( 180.0 / M_PI );
} /* rad_to_deg() */

double limit( const double value, const double low, const double high )
{
    if( value >= high )
    {
        return high;
    }
    else if( value <= low )
    {
        return low;
    }
    return value;
} /* limit() */

double delta_limit( const double delta, const double old, const double new_val )
{
    if( (new_val-old) > delta )
    {
        return old + delta;
    }
    if( (old-new_val) > delta )
    {
        return new_val - delta;
    }
    if( ((new_val-old) <= delta) && ((old-new_val) <= delta) )
    {
        return new_val;
    }
} /* delta_limit() */

template < class _T >
_T average( std::list<_T>& x )
{
    _T average = 0;
    typename std::list<_T>::iterator it;
    
    for( it = x.begin(); it != x.end(); it++ )
    {
        average += *it;
    }
    average /= x.size();
    return average;
} /* average() */

//~ double average( std::list<double> x )
//~ {
    //~ double average = 0;
    //~ std::list<double>::iterator it;
    //~ 
    //~ for( it = x.begin(); it != x.end(); it++ )
    //~ {
        //~ average += *it;
    //~ }
    //~ average /= x.size();
    //~ return average;
//~ }

#endif
