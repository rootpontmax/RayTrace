#pragma once

#include <cfloat>

////////////////////////////////////////////////////////////////////////////////////////////////////
enum Refl_t { DIFF, SPEC, REFR };
////////////////////////////////////////////////////////////////////////////////////////////////////
struct Vec
{
    double x;
    double y;
    double z;
    
    Vec(double x_= 0.0, double y_= 0.0, double z_= 0.0) : x( x_ ), y( y_ ), z( z_ ) {}
    Vec operator+(const Vec &b) const { return Vec(x+b.x,y+b.y,z+b.z); }
    Vec operator-(const Vec &b) const { return Vec(x-b.x,y-b.y,z-b.z); }
    Vec operator*(double b) const { return Vec(x*b,y*b,z*b); }
    Vec mult(const Vec &b) const { return Vec(x*b.x,y*b.y,z*b.z); }
    Vec& norm(){ return *this = *this * (1/sqrt(x*x+y*y+z*z)); }
    double dot(const Vec& b) const { return x*b.x+y*b.y+z*b.z; } // cross:
    Vec operator%( const Vec& b ) const {return Vec(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x);}
};
////////////////////////////////////////////////////////////////////////////////////////////////////
struct Ray
{
    Vec o;
    Vec d;
    
    Ray( Vec o_, Vec d_ ) : o( o_ ), d( d_ ) {}
};
////////////////////////////////////////////////////////////////////////////////////////////////////
struct Sphere
{
    double rad;       // radius
    Vec p, e, c;      // position, emission, color
    Refl_t refl;      // reflection type (DIFFuse, SPECular, REFRactive)
    Sphere(){}
    Sphere( double rad_, Vec p_, Vec e_, Vec c_, Refl_t refl_):
    rad(rad_), p(p_), e(e_), c(c_), refl(refl_) {}
    
    // returns distance, 0 if nohit
    double intersect(const Ray &r) const
    { 
        Vec op = p-r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        double t, eps=1e-4, b=op.dot(r.d), det=b*b-op.dot(op)+rad*rad;
        if( det < 0.0f )
            return 0;
        else
            det = sqrt( det );
            
    return ( t = b - det ) > eps ? t : ( ( t = b + det ) > eps ? t : 0 );
  }
};
////////////////////////////////////////////////////////////////////////////////////////////////////
inline double clamp(double x)
{
    return x < 0 ? 0 : x > 1.0 ? 1.0 : x;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
inline int toInt(double x)
{
    return int( pow( clamp( x ), 1.0 / 2.2 ) * 255 + 0.5 );
}
////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool intersect( const Ray &r, double &t, int &id, Sphere *pSphere, const int n )
{
    t = DBL_MAX; 
    for( int i = 0; i < n; ++i )
    {
        const double d = pSphere[i].intersect( r );
        if( d > 0.0 && d < t )
        {
            t = d;
            id = i;
        }
    }
        
    return ( t < DBL_MAX );
}
////////////////////////////////////////////////////////////////////////////////////////////////////

