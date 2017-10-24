#include <math.h>   // smallpt, a Path Tracer by Kevin Beason, 2008
#include <stdlib.h> // Make : g++ -O3 -fopenmp smallpt4k.cpp -o smallpt4k
#include <stdio.h>  //        Remove "-fopenmp" for g++ version < 4.2
#include <thread>

#include "ThreadPool.h"
#include "Type.h"
#include "Utils.h"
#include "erandom.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
static const int SCENE_OBJECT_COUNT = 9;
static Sphere spheres[SCENE_OBJECT_COUNT];
////////////////////////////////////////////////////////////////////////////////////////////////////
Vec radiance(const Ray &r, int depth, const int watchDog, unsigned short *Xi)
{
    if( watchDog <= 0 )
        return Vec();
        
    double t;                               // distance to intersection
    int id = 0;                               // id of intersected object
    if( !intersect(r, t, id, spheres, SCENE_OBJECT_COUNT ) )
        return Vec(); // if miss, return black
    
    const Sphere &obj = spheres[id];        // the hit object
    Vec x=r.o+r.d*t, n=(x-obj.p).norm(), nl=n.dot(r.d)<0?n:n*-1, f=obj.c;
    double p = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z; // max refl
    if( ++depth > 5 )
    {
        if( erand48( Xi ) < p )
            f=f*(1/p);
        else
            return obj.e; //R.R.
    }

    if( obj.refl == DIFF ) // Ideal DIFFUSE reflection
    {                  
        double r1=2*M_PI*erand48(Xi), r2=erand48(Xi), r2s=sqrt(r2);
        Vec w=nl, u=((fabs(w.x)>.1?Vec(0,1):Vec(1))%w).norm(), v=w%u;
        Vec d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1-r2)).norm();
        return obj.e + f.mult(radiance(Ray(x,d),depth, watchDog - 1, Xi));
    } 
    else if( obj.refl == SPEC )            // Ideal SPECULAR reflection
        return obj.e + f.mult(radiance(Ray(x,r.d-n*2*n.dot(r.d)),depth, watchDog - 1, Xi ) );
  
    // Ideal dielectric REFRACTION
    Ray reflRay(x, r.d-n*2*n.dot(r.d));     
    bool into = n.dot(nl)>0;                // Ray from outside going in?
    double nc=1, nt=1.5, nnt=into?nc/nt:nt/nc, ddn=r.d.dot(nl), cos2t;
    if ((cos2t=1-nnt*nnt*(1-ddn*ddn))<0)    // Total internal reflection
        return obj.e + f.mult(radiance(reflRay,depth, watchDog- 1, Xi));
  
    Vec tdir = (r.d*nnt - n*((into?1:-1)*(ddn*nnt+sqrt(cos2t)))).norm();
    double a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn:tdir.dot(n));
    double Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re,P=.25+.5*Re,RP=Re/P,TP=Tr/(1-P);
    return obj.e + f.mult(depth>2 ? (erand48(Xi)<P ?   // Russian roulette
    
    radiance(reflRay,depth, watchDog - 1, Xi ) * RP:radiance(Ray(x,tdir),depth, watchDog - 1, Xi ) * TP ) :
    radiance(reflRay,depth, watchDog - 1, Xi ) * Re + radiance(Ray(x,tdir),depth, watchDog - 1, Xi ) * Tr );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
static void InitScene()
{
    spheres[0]=Sphere(1e5, Vec( 1e5+1,40.8,81.6), Vec(),Vec(.75,.25,.25),DIFF);//Left
    spheres[1]=Sphere(1e5, Vec(-1e5+99,40.8,81.6),Vec(),Vec(.25,.25,.75),DIFF);//Rght
    spheres[2]=Sphere(1e5, Vec(50,40.8, 1e5),     Vec(),Vec(.75,.75,.75),DIFF);//Back
    spheres[3]=Sphere(1e5, Vec(50,40.8,-1e5+170), Vec(),Vec(),           DIFF);//Frnt
    spheres[4]=Sphere(1e5, Vec(50, 1e5, 81.6),    Vec(),Vec(.75,.75,.75),DIFF);//Botm
    spheres[5]=Sphere(1e5, Vec(50,-1e5+81.6,81.6),Vec(),Vec(.75,.75,.75),DIFF);//Top
    spheres[6]=Sphere(16.5,Vec(27,16.5,47),       Vec(),Vec(1,1,1)*.999, SPEC);//Mirr
    spheres[7]=Sphere(16.5,Vec(73,16.5,78),       Vec(),Vec(1,1,1)*.999, REFR);//Glas
    spheres[8]=Sphere(600, Vec(50,681.6-.27,81.6),Vec(12,12,12),  Vec(), DIFF);//Lite

}
////////////////////////////////////////////////////////////////////////////////////////////////////
static void ProcessPatch( const size_t threadID, const SThreadInfo& info )
{
    //printf( "Patch %d started...", info.pos );
    //printf( "Patch %d started by thread %d...\n", info.pos, (int)threadID );
    
    const Ray& camera = *info.pCamera;
    const Vec& cx = *info.pCx;
    const Vec& cy = *info.pCy;
    
    for( int y = 0; y < info.sizeH; ++y )
    {
        const int worldY = info.startY + y;
        const int y3 = worldY * worldY * worldY;
        const unsigned short y3us = static_cast< unsigned short >( y3 );    
        unsigned short Xi[3]={ 0, 0, y3us };
        
        for( int x = 0; x < info.sizeW; ++x )
        {
            const int worldX = info.startX + x;
            const int offset = ( info.imageH - worldY - 1 ) * info.imageW + worldX;
            
            for( int sy = 0; sy < 2; ++sy )         // 2x2 subpixel rows
            {
                Vec r;
                for( int sx = 0; sx < 2; ++sx )       // 2x2 subpixel cols
                {
                    for( int s = 0; s < info.samplesCount; ++s )
                    {
                        double r1 = 2.0 * erand48( Xi );
                        double r2 = 2.0 * erand48( Xi );
                        double dx = ( r1 < 1.0 ) ? sqrt( r1 ) - 1.0 : 1.0 - sqrt( 2.0 - r1 );
                        double dy = ( r2 < 1.0 ) ? sqrt( r2 ) - 1.0 : 1.0 - sqrt( 2.0 - r2 );
                        Vec d = cx * ( ( ( sx + 0.5 + dx ) / 2.0 + worldX ) / info.imageW - 0.5 ) + 
                                cy * ( ( ( sy + 0.5 + dy ) / 2.0 + worldY ) / info.imageH - 0.5 ) + camera.d;
                        r = r + radiance( Ray( camera.o + d * 140.0, d.norm() ), 0.0, info.watchDog, Xi ) * ( 1.0 / info.samplesCount );
                    }
                    info.pColor[offset] = info.pColor[offset] + Vec( clamp( r.x ), clamp( r.y ), clamp( r.z ) ) * 0.25;
                }
            }
        }
    }
    
    //printf( "Patch %d ended\n", info.pos );
}
////////////////////////////////////////////////////////////////////////////////////////////////////
static void ProcessImage( Vec *pColor,
                          const int imageW, const int imageH, const int samplesCount, const int watchDog,
                          const Ray& camera, const Vec& cx, const Vec& cy  )
{
    // Divide image into several pathces
    const int PATCH_SIZE = 8;
    const int PATCH_SIZE_SRQ = PATCH_SIZE * PATCH_SIZE;
    const int patchCount = imageW * imageH / PATCH_SIZE_SRQ;
    
    int pos = 0;
    for( int y = 0; y < imageH; y += PATCH_SIZE )
        for( int x = 0; x < imageW; x += PATCH_SIZE )
        {
            SThreadInfo info;
            
            info.pCamera = &camera;
            info.pCx = &cx;
            info.pCy = &cy;
            info.pColor = pColor;
            info.startX = x;
            info.startY = y;
            info.sizeW = PATCH_SIZE;
            info.sizeH = PATCH_SIZE;
            info.imageW = imageW;
            info.imageH = imageH;
            info.samplesCount = samplesCount;
            info.watchDog = watchDog;
            info.pos = pos;
            
            
            const int fakeID = 0;
            ProcessPatch( fakeID, info );
                          
            ++pos;
            const float progress = 100.0f * static_cast< float >( pos ) / static_cast< float >( patchCount );
            fprintf( stderr,"\rRendering (%d spp) %5.2f%%", samplesCount, progress );
        }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
static void ProcessImageMultithread( Vec *pColor,
                                     const int imageW, const int imageH, const int samplesCount, const int watchDog,
                                     const Ray& camera, const Vec& cx, const Vec& cy,
                                     CThreadPool& pool  )
{
    // Divide image into several pathces
    const int PATCH_SIZE = 8;
    const int PATCH_SIZE_SRQ = PATCH_SIZE * PATCH_SIZE;
    const int patchCount = imageW * imageH / PATCH_SIZE_SRQ;
    assert( 0 == ( patchCount % PATCH_SIZE_SRQ ) );
    
    //std::vector< SThreadInfo > taskPool;
    //taskPool.reserve( patchCount ); 
    
    
    int pos = 0;
    for( int y = 0; y < imageH; y += PATCH_SIZE )
        for( int x = 0; x < imageW; x += PATCH_SIZE )
        {
            SThreadInfo info;
            info.pCamera = &camera;
            info.pCx = &cx;
            info.pCy = &cy;
            info.pColor = pColor;
            
            info.startX = x;
            info.startY = y;
            info.sizeW = PATCH_SIZE;
            info.sizeH = PATCH_SIZE;
            info.imageW = imageW;
            info.imageH = imageH;
            info.samplesCount = samplesCount;
            info.watchDog = watchDog;
            info.pos = pos;
            
            
            pool.Add( ProcessPatch, info );
            
            //taskPool.push_back( info );
            
            //taskPool.Add( ProcessPatch, info );
            
            
            //ProcessPatch( info );
            ++pos;
            
            //const float progress = 100.0f * static_cast< float >( pos ) / static_cast< float >( patchCount );
            //fprintf( stderr,"\rRendering (%d spp) %5.2f%%", samplesCount, progress );
        }        
    pool.WaitAllDone();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
static void ProcessScene( Vec *pColor,
                          const int sizeW, const int sizeH, const int samplesCount, const int watchDog,
                          const Ray& camera, const Vec& cx, const Vec& cy  )
{
    for( int y = 0; y < sizeH; ++y )
    {
        const float progress = 100.0f * static_cast< float >( y ) / static_cast< float >( sizeH - 1 );
        fprintf( stderr,"\rRendering (%d spp) %5.2f%%", samplesCount, progress );
    
        const int y3 = y * y * y;
        const unsigned short y3us = static_cast< unsigned short >( y3 );
    
        unsigned short Xi[3]={ 0,0, y3us };
        for( int x = 0; x < sizeW; ++x )
        {
            const int offset = ( sizeH - y - 1) * sizeW + x;
            for( int sy = 0; sy<2; ++sy )         // 2x2 subpixel rows
            {
                Vec r;
                for( int sx = 0; sx < 2; ++sx )       // 2x2 subpixel cols
                {
                    for( int s = 0; s < samplesCount; ++s )
                    {
                        double r1 = 2.0 * erand48( Xi );
                        double r2 = 2.0 * erand48( Xi );
                        double dx = ( r1 < 1.0 ) ? sqrt( r1 ) - 1.0 : 1.0 - sqrt( 2.0 - r1 );
                        double dy = ( r2 < 1.0 ) ? sqrt( r2 ) - 1.0 : 1.0 - sqrt( 2.0 - r2 );
                        Vec d = cx * ( ( ( sx + 0.5 + dx ) / 2.0 + x ) / sizeW - 0.5 ) + 
                                cy * ( ( ( sy + 0.5 + dy ) / 2.0 + y ) / sizeH - 0.5 ) + camera.d;
                        r = r + radiance( Ray( camera.o + d * 140.0, d.norm() ), 0.0, watchDog, Xi ) * ( 1.0 / samplesCount );
                    }
                    
                    // Camera rays are pushed ^^^^^ forward to start in interior
                    pColor[offset] = pColor[offset] + Vec( clamp( r.x ), clamp( r.y ), clamp( r.z ) ) * 0.25;
                }
            }
        }
    }

}
////////////////////////////////////////////////////////////////////////////////////////////////////
static void SaveImage( const int imageW, const int imageH, const Vec *pColor )
{
    const int imageSize = imageW * imageH;
    FILE *f = fopen("image.ppm", "w");         // Write image to PPM file.
    fprintf( f, "P3\n%d %d\n%d\n", imageW, imageH, 255 );
    for( int i = 0; i < imageSize; ++i )
        fprintf( f, "%d %d %d ", toInt( pColor[i].x ), toInt( pColor[i].y ), toInt( pColor[i].z ) );
    fclose(f);
}    
////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    const int w = 128;
    const int h = 128;
    const int samps = 128;
    const int watchDog = 256;
    
    const int coreNumber = std::thread::hardware_concurrency();
    fprintf( stderr,"\nCore count: %d\n", coreNumber );
    //printf( "/nCore count: %d/n", coreNumber );
    
    InitScene();
    
    const Ray cam( Vec( 50.0, 52.0, 295.6 ), Vec( 0.0, -0.042612, -1.0 ).norm()); // cam pos, dir
    const Vec cx = Vec( w * 0.5135 / h );
    const Vec cy = ( cx % cam.d ).norm() * 0.5135;
    Vec *pColor = (Vec*)malloc(sizeof(Vec) * w * h );
    
    CThreadPool threadPool( coreNumber );
    
    fprintf( stderr,"Processing started\n" );;
    const uint64_t timeA = GetRealTimeNano();
    //ProcessScene( pColor, w, h, samps, cam, cx, cy ); // Old version
    //ProcessImage( pColor, w, h, samps, watchDog, cam, cx, cy );   // One thread version
    ProcessImageMultithread( pColor, w, h, samps, watchDog, cam, cx, cy, threadPool );   // Multithread version
    
    const uint64_t timeB = GetRealTimeNano();
    SaveImage( w, h, pColor );
    
    // Final time calculation
    const uint64_t timeC = GetRealTimeNano();
    const uint64_t timeProcess = ( timeB - timeA ) / 1000000;
    const uint64_t timeSave = ( timeC - timeB ) / 1000000;
    const int timeProcessMS = static_cast< int >( timeProcess ); 
    const int timeSaveMS = static_cast< int >( timeSave );
    
    // Log
    fprintf( stderr, "\nCompleted\n" );
    fprintf( stderr, "Process time: %d ms\n", timeProcessMS );
    fprintf( stderr, "Save time: %d ms\n", timeSaveMS );
    //printf( "Process time: %d ms /n", timeProcessMS );
    //printf( "Save time: %d ms /n", timeSaveMS );
    
    //threadPool.StopAll();
    
    return 0;
}
