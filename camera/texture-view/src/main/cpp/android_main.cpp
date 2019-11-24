/**
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Description
 *     Demonstrate NDKCamera's PREVIEW mode -- hooks camera directly into
 *     display.
 *     Control:
 *         Double-Tap on Android device's screen to toggle start/stop preview
 *     Tested:
 *         Google Pixel and Nexus 6 phones
 */
#include <jni.h>
#include <cstring>
#include <utils/native_debug.h>
#include "camera_manager.h"
#include "camera_engine.h"
#include <cmath>

/**
 * Application object:
 *   the top level camera application object, maintained by native code only
 */
CameraAppEngine *pEngineObj = nullptr;

/**
 * createCamera() Create application instance and NDK camera object
 * @param  width is the texture view window width
 * @param  height is the texture view window height
 * In this sample, it takes the simplest approach in that:
 * the android system display size is used to view full screen
 * preview camera images. Onboard Camera most likely to
 * support the onboard panel size on that device. Camera is most likely
 * to be oriented as landscape mode, the input width/height is in
 * landscape mode. TextureView on Java side handles rotation for
 * portrait mode.
 * @return application object instance ( not used in this sample )
 */
extern "C" JNIEXPORT jlong JNICALL
Java_com_sample_textureview_ViewActivity_createCamera(JNIEnv *env,
                                                      jobject instance,
                                                      jint width, jint height) {
  pEngineObj = new CameraAppEngine(env, instance, width, height);
  return reinterpret_cast<jlong>(pEngineObj);
}

/**
 * deleteCamera():
 *   releases native application object, which
 *   triggers native camera object be released
 */
extern "C" JNIEXPORT void JNICALL
Java_com_sample_textureview_ViewActivity_deleteCamera(JNIEnv *env,
                                                      jobject instance,
                                                      jlong ndkCameraObj) {
  if (!pEngineObj || !ndkCameraObj) {
    return;
  }
  CameraAppEngine *pApp = reinterpret_cast<CameraAppEngine *>(ndkCameraObj);
  ASSERT(pApp == pEngineObj, "NdkCamera Obj mismatch");

  delete pApp;

  // also reset the private global object
  pEngineObj = nullptr;
}

/**
 * getCameraCompatibleSize()
 * @returns minimium camera preview window size for the given
 * requested camera size in CreateCamera() function, with the same
 * ascpect ratio. essentially,
 *   1) Display device size decides NDKCamera object preview size
 *   2) Chosen NDKCamera preview size passed to TextView to
 *      reset textureView size
 *   3) textureView size is stretched when previewing image
 *      on display device
 */

int mask [3][3] = {1 ,2 ,1 ,
                   2 ,3 ,2 ,
                   1 ,2 ,1 };
unsigned char getPixel ( unsigned char * arr , int col , int row , int k,int height,int width ) {
    int sum = 0;
    int denom = 0;
    unsigned char pixel;
    for ( int j = -1; j <=1; j ++) {
        for ( int i = -1; i <=1; i ++) {
            if ((row + j) >= 0 && (row + j) < height && (col + i) >= 0 && (col + i) < width) {
                unsigned char color = arr [( row + j ) * 3 * width + ( col + i ) * 3 + k];
                sum += color * mask [ i +1][ j +1];
                denom += mask [ i +1][ j +1];
            }
        }
    }
    pixel =(unsigned char) ( (sum / denom) > 255)? 255:(sum / denom);
    pixel =(unsigned char) ( (sum / denom) < 0)? 0:(sum / denom);
    return pixel;
}
    void h_blur ( unsigned char * arr ,int height,int width ) {
      for ( int row =0; row < height; row ++) {
        for ( int col =0; col < width; col ++) {
          for (int k = 0; k < 3; k++) {
            if( (3 * row * width + 3 * col + k) <=  height*width){
              arr [ 3 * row * width + 3 * col + k] = getPixel ( arr , col , row , k, height, width ) ;
            }

          }
        }
      }
    }

    struct cannyAlgorithm
    {
        unsigned char * image;

        unsigned width;
        unsigned height;
        unsigned size;
        unsigned bpp;

        short * Gx;
        short * Gy;

        short * Direction;

        double * Magnitude;

        cannyAlgorithm( unsigned char * data, unsigned imgw, unsigned imgh, unsigned bitsperpixel )
        {
          width   = imgw;
          height  = imgh;

          size = width* height;

          bpp = bitsperpixel;

          Gx = new short[ size ];
          Gy = new short[ size ];

          Direction = new short[ size ];

          Magnitude = new double[ size ];

          image = new unsigned char[ size* bpp ];
          memcpy( image, data, size* bpp );
        }
        ~cannyAlgorithm( void )
        {
          delete[] Gx;
          delete[] Gy;
          delete[] Direction;
          delete[] Magnitude;
        }
        int max( int a, int b )
        {
          int c = a > b ? a : b;
          return c;
        }
        int min( int a, int b )
        {
          int c = a < b ? a : b;
          return c;
        }
        short getAngle( double X, double Y )
        {
          short Angle;

          if( X* Y > 0 )  // Quadrant 1 or 3
          {
            if( abs( X ) >= abs( Y ) )
              Angle = 0;
            else
              Angle = 180;
          }
          else            // Quadrant 2 or 4
          {
            if( abs(X) >= abs(Y) )
              Angle = 90;
            else
              Angle = 270;
          }

          return( Angle );
        }
        double hypotenuse( double a, double b )
        {
          double h = sqrt( a*a + b*b );
          return(h);
        }
        bool isLocalMax( unsigned offset )
        {
          unsigned bottom     = max(offset - width, 0);
          unsigned top        = min(offset + width, size);
          unsigned left       = max(offset - 1, 0);
          unsigned right      = min(offset + 1, size);
          unsigned bottomLeft = max(bottom - 1, 0);
          unsigned bottomRight= min(bottom + 1, size);
          unsigned topLeft    = max(top - 1, 0);
          unsigned topRight   = min(top + 1, size);

          double thisPoint = 0.0;
          double mag[2]    = { 0.0 };

          switch( Direction[offset] )
          {
            case 0:
            {
              /*   90
                    *
                    |******
                    |******
              -------------* 0
                    |
                    |
              */
              thisPoint = abs( Gx[offset]* Magnitude[offset] );

              mag[0] = abs( Gy[offset]* Magnitude[topRight  ] + ( Gx[offset] - Gy[offset] )* Magnitude[right] );
              mag[1] = abs( Gy[offset]* Magnitude[bottomLeft] + ( Gx[offset] - Gy[offset] )* Magnitude[left ] );
            }break;

            case 90:
            {
              /*
                    90
                    *
               *****|
               *****|
          180 *-------------
                    |
                    |
              */
              thisPoint = abs(Gx[offset]* Magnitude[offset] );

              mag[0] = abs( Gy[offset]* Magnitude[bottomRight] - ( Gx[offset] + Gy[offset])* Magnitude[right] );
              mag[1] = abs( Gy[offset]* Magnitude[topLeft    ] - ( Gx[offset] + Gy[offset])* Magnitude[left ] );
            }break;

            case 180:
            {
              /*
                    |
                    |
          180 *-------------
              ******|
              ******|
                    *
                   270
              */
              thisPoint = abs( Gy[offset]* Magnitude[offset] );

              mag[0] = abs( Gx[offset]* Magnitude[topRight  ] + ( Gy[offset] - Gx[offset] )* Magnitude[top   ] );
              mag[1] = abs( Gx[offset]* Magnitude[bottomLeft] + ( Gy[offset] - Gx[offset] )* Magnitude[bottom] );
            }break;

            case 270:
            {
              /*
                    |
                    |
              -------------* 0
                    |*******
                    |*******
                    *
                   270
              */
              thisPoint = abs( Gy[offset]* Magnitude[offset] );

              mag[0] = abs( Gx[offset]* Magnitude[bottomRight] - ( Gy[offset] + Gx[offset] )* Magnitude[bottom] );
              mag[1] = abs( Gx[offset]* Magnitude[topLeft    ] - ( Gy[offset] + Gx[offset] )* Magnitude[top   ] );
            }break;

            default:
              break;
          }

          if( thisPoint >= mag[0] && thisPoint >= mag[1] )
            return( true );
          return( false );
        }
        void grayScaleCompress( void )
        {
          unsigned char * compressed = new unsigned char[ size ];

          for( unsigned offset = 0; offset < size; offset++ )
            compressed[offset] = image[offset* bpp];

          delete[] image;
          image = new unsigned char[ size ];
          memcpy( image, compressed, size );

          delete[] compressed;
        }
        void continuousTracing( unsigned offset, unsigned char * in, unsigned char * out, unsigned thresholding )
        {
          /*
          The concept is sample:
          I found a possible edge and I will follow this edge until its end.
          Test 8 neighboring pixels and if someone is higher than thresholding then
          that pixel will be another edge and I will follow it.
          This process is repeated until the value of the current pixel tested is null.
          */
          const unsigned edge = 255;

          unsigned dir[2];
          dir[0] = width;      // Top - Bottom
          dir[1] = 1;          // Left - Right

          unsigned top = min( offset + dir[0], size );
          if( in[top] >= thresholding )
            do
            {
              if( !out[top] )
              {
                out[top] = edge;
                continuousTracing( top, in, out, thresholding );
              }
              else
                break;

              top += dir[0];

              if( top > size )
                break;

            }while( in[top] >= thresholding );

          unsigned bottom = max( offset - dir[0], 0 );
          if( in[bottom >= thresholding] )
            do
            {
              if( !out[bottom] )
              {
                out[bottom] = edge;
                continuousTracing( bottom, in, out, thresholding );
              }
              else
                break;

              bottom -= dir[0];

              if( bottom < 0 )
                break;

            }while( in[bottom] >= thresholding );

          unsigned right = min( offset + dir[1], size );
          if( in[right] >= thresholding )
            do
            {
              if( !out[right] )
              {
                out[right] = edge;
                continuousTracing( right, in, out, thresholding );
              }
              else
                break;

              right += dir[1];

              if( right > size )
                break;

            }while( in[right] >= thresholding );

          unsigned left = max( offset - dir[1], 0 );
          if( in[left] >= thresholding )
            do
            {
              if( !out[left] )
              {
                out[left] = edge;
                continuousTracing( left, in, out, thresholding );
              }
              else
                break;

              left -= dir[1];

              if( left < 0 )
                break;

            }while( in[left] >= thresholding );

          unsigned topRight = min( offset + dir[0] + dir[1], size );
          if( in[topRight] >= thresholding )
            do
            {
              if( !out[topRight] )
              {
                out[topRight] = edge;
                continuousTracing( left, in, out, thresholding );
              }
              else
                break;

              topRight += dir[0] + dir[1];

              if( topRight > size )
                break;

            }while( in[topRight] >= thresholding );

          unsigned bottomLeft = max( offset - dir[0] - dir[1], 0 );
          if( in[bottomLeft] >= thresholding )
            do
            {
              if( !out[bottomLeft] )
              {
                out[bottomLeft] = edge;
                continuousTracing( bottomLeft, in, out, thresholding );
              }
              else
                break;

              bottomLeft -= dir[0] - dir[1];

              if( bottomLeft < 0 )
                break;

            }while( in[bottomLeft] >= thresholding );

          unsigned topLeft = min( offset + dir[0] - dir[1], size );
          if( in[topLeft] >= thresholding )
            do
            {
              if( !out[topLeft] )
              {
                out[topLeft] = edge;
                continuousTracing( topLeft, in, out, thresholding );
              }
              else
                break;

              topLeft += dir[0] - dir[1];

              if( topLeft > size )
                break;

            }while( in[topLeft] >= thresholding );

          unsigned bottomRight = max( offset - dir[0] + dir[1], 0 );
          if( in[bottomRight] >= thresholding )
            do
            {
              if( !out[bottomRight] )
              {
                out[bottomRight] = edge;
                continuousTracing( bottomRight, in, out, thresholding );
              }
              else
                break;

              bottomRight -= dir[0] + dir[1];

              if( bottomRight < 0 )
                break;

            }while( in[bottomRight] >= thresholding );

          /* Works with feedback and not will be an infinite loop cause I am saving the new data into a new image */
        }
        void computeGradients( void )
        {
          // Compute Gradients in X
          for( unsigned y = 0; y < height; y++ )
          {
            unsigned offset = y* width;

            Gx[offset] = image[offset + 1] - image[offset];

            offset++;
            for( unsigned x = 1; x < width - 1; x++, offset++ )
              Gx[offset] = image[offset + 1] - image[offset - 1];

            Gx[offset] = image[offset] - image[offset - 1];
          }
          // Compute Gradients in Y
          for( unsigned x = 0; x < width; x++ )
          {
            unsigned offset = x;

            Gy[offset] = image[offset + width] - image[offset];

            offset += width;
            for( unsigned y = 1; y < height - 1; y++, offset += width )
              Gy[offset] = image[offset + width] - image[offset - width];

            Gy[offset] = image[offset] - image[offset - width];
          }
          // Hypotenuse = sqrt(x^2 + y^2)
          for( unsigned y = 0, offset = 0; y < height; y++ )
            for( unsigned x = 0; x < width; x++, offset++ )
              Magnitude[offset] = hypotenuse( Gx[offset], Gy[offset] );
          // Okay, edges of the image must be null
          for( unsigned x = 0; x < width; x++ )
            Magnitude[x] = 0;

          for( unsigned x = 0, offset = width* (height - 1); x < width; x++, offset++ )
            Magnitude[offset] = 0;

          for( unsigned y = 0; y < width* height; y += width )
            Magnitude[y] = 0;

          for( unsigned y = 0, offset = width - 1; y < width* height; y += width, offset += width)
            Magnitude[offset] = 0;
        }
        void nonMaxSupress( void )
        {
          /* Compute magnitudes direction and save it */
          for( unsigned y = 0, offset = 0; y < height; y++ )
            for( unsigned x = 0; x < width; x++, offset++ )
              Direction[offset] = getAngle( Gx[offset], Gy[offset] );
          /*
              The most complicated part:
              If the pixel is not a local maximum then kill it.
              How do I know if my point is a local max ?
              I will compare the current pixel with neighboring pixels in the gradient direction.
              Remember: Pixel with null magnitude are not candidate to be an edge.
          */
          for( unsigned y = 0, offset = 0; y < height; y++ )
            for( unsigned x = 0; x < width; x++, offset++ )
            {
              if( Magnitude[offset] && isLocalMax(offset) )
                image[offset] = Magnitude[offset] > 255 ? 255 : (unsigned char)Magnitude[offset];
              else
                image[offset] = 0;
            }
        }
        void hysteresis( float lowScale, float highScale )
        {
          /*
          We need a High value and a Low value, High value will be the edge color.
          All pixels with color higher than ' Hight value ' will be edges
          and we will follow this pixel until another pixel is founded.
          The pixel founded must be a color higher than ' Low value ', if that is the case
          then we will set this pixel like edge, else it will be background ( null ).
          */

          lowScale    = lowScale <= 0.0f ? 0.01f : lowScale > 1.0f ? 1.0f : lowScale;
          highScale   = highScale <= 0.0f ? 0.01f : highScale > 1.0f ? 1.0f : highScale;

          unsigned char globalMax = 0;
          for( unsigned offset = 0; offset < size; offset++ )
            if( image[offset] > globalMax )
              globalMax = image[offset];

          unsigned highV = globalMax* highScale;
          unsigned lowV = globalMax* lowScale;

          unsigned char * finalPic = new unsigned char[ size ];
          memset( finalPic, 0, size );

          for( unsigned y = 1,offset = 1; y < height - 1; y++ )
            for( unsigned x = 1; x < width - 1; x++, offset++ )
              if( image[offset] >= highV && !finalPic[offset] )
              {
                finalPic[offset] = 255;
                continuousTracing( offset, image, finalPic, lowV );
              }

          delete[] image;
          image = new unsigned char[ size ];
          memcpy( image, finalPic, size );

          delete[] finalPic;
        }
        void grayScaleDecompress( void )
        {
          size = width* height* bpp;
          unsigned char * decompressed = new unsigned char[ size ];

          for( unsigned offset = 0; offset < width* height; offset++ )
            decompressed[offset*bpp + 0] = decompressed[offset* bpp + 1] = decompressed[offset* bpp + 2] = image[offset];

          delete[] image;
          image = new unsigned char[ size ];
          memcpy( image, decompressed, size );

          delete[] decompressed;
        }
        void AutoEdgeDetection( unsigned char * data, float lowV, float highV )
        {
          grayScaleCompress();
          computeGradients();
          nonMaxSupress();
          hysteresis(lowV, highV);
          grayScaleDecompress();

          memcpy( data, image, size );
        }
        unsigned char * get_data( void )
        {
          grayScaleDecompress();
          return( image );
        }
    };

void algoritmoCanny(jint width, jint height, jobject pNV21FrameData) {

  cannyAlgorithm cpix((unsigned char *)pNV21FrameData, (unsigned)width, (unsigned)height, (unsigned) 1 );
  h_blur ((unsigned char *)pNV21FrameData, height, width );

  cpix.AutoEdgeDetection((unsigned char *)pNV21FrameData, 0.1f, 0.11f); // 0.2f, 0.25f// 0.1f, 0.1f // 0.1f, 0.20f //  0.9f, 0.1f

  //   applyGrayScale((unsigned char *)pNV21FrameData,width,height);


}

extern "C" JNIEXPORT jobject JNICALL
Java_com_sample_textureview_ViewActivity_getMinimumCompatiblePreviewSize(
    JNIEnv *env, jobject instance, jlong ndkCameraObj) {
  if (!ndkCameraObj) {
    return nullptr;
  }
  CameraAppEngine *pApp = reinterpret_cast<CameraAppEngine *>(ndkCameraObj);
  jclass cls = env->FindClass("android/util/Size");
  jobject previewSize =
      env->NewObject(cls, env->GetMethodID(cls, "<init>", "(II)V"),
                     pApp->GetCompatibleCameraRes().width,
                     pApp->GetCompatibleCameraRes().height);

  // aqui quizas se podria llamar a algun metodo que modifique el previewSize
  //stringFromJNI(frameWidth, frameHeight, raw, pixels);
  //algoritmoCanny(pApp->GetCompatibleCameraRes().width,
    //      pApp->GetCompatibleCameraRes().height, previewSize);
  return previewSize;
}

/**
 * getCameraSensorOrientation()
 * @ return camera sensor orientation angle relative to Android device's
 * display orientation. This sample only deal to back facing camera.
 */
extern "C" JNIEXPORT jint JNICALL
Java_com_sample_textureview_ViewActivity_getCameraSensorOrientation(
    JNIEnv *env, jobject instance, jlong ndkCameraObj) {
  ASSERT(ndkCameraObj, "NativeObject should not be null Pointer");
  CameraAppEngine *pApp = reinterpret_cast<CameraAppEngine *>(ndkCameraObj);
  return pApp->GetCameraSensorOrientation(ACAMERA_LENS_FACING_BACK);
}

/**
 * OnPreviewSurfaceCreated()
 *   Notification to native camera that java TextureView is ready
 *   to preview video. Simply create cameraSession and
 *   start camera preview
 */
extern "C" JNIEXPORT void JNICALL
Java_com_sample_textureview_ViewActivity_onPreviewSurfaceCreated(
    JNIEnv *env, jobject instance, jlong ndkCameraObj, jobject surface) {
  ASSERT(ndkCameraObj && (jlong)pEngineObj == ndkCameraObj,
         "NativeObject should not be null Pointer");
  CameraAppEngine *pApp = reinterpret_cast<CameraAppEngine *>(ndkCameraObj);
  pApp->CreateCameraSession(surface);
  pApp->StartPreview(true);
}

/**
 * OnPreviewSurfaceDestroyed()
 *   Notification to native camera that java TextureView is destroyed
 *   Native camera would:
 *      * stop preview
 */
extern "C" JNIEXPORT void JNICALL
Java_com_sample_textureview_ViewActivity_onPreviewSurfaceDestroyed(
    JNIEnv *env, jobject instance, jlong ndkCameraObj, jobject surface) {
  CameraAppEngine *pApp = reinterpret_cast<CameraAppEngine *>(ndkCameraObj);
  ASSERT(ndkCameraObj && pEngineObj == pApp,
         "NativeObject should not be null Pointer");
  jclass cls = env->FindClass("android/view/Surface");
  jmethodID toString =
      env->GetMethodID(cls, "toString", "()Ljava/lang/String;");

  jstring destroyObjStr =
      reinterpret_cast<jstring>(env->CallObjectMethod(surface, toString));
  const char *destroyObjName = env->GetStringUTFChars(destroyObjStr, nullptr);

  jstring appObjStr = reinterpret_cast<jstring>(
      env->CallObjectMethod(pApp->GetSurfaceObject(), toString));
  const char *appObjName = env->GetStringUTFChars(appObjStr, nullptr);

  ASSERT(!strcmp(destroyObjName, appObjName), "object Name MisMatch");

  env->ReleaseStringUTFChars(destroyObjStr, destroyObjName);
  env->ReleaseStringUTFChars(appObjStr, appObjName);

  pApp->StartPreview(false);
}
