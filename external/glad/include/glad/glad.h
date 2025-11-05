#ifndef __glad_h_
#define __glad_h_

#ifdef __gl_h_
#error OpenGL header already included, remove this include, glad.h is meant to replace the header
#endif

#define __gl_h_

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__) && !defined(__SCITECH_SNAP__)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>
#endif

#ifndef APIENTRY
#define APIENTRY
#endif
#ifndef APIENTRYP
#define APIENTRYP APIENTRY *
#endif
#ifndef GLAPI
#define GLAPI extern
#endif

typedef void GLvoid;
typedef unsigned int GLenum;
typedef float GLfloat;
typedef int GLint;
typedef int GLsizei;
typedef unsigned int GLbitfield;
typedef unsigned char GLboolean;
typedef signed char GLbyte;
typedef short GLshort;
typedef unsigned char GLubyte;
typedef unsigned short GLushort;
typedef unsigned int GLuint;
typedef int GLintptr;
typedef int GLsizeiptr;
typedef double GLdouble;

#define GL_DEPTH_TEST                     0x0B71
#define GL_LEQUAL                        0x0203
#define GL_LINE_SMOOTH                   0x0B20
#define GL_LINE_SMOOTH_HINT              0x0C52
#define GL_NICEST                        0x1102
#define GL_COLOR_BUFFER_BIT              0x00004000
#define GL_DEPTH_BUFFER_BIT              0x00000100
#define GL_LINES                         0x0001
#define GL_QUAD_STRIP                    0x0008
#define GL_PROJECTION                    0x1701
#define GL_MODELVIEW                     0x1700
#define GL_TEXTURE_2D                    0x0DE1

#define GL_TRUE                          1
#define GL_FALSE                         0

typedef void (APIENTRYP PFNGLCLEARCOLORPROC) (GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
typedef void (APIENTRYP PFNGLCLEARPROC) (GLbitfield mask);
typedef void (APIENTRYP PFNGLENABLEPROC) (GLenum cap);
typedef void (APIENTRYP PFNGLDEPTHFUNCPROC) (GLenum func);
typedef void (APIENTRYP PFNGLHINTPROC) (GLenum target, GLenum mode);
typedef void (APIENTRYP PFNGLVIEWPORTPROC) (GLint x, GLint y, GLsizei width, GLsizei height);
typedef void (APIENTRYP PFNGLLINEWIDTHPROC) (GLfloat width);
typedef void (APIENTRYP PFNGLCOLOR3FPROC) (GLfloat red, GLfloat green, GLfloat blue);
typedef void (APIENTRYP PFNGLBEGINPROC) (GLenum mode);
typedef void (APIENTRYP PFNGLENDPROC) (void);
typedef void (APIENTRYP PFNGLVERTEX3FPROC) (GLfloat x, GLfloat y, GLfloat z);
typedef void (APIENTRYP PFNGLNORMAL3FPROC) (GLfloat nx, GLfloat ny, GLfloat nz);
typedef void (APIENTRYP PFNGLLOADIDENTITYPROC) (void);
typedef void (APIENTRYP PFNGLLOADMATRIXFPROC) (const GLfloat *m);
typedef void (APIENTRYP PFNGLMATRIXMODEPROC) (GLenum mode);
typedef void (APIENTRYP PFNGLORTHOPROC) (GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar);
typedef void (APIENTRYP PFNGLPUSHMATRIXPROC) (void);
typedef void (APIENTRYP PFNGLPOPMATRIXPROC) (void);
typedef void (APIENTRYP PFNGLTRANSLATEFPROC) (GLfloat x, GLfloat y, GLfloat z);
typedef void (APIENTRYP PFNGLROTATEFPROC) (GLfloat angle, GLfloat x, GLfloat y, GLfloat z);
typedef const GLubyte* (APIENTRYP PFNGLGETSTRINGPROC) (GLenum name);

GLAPI PFNGLCLEARCOLORPROC glClearColor;
GLAPI PFNGLCLEARPROC glClear;
GLAPI PFNGLENABLEPROC glEnable;
GLAPI PFNGLDEPTHFUNCPROC glDepthFunc;
GLAPI PFNGLHINTPROC glHint;
GLAPI PFNGLVIEWPORTPROC glViewport;
GLAPI PFNGLLINEWIDTHPROC glLineWidth;
GLAPI PFNGLCOLOR3FPROC glColor3f;
GLAPI PFNGLBEGINPROC glBegin;
GLAPI PFNGLENDPROC glEnd;
GLAPI PFNGLVERTEX3FPROC glVertex3f;
GLAPI PFNGLNORMAL3FPROC glNormal3f;
GLAPI PFNGLLOADIDENTITYPROC glLoadIdentity;
GLAPI PFNGLLOADMATRIXFPROC glLoadMatrixf;
GLAPI PFNGLMATRIXMODEPROC glMatrixMode;
GLAPI PFNGLORTHOPROC glOrtho;
GLAPI PFNGLPUSHMATRIXPROC glPushMatrix;
GLAPI PFNGLPOPMATRIXPROC glPopMatrix;
GLAPI PFNGLTRANSLATEFPROC glTranslatef;
GLAPI PFNGLROTATEFPROC glRotatef;
GLAPI PFNGLGETSTRINGPROC glGetString;

#define GL_EXTENSIONS 0x1F03

#ifdef __cplusplus
extern "C" {
#endif

int gladLoadGL(void);
int gladLoadGLLoader(void* (*load)(const char*));

#ifdef __cplusplus
}
#endif

#endif

