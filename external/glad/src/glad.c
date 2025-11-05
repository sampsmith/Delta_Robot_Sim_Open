#include <glad/glad.h>
#include <GLFW/glfw3.h>

static void* glad_gl_get_proc(const char *name) {
    return (void*)glfwGetProcAddress(name);
}

int gladLoadGLLoader(void* (*load)(const char*)) {
    if (!load) return 0;
    
    glClearColor = (PFNGLCLEARCOLORPROC)load("glClearColor");
    glClear = (PFNGLCLEARPROC)load("glClear");
    glEnable = (PFNGLENABLEPROC)load("glEnable");
    glDepthFunc = (PFNGLDEPTHFUNCPROC)load("glDepthFunc");
    glHint = (PFNGLHINTPROC)load("glHint");
    glViewport = (PFNGLVIEWPORTPROC)load("glViewport");
    glLineWidth = (PFNGLLINEWIDTHPROC)load("glLineWidth");
    glColor3f = (PFNGLCOLOR3FPROC)load("glColor3f");
    glBegin = (PFNGLBEGINPROC)load("glBegin");
    glEnd = (PFNGLENDPROC)load("glEnd");
    glVertex3f = (PFNGLVERTEX3FPROC)load("glVertex3f");
    glNormal3f = (PFNGLNORMAL3FPROC)load("glNormal3f");
    glLoadIdentity = (PFNGLLOADIDENTITYPROC)load("glLoadIdentity");
    glLoadMatrixf = (PFNGLLOADMATRIXFPROC)load("glLoadMatrixf");
    glMatrixMode = (PFNGLMATRIXMODEPROC)load("glMatrixMode");
    glOrtho = (PFNGLORTHOPROC)load("glOrtho");
    glPushMatrix = (PFNGLPUSHMATRIXPROC)load("glPushMatrix");
    glPopMatrix = (PFNGLPOPMATRIXPROC)load("glPopMatrix");
    glTranslatef = (PFNGLTRANSLATEFPROC)load("glTranslatef");
    glRotatef = (PFNGLROTATEFPROC)load("glRotatef");
    glGetString = (PFNGLGETSTRINGPROC)load("glGetString");
    
    return 1;
}

int gladLoadGL(void) {
    return gladLoadGLLoader((void*(*)(const char*))glad_gl_get_proc);
}

