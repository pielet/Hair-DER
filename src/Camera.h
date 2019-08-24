#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <GLFW/glfw3.h>
#include <AntTweakBar.h>
#include "MathDefs.h"

class Camera
{    
    double m_init_dist;
    Vector3s m_init_lookAt;
    Vector3s m_init_up;
    Vector3s m_init_right;

    double m_dist;
    Vector3s m_lookAt;
    Vector3s m_up;
    Vector3s m_right;

    const double m_translation_rate = 0.001;
    const double m_rotation_rate = -0.003;
    const double m_dist_rate = -0.1;

    bool m_is_mouse_middle_down = false;
    bool m_is_mouse_right_down = false;
    double m_xpos = 0.0;
    double m_ypos = 0.0;

    bool m_dirty = true;
    Matrix4s m_camera = Matrix4s::Identity();

public:
    Camera( const Vector3s& init_lookAt, const Vector3s& init_pos, const Vector3s& init_up );

    const Matrix4s& getLookAt();

    void center();
    
    void mouse_button_callback( int button, int action );
    void cursor_position_callback( double xpos, double ypos );
    void scroll_callback( double yoffset );

    static Matrix4s lookAt( const Vector3s& eye, const Vector3s& center, const Vector3s& up );
    static Matrix4s perspective( double fovy, double aspect, double zNear, double zFar );
    static double radians( double angle ) { return M_PI * (angle / 180.0); }
};

#endif