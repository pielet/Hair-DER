#include "Camera.h"
#include <iostream>

Camera::Camera( const Vector3s& init_lookAt, const Vector3s& init_pos, const Vector3s& init_up ) :
    m_init_lookAt( init_lookAt ),
	m_init_forward(init_lookAt - init_pos)
{
	m_init_dist = m_init_forward.norm();
	m_init_forward.normalize();
    m_init_right = m_init_forward.cross(Vector3s(0, 1.0, 0)).normalized();

    center();
}

void Camera::mouse_button_callback( int button, int action ) {
    if (!TwEventMouseButtonGLFW( button, action )) {
        if (button == GLFW_MOUSE_BUTTON_MIDDLE)
            m_is_mouse_middle_down = (action == GLFW_PRESS);
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            m_is_mouse_right_down = (action == GLFW_PRESS);
    }
}

void Camera::cursor_position_callback( double xpos, double ypos ) {
    if (!TwEventMousePosGLFW( int(xpos), int(ypos))) {
        double xoffset = xpos - m_xpos;
        double yoffset = ypos - m_ypos;

		Vector3s up = m_right.cross(m_forward).normalized();
        if (m_is_mouse_middle_down) {   // translation first
            m_lookAt += m_translation_rate * (-xoffset * m_right + yoffset * up);
            m_dirty = true;
        }
        else if (m_is_mouse_right_down) {
            m_forward = Eigen::AngleAxisd( m_rotation_rate * xoffset, Vector3s(0, 1.0, 0)) * m_forward;
            m_forward = Eigen::AngleAxisd( m_rotation_rate * yoffset, m_right) * m_forward;
			m_right = m_forward.cross(Vector3s(0, 1.0, 0)).normalized();
            m_dirty = true;
        }

        m_xpos = xpos;
        m_ypos = ypos;
    }
}

void Camera::scroll_callback( double yoffset ) {
    if (!TwEventMouseWheelGLFW( int(yoffset) )) {
        m_dist += m_dist_rate * yoffset;
        m_dirty = true;
    }
}

const Matrix4s& Camera::getLookAt() {
    if (m_dirty) {
        m_camera = lookAt(m_lookAt - m_dist * m_forward, m_lookAt, m_right.cross(m_forward).normalized() );
        m_dirty = false;
    }
    return m_camera;
}


Matrix4s Camera::lookAt(Vector3s const& eye, Vector3s const& center, Vector3s const& up)
{
    Vector3s f(center - eye);
    Vector3s s(f.cross(up));
    Vector3s  u(s.cross(f));
    f.normalize();
    s.normalize();
    u.normalize();

    Matrix4s  Result = Matrix4s::Identity();
    Result(0, 0) = s.x();
    Result(0, 1) = s.y();
    Result(0, 2) = s.z();
    Result(1, 0) = u.x();
    Result(1, 1) = u.y();
    Result(1, 2) = u.z();
    Result(2, 0) = -f.x();
    Result(2, 1) = -f.y();
    Result(2, 2) = -f.z();
    Result(0, 3) = -s.dot(eye);
    Result(1, 3) = -u.dot(eye);
    Result(2, 3) = f.dot(eye);
    return Result;
}

Matrix4s Camera::perspective(double fovy, double aspect, double zNear, double zFar)
{

    double const tanHalfFovy = tan(fovy / 2);

    Matrix4s  Result = Matrix4s::Zero();
    Result(0, 0) = 1 / (aspect * tanHalfFovy);
    Result(1, 1) = 1 / (tanHalfFovy);
    Result(2, 2) = -(zFar + zNear) / (zFar - zNear);
    Result(3, 2) = -1.0;
    Result(2, 3) = -(2 * zFar * zNear) / (zFar - zNear);
    return Result;
}

void Camera::center() {
    m_dirty = true;
    
	m_dist = m_init_dist;
	m_lookAt = m_init_lookAt;
	m_forward = m_init_forward;
    m_right = m_init_right;
}