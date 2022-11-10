#pragma once

#include <cmath>
#include <Eigen/Geometry>

class Pose2d
{
private:
  Eigen::Vector2d translation_;
  Eigen::Rotation2D<double> rotation_;

public:
  Pose2d() : translation_(0., 0.), rotation_(0.) {}
  Pose2d(const Eigen::Vector2d& trans, const Eigen::Rotation2D<double>& rotat) : translation_(trans), rotation_(rotat) {}
  Pose2d(const Eigen::Vector2d& trans, const double rotat) : translation_(trans), rotation_(rotat) {}
  Pose2d(const double x, const double y, const double rotat) : translation_(x, y), rotation_(rotat) {}

  template <typename T>
  Eigen::Transform<T, 3, Eigen::Affine> toAffine() const {
    Eigen::Transform<T, 3, Eigen::Affine> t;
    t.pretranslate(Eigen::Matrix<T, 3, 1>(x(), y(), 0.));
    t.rotate(Eigen::AngleAxis<T>(rotation_.angle(), Eigen::Matrix<T, 3, 1>::UnitZ()));
    return t;
  }

  template <typename T>
  static Pose2d fromAffine(const Eigen::Transform<T, 3, Eigen::Affine> &t) {
    Pose2d p(t.translation().x(), t.translation().y(), 0);
    p.set_rotation(t.rotation());
    return p;
  }

  Eigen::Vector2d translation() const {return translation_;}
  Eigen::Rotation2D<double> rotation() const {return rotation_;}

  void set_translation(const Eigen::Vector2d& trans) {translation_ = trans;}
  void set_translation(double x, double y) {translation_ = Eigen::Vector2d(x, y);}
  void set_rotation(const double rotat) {rotation_ = Eigen::Rotation2D<double>(rotat);}
  template <typename T>
  void set_rotation(const Eigen::Matrix<T, 3, 3> &quat) {
    const Eigen::Matrix<T, 3, 1> _dir = quat * Eigen::Matrix<T, 3, 1>::UnitX();
    double yaw = atan2(_dir.y(), _dir.x());
    set_rotation(yaw);
  }
  template <typename T>
  void set_rotation(const Eigen::Quaternion<T> &quat) {
    set_rotation(quat.toRotationMatrix());
  }

  Pose2d inverse() const {
    Eigen::Rotation2D<double> rotat_part = rotation_.inverse();
    Eigen::Vector2d trans_part = -(rotat_part * translation_);
    return Pose2d(trans_part, rotat_part);
  }

  Pose2d operator*(const Pose2d& rhs) const {
    Eigen::Vector2d trans_part = this->rotation() * rhs.translation() + this->translation();
    Eigen::Rotation2D<double> rotat_part = this->rotation() * rhs.rotation();
    return Pose2d(trans_part, Pose2d::normalize_angle(rotat_part.angle()));
  }

  Eigen::Vector2d operator*(const Eigen::Vector2d& rhs) const {
    return this->rotation() * rhs + this->translation();
  }

  static Pose2d interpolate(const Pose2d& start, const Pose2d& end, double ratio = 0.5) {
    Pose2d delta_pose = start.inverse() * end;
    if (fabs(delta_pose.y()) < 0.001) {
      return start * Pose2d(delta_pose.translation() * ratio, delta_pose.angle() * ratio);
    }
    double new_delta_theta = atan2(delta_pose.y(), delta_pose.x());
    double R = delta_pose.x() / sin(2. * new_delta_theta);

    //in theory, new_delta_theta * 2 = delta_pose.angle()
    //and the interpolated_pose's angle = ratio * new_delta_theta * 2 = ratio * delta_pose.angle()
    double inter_angle = ratio * (new_delta_theta*2 + delta_pose.angle()) / 2.;
    double dx = R * sin(inter_angle);
    double dy = R * (1. - cos(inter_angle));
    return start * Pose2d(dx, dy, inter_angle);
  }

  double x() const {return translation_.x();}
  double y() const {return translation_.y();}
  double angle() const {return normalize_angle(rotation_.angle());}
  double norm() const {return translation_.norm();}

private:
  static double normalize_angle(double a) {
    static double PI2 = 2. * M_PI;
    a = fmod(a + M_PI, PI2);
    if (a < 0)
        a += PI2;
    return a - M_PI;
  }
};
