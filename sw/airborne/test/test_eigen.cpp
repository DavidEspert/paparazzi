#define EIGEN_NO_MALLOC 1
#define EIGEN_NO_DEBUG 1
#define EIGEN_NO_STATIC_ASSERT 1
#define eigen_assert(_c) { }
#define assert(ignore) ((void)0)

//#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main()
{
  Matrix3f m = Matrix3f::Random(3,3);
  //Vector3f v = Vector3f(0., 0., 0.);
  Vector3f v = Vector3f::Zero();
  Quaternionf q = Quaternionf::Identity();

  while(1) {
    v(0) += m(1,2);
    if (v(0) < -1000 || v(0) > 1000) { v(0) = 0.; }
    m(0,0) = v(0);
    q.normalize();
  }
}

//void abort(void) {}
