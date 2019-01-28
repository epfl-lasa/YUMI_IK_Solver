#include "mathlib_eigen_conversions.h"


Eigen::MatrixXd M2E_m(MathLib::Matrix mm) {
  Eigen::Map<Eigen::MatrixXd> ei( mm.Array(), mm.ColumnSize(), mm.RowSize());

  return ei;
}

//Mat M2E_v(MathLib::Vector mm) {
Eigen::VectorXd M2E_v(MathLib::Vector mm) {
  Eigen::Map<Eigen::VectorXd> ei( mm.Array(), mm.Size() );

  return ei;
}

Eigen::MatrixXd M2E_v(MathLib::Vector3 mm) {
  Eigen::Map<Eigen::VectorXd> ei(mm.Array(), 3);

  return ei;
}

MathLib::Vector E2M_v(Eigen::VectorXd ev) {
  MathLib::Vector mv( ev.rows() );

  mv.Set( ev.data(), ev.rows() );
  return mv;
}

MathLib::Vector3 E2M_v3(Eigen::VectorXd ev) {
  //  ROS_ASSERT_MSG(ev.rows() == 3 , "Trying to convert MathLib::Vector3 to Eigen but wrong size !");
  MathLib::Vector3 mv;

  mv.Set( ev(0), ev(1), ev(2) );
  return mv;
}

MathLib::Matrix E2M_m(Eigen::MatrixXd em) {
  MathLib::Matrix mm( em.rows(), em.cols() );

  mm.Set( em.data(), em.rows(), em.cols() );
  return mm;
}

MathLib::Matrix3 E2M_m3(Eigen::MatrixXd em) {
  //  ROS_ASSERT_MSG((em.cols() == 3) && (em.rows() == 3) , "Trying to convert MathLib::Matrix3 to Eigen but wrong size !");
  MathLib::Matrix3 mm;

  mm.Set( em.data() );
  return mm;
}
