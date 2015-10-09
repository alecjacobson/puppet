#ifdef __GNUC__
// This is how it should be done
#  if __GNUC__ >= 4
#    if __GNUC_MINOR__ >= 6
#      pragma GCC diagnostic push
#      pragma GCC diagnostic ignored "-Weffc++"
#    endif
#  endif
// This is a hack
#  pragma GCC system_header
#endif

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#ifdef __GNUC__
#  if __GNUC__ >= 4
#    if __GNUC_MINOR__ >= 6
#      pragma GCC diagnostic pop
#    endif
#  endif
#endif
