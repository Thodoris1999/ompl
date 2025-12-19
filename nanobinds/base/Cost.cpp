#include <nanobind/nanobind.h>
#include "ompl/base/Cost.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::base::init_Cost(nb::module_& m)
{
    // TODO [ob::Cost][IMPLEMENT]
    nb::class_<ompl::base::Cost>(m, "Cost")
        ;
}
