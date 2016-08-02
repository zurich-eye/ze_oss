# Pangolin Wrappers for Debugging purposes

**Warning:**
Do not use the state watchers and pangolin singleton in a production build as it might result in unexpected behaviour.
The tools are designed for the purpose of debugging private and internal states in a quick and easy way.
The pangolin singleton is thread-safe but is untested in shared-library environments which might result in unexpected behaviour.

## Sample Usage

Add a (temporary) package dependency on `ze_pangolin` and include the pangolin header in the file where you want to watch
an expression: `#include <ze/pangolin/pangolin_insight.hpp>`.

### Watch a class member
Given a simple class:
```
class Data
{
private:
  int a;
  double b;
  uint c;
}
```

We might be interested in the internal states `a`, `b` or `c` and how they evolve over the curse of time or iterations.
To visualize the members in a plot every time the member is modified, simply wrap them in a `PANGOLIN_WATCH(TYPE, CLASS_MEMBER_NAME)` statement:
```
class Data
{
private:
  PANGOLIN_WATCH(int, a);
  PANGOLIN_WATCH(double, b);
  PANGLIN_WATCH(uint, c);
}
```

The current type watches support all common, numeric primitive types, but **no** Eigen-types.

### Watch a temporary expression
As opposed to a class member watch that actively tracks changes of the member one might want to explicitly push updates to the plots.
The expression watch `PANGOLIN_WATCH_EXPR(VALUE, TYPE, PLOT_NAME)` supports all types that `PANGOLIN_WATCH` accepts and Eigen-vector-types.

```
Vector3 tmp = Vector3::Random();
PANGOLIN_WATCH_EXPR(tmp, Vector3, The_Temporary);
```

The `PLOT_NAME` has to be a unique name used as identifier for a given plot. Any data provided with the same identifier will end up in the same plot.

## Known Issues:

* If a class member is constructor initialized, the plot-name will not be set properly but is attributed a default value. Tracking multiple class-members might currently result in mixed plots. A solution would be to attribute random plot-identifiers instead of a static value (currently `Default`).
