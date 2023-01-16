#ifndef COSM_BRANCH_HINTS_H__
#define COSM_BRANCH_HINTS_H__

// TODO: see if these work on all the compilers
// C++20 added something similar, but it's a bit weirder
#define COSM_LIKELY(x) __builtin_expect((x), 1)
#define COSM_UNLIKELY(x) __builtin_expect((x), 0)

#endif // COSM_BRANCH_HINTS_H__
