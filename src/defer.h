#ifndef COSM_DEFER_H__
#define COSM_DEFER_H__

#include "forward.h"

namespace cosm
{

template<typename F>
struct [[nodiscard]] defer
{
  template<typename FF = F>
  defer(FF&& f) noexcept
    : m_f(cosm::forward<FF>(f))
  {
  }
  ~defer() noexcept { m_f(); }

  defer(const defer&) = delete;
  defer(defer&&) = delete;

private:
  F m_f;
};

template<typename F>
auto
make_defer(F&& f)
{
  return defer<F>(cosm::forward<F>(f));
}

} // namespace cosm

#endif // COSM_DEFER_H__
