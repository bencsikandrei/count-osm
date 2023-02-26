#ifndef COSM_FORWARD_H__
#define COSM_FORWARD_H__

#include "traits.h"

namespace cosm
{

template<typename T>
[[nodiscard]] constexpr T&&
forward(remove_reference_t<T>& t) noexcept
{
  return static_cast<T&&>(t);
}

template<typename T>
[[nodiscard]] constexpr T&&
forward(remove_reference_t<T>&& t) noexcept
{
  return static_cast<T&&>(t);
}

} // namespace cosm

#endif // COSM_FORWARD_H__
