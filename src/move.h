#ifndef COSM_MOVE_H__
#define COSM_MOVE_H__

#include "traits.h"

namespace cosm
{

template<typename T>
[[nodiscard]] constexpr typename remove_reference<T>::type&&
move(T&& move_from) noexcept
{
  return static_cast<typename remove_reference<T>::type&&>(move_from);
}

} // namespace cosm

#endif // COSM_MOVE_H__
