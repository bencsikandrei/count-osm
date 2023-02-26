#ifndef COSM_THREAD_H__
#define COSM_THREAD_H__

#include <pthread.h>
#include <unistd.h>

namespace cosm
{

inline long
thread_get_hardware_concurrency() noexcept
{
  // or we could also use _SC_NPROCESSORS_ONLN
  return ::sysconf(_SC_NPROCESSORS_CONF);
}

struct thread final
{
  constexpr thread() = default;

  [[nodiscard]] int start(void* (*func)(void*), void* arg) noexcept
  {
    return pthread_create(&m_handle, nullptr, func, arg);
  }

  [[nodiscard]] int join() noexcept
  {
    // MUST be joinable!
    return pthread_join(m_handle, nullptr);
  }

  [[nodiscard]] constexpr bool joinable() const noexcept
  {
    return m_handle != 0u;
  }

  constexpr unsigned long id() const noexcept { return m_handle; }

  pthread_t m_handle = 0u;
};

static_assert(sizeof(thread) == 8);

} // namespace cosm

#endif // COSM_THREAD_H__
