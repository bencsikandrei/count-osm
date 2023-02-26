#ifndef COSM_MUTEX_H_
#define COSM_MUTEX_H_

#include <pthread.h>

namespace cosm
{

struct mutex final
{
  constexpr mutex() = default;
  ~mutex() noexcept { static_cast<void>(pthread_mutex_destroy(&m_mutex)); }

  mutex(const mutex&) = delete;
  mutex& operator=(const mutex&) = delete;

  void lock() noexcept { static_cast<void>(pthread_mutex_lock(&m_mutex)); }
  void unlock() noexcept { static_cast<void>(pthread_mutex_unlock(&m_mutex)); }

  pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
};

} // namespace cosm

#endif // COSM_MUTEX_H_
