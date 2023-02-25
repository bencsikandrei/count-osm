#ifndef COSM_LOCK_GUARD_H__
#define COSM_LOCK_GUARD_H__

namespace cosm
{

/**
 * @brief RAII struct for mutex lock/unlock within scope
 * @tparam mutex_t must have lock/unlock
 */
template<typename mutex_t>
struct [[nodiscard]] lock_guard
{
  /**
   * @brief Lock the mutex
   * @param m mutex to lock
   */
  explicit lock_guard(mutex_t& m) noexcept
    : mutex(m)
  {
    mutex.lock();
  }

  /**
   * @brief Unlocks mutex
   */
  ~lock_guard() noexcept { mutex.unlock(); }

  lock_guard(lock_guard const&) = delete;
  lock_guard& operator=(lock_guard const&) = delete;

  mutex_t& mutex;
};

} // namespace cosm

#endif // COSM_LOCK_GUARD_H__
