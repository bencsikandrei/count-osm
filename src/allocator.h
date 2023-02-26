#ifndef COSM_ALLOCATOR_H_
#define COSM_ALLOCATOR_H_

#include <cstdint>

namespace cosm
{

static inline uint64_t
align_up(uint64_t n, uint64_t alignment) noexcept
{
  return (n + (alignment - 1u)) & ~(alignment - 1u);
}

static inline void*
align_memory_up(void* ptr, uint64_t alignment) noexcept
{
  const uintptr_t intptr = reinterpret_cast<uintptr_t>(ptr);
  const uintptr_t aligned = (intptr + (alignment - 1u)) & ~(alignment - 1u);
  return reinterpret_cast<void*>(aligned);
}

struct bump_allocator
{
  bump_allocator() = default;

  bump_allocator(void* memory, uint64_t size)
    : m_memory_start(static_cast<uint8_t*>(memory))
    , m_memory_current(m_memory_start)
    , m_memory_end(m_memory_start + size)
  {
  }

  bump_allocator(const bump_allocator&) = delete;
  bump_allocator& operator=(const bump_allocator&) = delete;

  bump_allocator(bump_allocator&& rhs) noexcept
    : m_memory_start(rhs.m_memory_start)
    , m_memory_current(rhs.m_memory_current)
    , m_memory_end(rhs.m_memory_end)
  {
    rhs.m_memory_start = nullptr;
    rhs.m_memory_current = nullptr;
    rhs.m_memory_end = nullptr;
  }

  bump_allocator& operator=(bump_allocator&& rhs) noexcept
  {
    m_memory_start = rhs.m_memory_start;
    m_memory_current = rhs.m_memory_current;
    m_memory_end = rhs.m_memory_end;
    rhs.m_memory_start = nullptr;
    rhs.m_memory_current = nullptr;
    rhs.m_memory_end = nullptr;
    return *this;
  }

  void* allocate_raw(uint64_t how_many_bytes, uint32_t alignment) noexcept
  {
    // TODO: what happens when we `allocate_raw(0, ..)`?
    const uint64_t how_many_bytes_aligned = align_up(how_many_bytes, alignment);
    void* current = align_memory_up(m_memory_current, alignment);
    if ((static_cast<uint8_t*>(current) + how_many_bytes_aligned) >=
        m_memory_end)
    {
      return nullptr;
    }
    m_memory_current = static_cast<uint8_t*>(current) + how_many_bytes_aligned;
    return current;
  }

  template<typename Pod>
  Pod* allocate(uint64_t how_many_items,
                uint32_t alignment = alignof(Pod)) noexcept
  {
    return reinterpret_cast<Pod*>(
      allocate_raw(sizeof(Pod) * how_many_items, alignment));
  }

  bool ok() const noexcept
  {
    return m_memory_current != nullptr && m_memory_end != nullptr;
  }

  uint64_t used() const noexcept { return m_memory_current - m_memory_start; }

private:
  uint8_t* m_memory_start = nullptr;
  uint8_t* m_memory_current = nullptr;
  uint8_t* m_memory_end = nullptr;
};

} // namespace cosm

#endif // COSM_ALLOCATOR_H_