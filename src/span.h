#ifndef COSM_SPAN_H__
#define COSM_SPAN_H__

namespace cosm
{

template<typename T, unsigned Size>
struct span_fixed_size
{
  using size_type = decltype(Size);
  using value_type = T;

  constexpr span_fixed_size() = default;

  constexpr span_fixed_size(T* data) noexcept
    : m_data(data)
  {
  }

  template<unsigned ArrSize>
  constexpr span_fixed_size(T (&arr)[ArrSize]) noexcept
    : m_data(arr)
  {
    static_assert(ArrSize >= Size, "The array must be at least Size elements");
  }

  constexpr span_fixed_size(const span_fixed_size&) noexcept = default;
  constexpr span_fixed_size& operator=(const span_fixed_size&) noexcept =
    default;
  constexpr span_fixed_size(span_fixed_size&&) noexcept = default;
  constexpr span_fixed_size& operator=(span_fixed_size&&) noexcept = default;

  constexpr T& operator[](unsigned index) noexcept { return *(m_data + index); }
  constexpr const T& operator[](unsigned index) const noexcept
  {
    return *(m_data + index);
  }

  constexpr size_type size() const noexcept { return Size; }

  constexpr const T* data() const noexcept { return m_data; }
  T* data() noexcept { return m_data; }

  T* begin() noexcept { return m_data; }
  T* end() noexcept { return m_data + size(); }

  const T* begin() const noexcept { return m_data; }
  const T* end() const noexcept { return m_data + size(); }

  T* m_data = nullptr;
};

static_assert(sizeof(span_fixed_size<int, 4>) == sizeof(void*));

template<typename T>
struct span
{
  using size_type = unsigned long long;
  using value_type = T;

  constexpr span() = default;

  constexpr span(T* data, size_type size) noexcept
    : m_data(data)
    , m_size(size)
  {
  }

  template<unsigned ArrSize>
  constexpr span(T (&arr)[ArrSize]) noexcept
    : m_data(arr)
    , m_size(ArrSize)
  {
  }

  constexpr span(const span&) noexcept = default;
  constexpr span& operator=(const span&) noexcept = default;
  constexpr span(span&&) noexcept = default;
  constexpr span& operator=(span&&) noexcept = default;

  constexpr T& operator[](unsigned index) noexcept { return *(m_data + index); }
  constexpr const T& operator[](unsigned index) const noexcept
  {
    return *(m_data + index);
  }

  constexpr size_type size() const noexcept { return m_size; }

  constexpr const T* data() const noexcept { return m_data; }
  T* data() noexcept { return m_data; }

  T* begin() noexcept { return m_data; }
  T* end() noexcept { return m_data + size(); }

  const T* begin() const noexcept { return m_data; }
  const T* end() const noexcept { return m_data + size(); }

  T* m_data = nullptr;
  size_type m_size = 0ul;
};

static_assert(sizeof(span<int>) == 2 * sizeof(void*));

} // namespace cosm

#endif // COSM_SPAN_H__