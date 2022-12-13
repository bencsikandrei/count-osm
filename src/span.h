#ifndef COSM_SPAN_H__
#define COSM_SPAN_H__

namespace cosm
{

template<typename T, unsigned Size>
struct span_fixed_size
{
  using size_type = decltype(Size);
  using value_type = T;

  constexpr span_fixed_size(T* data) noexcept
    : m_data(data)
  {
  }

  template<unsigned ArrSize>
  constexpr span_fixed_size(T (&arr)[ArrSize])
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

  T* m_data;
};

static_assert(sizeof(span_fixed_size<int, 4>) == sizeof(void*));

} // namespace cosm

#endif // COSM_SPAN_H__