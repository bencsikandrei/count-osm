#ifndef COSM_TRAITS_H__
#define COSM_TRAITS_H__

namespace cosm
{

template<typename T>
struct remove_reference
{
  typedef T type;
};

template<typename T>
struct remove_reference<T&>
{
  typedef T type;
};

template<typename T>
struct remove_reference<T&&>
{
  typedef T type;
};

template<typename T>
using remove_reference_t = typename remove_reference<T>::type;

} // namespace cosm

#endif // COSM_TRAITS_H__
