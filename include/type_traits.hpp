#pragma once

#include <utility>
#include <type_traits>

namespace traits {

template<typename T, size_t N, typename Idx = std::make_index_sequence<N>>
struct repeated_tuple;

template<typename T, size_t N, size_t... Is>
struct repeated_tuple<T, N, std::index_sequence<Is...>> {
    using type = std::tuple<std::conditional_t<true, T, std::integral_constant<size_t, Is>>...>;
};

template<typename T, size_t N>
using repeated_tuple_t = typename repeated_tuple<T, N>::type;

template <typename Shape, typename... Args>
constexpr bool is_one_of = (... || std::is_same_v<Shape, Args>);

}  // namespace traits
