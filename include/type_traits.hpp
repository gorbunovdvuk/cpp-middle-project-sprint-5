#pragma once

#include <type_traits>

namespace traits {

template <typename Shape, typename... Args>
constexpr bool is_one_of = (... || std::is_same_v<Shape, Args>);

}  // namespace traits
