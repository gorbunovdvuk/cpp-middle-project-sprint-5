#pragma once

#include <range/v3/all.hpp>

namespace combinations {

template<std::ranges::range R>
auto combinations(R&& range) {
    return ranges::views::cartesian_product(ranges::views::enumerate(range), ranges::views::enumerate(range)) |
        ranges::views::filter([](const auto& t) {
            return std::get<0>(t).first < std::get<1>(t).first;
        }) | ranges::views::transform([](const auto& t) {
            return std::make_tuple(std::get<0>(t).second, std::get<1>(t).second);
        });
}

}  // namespace combinations
