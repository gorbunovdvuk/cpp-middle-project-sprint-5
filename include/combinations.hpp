#pragma once

#include "type_traits.hpp"
#include <algorithm>
#include <range/v3/all.hpp>
#include <boost/stl_interfaces/iterator_interface.hpp>
#include <numeric>
#include <ranges>

namespace combinations {

template<std::ranges::range R, size_t N>
requires (std::ranges::sized_range<R> && N > 0)
class combinations_view : public ranges::view_interface<combinations_view<R, N>> {
public:
    using range_element_type = std::ranges::range_reference_t<R>;
    using value_type = traits::repeated_tuple_t<range_element_type, N>;

    template<typename base_type = boost::stl_interfaces::proxy_iterator_interface<std::forward_iterator_tag, value_type>>
    class iterator : public base_type {
    public:
        iterator(): parent_(nullptr), all_done_(true) {}
        explicit iterator(combinations_view* parent): parent_(parent) {
            std::iota(indices_.begin(), indices_.end(), 0);
        }

        value_type operator*() const {
            return [this]<size_t... Is>(std::index_sequence<Is...>) {
                return std::tie(parent_->range_[indices_[Is]]...);
            }(std::make_index_sequence<N>{});
        }

        iterator& operator++() {
            if (all_done_) {
                throw std::logic_error("Cannot increment past end");
            }
            size_t size = parent_->range_.size();
            for (size_t i = N; i-- > 0;) {
                if (indices_[i] + (N - i) < size) {
                    indices_[i]++;
                    for (size_t j = i + 1; j < N; j++) {
                        indices_[j] = indices_[j - 1] + 1;
                    }
                    return *this;
                }
            }
            all_done_ = true;
            return *this;
        }

        friend bool operator==(const iterator& it1, const iterator& it2) {
            return it1.parent_ == it2.parent_ && it1.all_done_ == it2.all_done_ && it1.indices_ == it2.indices_;
        }

        friend bool operator==(const iterator& it, std::default_sentinel_t) {
            return it.all_done_;
        }

        using base_type::operator++;

    private:
        combinations_view* parent_;
        std::array<int, N> indices_;
        bool all_done_ = false;
    };

    combinations_view() = default;
    explicit combinations_view(R range): range_(std::move(range)) {}

    iterator<> begin() { return iterator<>(this); }
    std::default_sentinel_t end() { return std::default_sentinel; }

private:
    R range_;
};

template<size_t N>
struct make_combinations {
    template<std::ranges::viewable_range R>
    auto operator()(R&& range) const {
        return combinations_view<std::decay_t<R>, N>(std::views::all(std::forward<R>(range)));
    }
};

template<size_t N>
inline constexpr make_combinations<N> combinations{};

}  // namespace combinations
