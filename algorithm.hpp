#pragma once

#include <algorithm>
#ifndef _ALGORITHM_HPP
#define _ALGORITHM_HPP

#include <array>
#include <vector>

#include "node.hpp"

// #include <eigen3/Eigen/Core>

// 离散分布，interval所有可能的选择的集合， locale当前位置， pheromates信息素，dists距离矩阵
// unsigned int discrete_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const Eigen::MatrixXf& pheromates, const Eigen::MatrixXf& dists);
// 约束分布，interval所有可能的选择的集合, locale当前位置, _timenow当前时间， dists距离矩阵
// unsigned int constrain_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const unsigned int _timenow, const Eigen::MatrixXf& dists);

/// @brief 为搜索提供的一些算法
namespace ALG {

	/**
	 * 将范围[first, last)中的元素旋转k个位置。
	 *
	 * @tparam _FwdIt 前向迭代器类型。
	 * @param _first 范围中第一个元素的迭代器。
	 * @param _last 范围中最后一个元素的迭代器。
	 * @param k 旋转元素的位置数。如果为正数，则向右旋转元素。如果为负数，则向左旋转元素。
	 */
	template <class _FwdIt>
	inline void rotate(_FwdIt _first, _FwdIt _last, int k) {
		if (_last <= _first + 1 || k == 0) return;
		if (k > 0) {  // 向右旋转
			_FwdIt _mid{_last - k};
			std::reverse(_first, _mid);
			std::reverse(_mid + 1, _last);
			std::reverse(_first, _last);
		} else {  // 向左旋转
			_FwdIt _mid{_first - k};
			std::reverse(_first, _mid - 1);
			std::reverse(_mid, _last);
			std::reverse(_first, _last);
		}
	}
}  // namespace ALG

/**
 * @brief
 * @link https://github.com/Reputeless/Xoshiro-cpp/blob/master/XoshiroCpp.hpp @endlink
 */
namespace Xoshiro {

	inline constexpr unsigned long long int DefaultSeed = 0x123456789ULL;

	[[nodiscard]] static constexpr unsigned long long int RotL(const unsigned long long int x, const int s) noexcept {
		return (x << s) | (x >> (64 - s));
	}

	[[nodiscard]] static constexpr unsigned int RotL(const unsigned int x, const int s) noexcept {
		return (x << s) | (x >> (32 - s));
	}
	/**
	 * @brief SplitMix64
	 * Output: 64 bits
	 * Period: 2^64
	 * Footprint: 8 bytes
	 * Original implementation: http://prng.di.unimi.it/splitmix64.c
	 */
	class SplitMix64 {
	  public:
		using state_type = unsigned long long int;
		using result_type = unsigned long long int;

		[[nodiscard]] explicit constexpr SplitMix64(state_type state = DefaultSeed) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			unsigned long long int z = (m_state += 0x9e3779b97f4a7c15);
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
			z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
			return z ^ (z >> 31);
		}

		template <std::size_t N>
		[[nodiscard]] constexpr std::array<unsigned long long int, N> generateSeedSequence() noexcept {
			std::array<unsigned long long int, N> seeds = {};
			for (auto& seed : seeds) {
				seed = operator()();
			}
			return seeds;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const SplitMix64& lhs, const SplitMix64& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const SplitMix64& lhs, const SplitMix64& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};

	/**
	 * @brief xoshiro256**
	 * @return 64 bits
	 * @link http://prng.di.unimi.it/xoshiro256starstar.c @endlink
	 * @version 1.0
	 */
	class Xoshiro256ss {
	  public:
		using state_type = std::array<unsigned long long int, 4>;
		using result_type = unsigned long long int;

		[[nodiscard]] explicit constexpr Xoshiro256ss(unsigned long long int seed = DefaultSeed) noexcept : m_state(SplitMix64{seed}.generateSeedSequence<4>()) {}

		[[nodiscard]] explicit constexpr Xoshiro256ss(state_type state) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			const unsigned long long int result = RotL(m_state[1] * 5, 7) * 9;
			const unsigned long long int t = m_state[1] << 17;
			m_state[2] ^= m_state[0];
			m_state[3] ^= m_state[1];
			m_state[1] ^= m_state[2];
			m_state[0] ^= m_state[3];
			m_state[2] ^= t;
			m_state[3] = RotL(m_state[3], 45);
			return result;
		}

		constexpr void jump() noexcept {
			constexpr unsigned long long int JUMP[] = {0x180ec6d33cfd0aba, 0xd5a61266f0c9392c, 0xa9582618e03fc9aa, 0x39abdc4529b1661c};
			unsigned long long int s0 = 0;
			unsigned long long int s1 = 0;
			unsigned long long int s2 = 0;
			unsigned long long int s3 = 0;

			for (unsigned long long int jump : JUMP) {
				for (int b = 0; b < 64; ++b) {
					if (jump & 0x1ULL << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		constexpr void longJump() noexcept {
			constexpr unsigned long long int LONG_JUMP[] = {0x76e15d3efefdcbbf, 0xc5004e441c522fb3, 0x77710069854ee241, 0x39109bb02acbe635};

			unsigned long long int s0 = 0;
			unsigned long long int s1 = 0;
			unsigned long long int s2 = 0;
			unsigned long long int s3 = 0;

			for (unsigned long long int jump : LONG_JUMP) {
				for (int b = 0; b < 64; ++b) {
					if (jump & 0x1ULL << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const Xoshiro256ss& lhs, const Xoshiro256ss& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const Xoshiro256ss& lhs, const Xoshiro256ss& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};

	/**
	 * xoshiro128**
	 * @return 64 bits
	 * @link http://prng.di.unimi.it/xoshiro128starstar.c @endlink
	 * @version 1.1
	 */
	class Xoshiro128ss {
	  public:
		using state_type = std::array<unsigned int, 4>;
		using result_type = unsigned int;

		[[nodiscard]] explicit constexpr Xoshiro128ss(unsigned long long int seed = DefaultSeed) noexcept : m_state() {
			SplitMix64 splitmix{seed};

			for (auto& state : m_state) {
				state = static_cast<unsigned int>(splitmix());
			}
		}

		[[nodiscard]] explicit constexpr Xoshiro128ss(state_type state) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			const unsigned int result = RotL(m_state[1] * 5, 7) * 9;
			const unsigned int t = m_state[1] << 9;
			m_state[2] ^= m_state[0];
			m_state[3] ^= m_state[1];
			m_state[1] ^= m_state[2];
			m_state[0] ^= m_state[3];
			m_state[2] ^= t;
			m_state[3] = RotL(m_state[3], 11);
			return result;
		}

		constexpr void jump() noexcept {
			constexpr unsigned int JUMP[] = {0x8764000b, 0xf542d2d3, 0x6fa035c3, 0x77f2db5b};

			unsigned int s0 = 0;
			unsigned int s1 = 0;
			unsigned int s2 = 0;
			unsigned int s3 = 0;

			for (unsigned int jump : JUMP) {
				for (int b = 0; b < 32; ++b) {
					if (jump & 0x1U << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		constexpr void longJump() noexcept {
			constexpr unsigned int LONG_JUMP[] = {0xb523952e, 0x0b6f099f, 0xccf5a0ef, 0x1c580662};

			unsigned int s0 = 0;
			unsigned int s1 = 0;
			unsigned int s2 = 0;
			unsigned int s3 = 0;

			for (unsigned int jump : LONG_JUMP) {
				for (int b = 0; b < 32; ++b) {
					if (jump & 0x1U << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const Xoshiro128ss& lhs, const Xoshiro128ss& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const Xoshiro128ss& lhs, const Xoshiro128ss& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};
}  // namespace Xoshiro
#endif