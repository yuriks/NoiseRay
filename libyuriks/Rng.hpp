#pragma once
#include <cstdint>
#include <limits>

namespace yks {

	struct Rng {
		typedef uint64_t result_type;

		uint64_t s[2];

		result_type operator()() {
			// This is the xorshift128+ PRNG
			// http://xorshift.di.unimi.it/
			uint64_t s1 = s[0];
			const uint64_t s0 = s[1];
			s[0] = s0;
			s1 ^= s1 << 23; // a
			return (s[1] = (s1 ^ s0 ^ (s1 >> 17) ^ (s0 >> 26))) + s0; // b, c
		}

		static result_type min() {
			return std::numeric_limits<result_type>::min();
		}

		static result_type max() {
			return std::numeric_limits<result_type>::max();
		}

		void seed_with_default() {
			s[0] = 0x4587ba0ead01370fULL;
			s[1] = 0xdd817882dc98c4aaULL;
		}

		void seed_with_rng(Rng& rng) {
			s[0] = rng();
			s[1] = rng();
		}

		float canonical() {
			return (*this)() * (1.0f / max());
		}
	};

}
