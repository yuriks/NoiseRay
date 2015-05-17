#pragma once
#include <cstdint>
#include <limits>

namespace yks {

	struct Rng {
		typedef uint64_t result_type;

		uint64_t s[16];
		unsigned int p = 0;

		result_type operator()() {
			// This is the xorshift1024* PRNG
			// http://xorshift.di.unimi.it/
			uint64_t s0 = s[p];
			uint64_t s1 = s[p = (p + 1) % 16];

			s1 ^= s1 << 31;
			s1 ^= s1 >> 11;
			s0 ^= s0 >> 30;
			return (s[p] = s0 ^ s1) * 1181783497276652981LL;
		}

		static result_type min() {
			return std::numeric_limits<result_type>::min();
		}

		static result_type max() {
			return std::numeric_limits<result_type>::max();
		}

		void seed_with_default() {
			s[0] = 0xd00b2f639592faf0ULL;
			s[1] = 0x1b748bead91f729bULL;
			s[2] = 0xf80b426766fb00acULL;
			s[3] = 0x1773f0bd9fe62d72ULL;
			s[4] = 0xfa3824c82db94c37ULL;
			s[5] = 0xad6d6f013d1d50b8ULL;
			s[6] = 0x6d07ad8224ae4327ULL;
			s[7] = 0x5d58864e8bfc315dULL;
			s[8] = 0xa9a16f6ea16d0bf0ULL;
			s[9] = 0x51b453d66acc9d37ULL;
			s[10] = 0x956ef667add1bda2ULL;
			s[11] = 0xaf5aca0d291a915aULL;
			s[12] = 0xfa3c060d47b87d8dULL;
			s[13] = 0x750b2ddb01452e91ULL;
			s[14] = 0x99d372e94f4d7619ULL;
			s[15] = 0x3d00c7f84ed77ebcULL;
		}

		void seed_with_rng(Rng& rng) {
			for (uint64_t& x : s) {
				x = rng();
			}
		}

		float canonical() {
			const result_type mask = 0x7fffffULL;
			const float reciprocal = (1.0f / 0x800000ULL);
			// This is int64_t because it generates more compact int -> float conversion code than unsigned.
			int64_t mantissa = (*this)() & mask;
			return (float)mantissa * reciprocal;
		}
	};

}
