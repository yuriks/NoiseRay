#pragma once
#include <cstdint>
#include <random>

struct Rng {
	std::mt19937 engine;
	std::uniform_real_distribution<float> canonical_distribution; // std::generate_canonical is broken in VS2013

	Rng() {
		static const uint32_t seed_seq[] = {
			0x4587ba0e, 0xad01370f, 0xdd817882, 0xdc98c4aa,
			0x4cbf0235, 0x7dba82eb, 0xea593627, 0x597e5052
		};
		std::seed_seq seq(std::begin(seed_seq), std::end(seed_seq));
		engine.seed(seq);
	}

	float canonical() {
		return canonical_distribution(engine);
	}
};