#pragma once
#include <rusefi/interpolation.h>

template <typename D, typename S, size_t N, size_t M>
void copyTable(D (&dest)[N][M], const S (&src)[N][M]) {
	for (size_t r = 0; r < N; ++r)
		for (size_t c = 0; c < M; ++c)
			dest[r][c] = src[r][c];
}

template <typename D, typename V, size_t N, size_t M>
void setTable(D (&dest)[N][M], V value) {
	for (auto& row : dest)
		for (auto& cell : row)
			cell = value;
}
