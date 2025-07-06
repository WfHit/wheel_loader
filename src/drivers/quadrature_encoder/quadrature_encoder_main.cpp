/**
 * @file quadrature_encoder_main.cpp
 *
 * Main entry point for GPIO-based quadrature encoder driver
 */

#include "quadrature_encoder.hpp"

extern "C" __EXPORT int quadrature_encoder_main(int argc, char *argv[]);

int quadrature_encoder_main(int argc, char *argv[])
{
	return QuadratureEncoder::main(argc, argv);
}
