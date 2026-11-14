// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Long-run bitwise determinism test:
// - 1000 dynamic rigid bodies, 10000 frames, fixed seed
// - two independent runs compared frame by frame (position + rotation, bitwise)
// - pause/resume leaves the state untouched
// - snapshot/restore round trip preserves the state and the continued trajectory
// - SSE2 fallback (SIMD width 4) matches the native AVX2 path frame by frame
// - every frame hash is appended to determinism_10k_hashes.bin and folded into a
//   single golden digest, so a run on another machine or another optimization
//   level only has to match one uint64 (or diff the hash file)

#define D10K_BODY_COUNT 1000
#define D10K_FRAME_COUNT 10000
#define D10K_PAUSE_FRAME 5000
#define D10K_AGITATOR_COUNT 64
#define D10K_SEED UINT64_C( 0x2F5E8B1A9C4D7E03 )
#define D10K_HASH_FILE "determinism_10k_hashes.bin"

// splitmix64: integer-only PRNG, identical results on every platform
static uint64_t s_rngState;

static void SeedRng( uint64_t seed )
{
	s_rngState = seed;
}

static uint64_t NextU64( void )
{
	s_rngState += UINT64_C( 0x9E3779B97F4A7C15 );
	uint64_t z = s_rngState;
	z = ( z ^ ( z >> 30 ) ) * UINT64_C( 0xBF58476D1CE4E5B9 );
	z = ( z ^ ( z >> 27 ) ) * UINT64_C( 0x94D049BB133111EB );
	return z ^ ( z >> 31 );
}

static float NextFloat( float lo, float hi )
{
	uint64_t r = NextU64() >> 40;
	float t = (float)r * ( 1.0f / 16777216.0f );
	return lo + ( hi - lo ) * t;
}

static uint64_t MixHash( uint64_t digest, uint64_t hash )
{
	return digest ^ ( hash + UINT64_C( 0x9E3779B97F4A7C15 ) + ( digest << 6 ) + ( digest >> 2 ) );
}

// Builds the 1000-body scene. The PRNG draw order is fully determined by the seed,
// so two calls with the same seed produce bit-identical worlds.
static b2WorldId BuildScene( uint64_t seed, bool sse2Fallback, b2BodyId* outBodies )
{
	SeedRng( seed );

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 0;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2World_EnableSSE2Fallback( worldId, sse2Fallback );

	// Closed container so the agitator bodies stay in play for all 10000 frames
	{
		b2BodyDef groundDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &groundDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.6f;

		b2Segment floor = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2Segment leftWall = { { -40.0f, 0.0f }, { -40.0f, 70.0f } };
		b2Segment rightWall = { { 40.0f, 0.0f }, { 40.0f, 70.0f } };
		b2Segment ceiling = { { -40.0f, 70.0f }, { 40.0f, 70.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &floor );
		b2CreateSegmentShape( groundId, &shapeDef, &leftWall );
		b2CreateSegmentShape( groundId, &shapeDef, &rightWall );
		b2CreateSegmentShape( groundId, &shapeDef, &ceiling );
	}

	b2Polygon box = b2MakeBox( 0.45f, 0.45f );
	b2Circle circle = { { 0.0f, 0.0f }, 0.45f };

	for ( int i = 0; i < D10K_BODY_COUNT; ++i )
	{
		int column = i % 40;
		int row = i / 40;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -31.5f + 1.4f * (float)column + NextFloat( -0.15f, 0.15f ),
									2.0f + 1.4f * (float)row + NextFloat( -0.15f, 0.15f ) };
		bodyDef.rotation = b2MakeRot( NextFloat( -B2_PI, B2_PI ) );

		// A fixed subset never sleeps and keeps the solver active for the whole run
		if ( i < D10K_AGITATOR_COUNT )
		{
			bodyDef.enableSleep = false;
			bodyDef.linearVelocity = (b2Vec2){ NextFloat( -6.0f, 6.0f ), NextFloat( -6.0f, 6.0f ) };
			bodyDef.angularVelocity = NextFloat( -4.0f, 4.0f );
		}

		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = NextFloat( 0.0f, 0.5f );
		shapeDef.material.friction = NextFloat( 0.1f, 0.8f );

		if ( ( NextU64() & 1 ) != 0 )
		{
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
		else
		{
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		outBodies[i] = bodyId;
	}

	return worldId;
}

// Bitwise pose comparison: position and rotation (cos/sin) must be exact
static bool PoseEquals( b2BodyId bodyA, b2BodyId bodyB )
{
	b2Vec2 posA = b2Body_GetPosition( bodyA );
	b2Vec2 posB = b2Body_GetPosition( bodyB );
	b2Rot rotA = b2Body_GetRotation( bodyA );
	b2Rot rotB = b2Body_GetRotation( bodyB );
	return memcmp( &posA, &posB, sizeof( posA ) ) == 0 && memcmp( &rotA, &rotB, sizeof( rotA ) ) == 0;
}

int Determinism10kTest( void )
{
	const float timeStep = 1.0f / 60.0f;
	const int subSteps = 4;

	static b2BodyId bodies1[D10K_BODY_COUNT];
	static b2BodyId bodies2[D10K_BODY_COUNT];
	static b2BodyId bodies4[D10K_BODY_COUNT];

	// Run 1 and run 2: identical seed, identical construction
	b2WorldId world1 = BuildScene( D10K_SEED, false, bodies1 );
	b2WorldId world2 = BuildScene( D10K_SEED, false, bodies2 );

	// Run 4: same scene on the width-4 SSE2 path, standing in for a different CPU
	bool checkSimdWidth = b2IsAVX2Available();
	b2WorldId world4 = b2_nullWorldId;
	if ( checkSimdWidth )
	{
		world4 = BuildScene( D10K_SEED, true, bodies4 );
	}
	else
	{
		printf( "  AVX2 not available, SSE2 fallback comparison skipped\n" );
	}

	b2WorldId world3 = b2_nullWorldId;
	uint8_t* image1 = NULL;
	uint64_t firstFrameHash = 0;
	uint64_t digest = UINT64_C( 0xCBF29CE484222325 );

	FILE* hashFile = fopen( D10K_HASH_FILE, "wb" );
	ENSURE( hashFile != NULL );

	for ( int frame = 0; frame < D10K_FRAME_COUNT; ++frame )
	{
		b2World_Step( world1, timeStep, subSteps );
		b2World_Step( world2, timeStep, subSteps );
		if ( checkSimdWidth )
		{
			b2World_Step( world4, timeStep, subSteps );
		}
		if ( b2World_IsValid( world3 ) )
		{
			b2World_Step( world3, timeStep, subSteps );
		}

		uint64_t hash1 = b2World_GetStateHash( world1 );
		if ( frame == 0 )
		{
			firstFrameHash = hash1;
		}

		// Two runs: engine-level state hash must match every frame
		uint64_t hash2 = b2World_GetStateHash( world2 );
		if ( hash1 != hash2 )
		{
			printf( "  run divergence at frame %d: hash1=0x%016llX hash2=0x%016llX\n", frame, (unsigned long long)hash1,
					(unsigned long long)hash2 );
			return 1;
		}

		// Two runs: per-body position and rotation, bitwise, every frame
		for ( int i = 0; i < D10K_BODY_COUNT; ++i )
		{
			if ( PoseEquals( bodies1[i], bodies2[i] ) == false )
			{
				b2Vec2 p1 = b2Body_GetPosition( bodies1[i] );
				b2Vec2 p2 = b2Body_GetPosition( bodies2[i] );
				printf( "  pose divergence at frame %d body %d: (%a, %a) vs (%a, %a)\n", frame, i, p1.x, p1.y, p2.x,
						p2.y );
				return 1;
			}
		}

		// SIMD width 4 vs native width, every frame
		if ( checkSimdWidth && b2World_GetStateHash( world4 ) != hash1 )
		{
			printf( "  SSE2/AVX2 divergence at frame %d\n", frame );
			return 1;
		}

		// World restored from the mid-run snapshot must track the live runs
		if ( b2World_IsValid( world3 ) && b2World_GetStateHash( world3 ) != hash1 )
		{
			printf( "  restored-world divergence at frame %d\n", frame );
			return 1;
		}

		digest = MixHash( digest, hash1 );
		fwrite( &hash1, sizeof( hash1 ), 1, hashFile );

		if ( frame == D10K_PAUSE_FRAME )
		{
			// Pause: wall-clock time passes, no Step is issued, state must not move
			uint64_t pauseHash = hash1;
			uint64_t pauseTicks = b2GetTicks();
			while ( b2GetMilliseconds( pauseTicks ) < 20.0f )
			{
			}
			ENSURE( b2World_GetStateHash( world1 ) == pauseHash );

			// Serialization is deterministic: two snapshots of one state are byte identical
			int imageSize = b2World_GetSnapshot( world1, NULL, 0 );
			ENSURE( imageSize > 0 );
			image1 = (uint8_t*)malloc( (size_t)imageSize );
			uint8_t* image2 = (uint8_t*)malloc( (size_t)imageSize );
			ENSURE( image1 != NULL && image2 != NULL );
			ENSURE( b2World_GetSnapshot( world1, image1, imageSize ) == imageSize );
			ENSURE( b2World_GetSnapshot( world1, image2, imageSize ) == imageSize );
			ENSURE( memcmp( image1, image2, (size_t)imageSize ) == 0 );
			free( image2 );

			// The second independent run serializes to the same bytes
			uint8_t* imageB = (uint8_t*)malloc( (size_t)imageSize );
			ENSURE( imageB != NULL );
			ENSURE( b2World_GetSnapshot( world2, NULL, 0 ) == imageSize );
			ENSURE( b2World_GetSnapshot( world2, imageB, imageSize ) == imageSize );
			ENSURE( memcmp( image1, imageB, (size_t)imageSize ) == 0 );
			free( imageB );

			// Deserialize round trip in place: state is unchanged
			ENSURE( b2World_RestoreFromSnapshot( world1, image1, imageSize ) );
			ENSURE( b2World_GetStateHash( world1 ) == pauseHash );

			// Resume from the snapshot in a fresh world (checked per frame above)
			world3 = b2CreateWorldFromSnapshot( image1, imageSize, 0 );
			ENSURE( b2World_IsValid( world3 ) );
			ENSURE( b2World_GetStateHash( world3 ) == pauseHash );

			printf( "  frame %d: pause/resume ok, snapshot %d bytes, restore ok\n", frame, imageSize );
		}

		if ( ( frame + 1 ) % 1000 == 0 )
		{
			printf( "  frame %5d/%d  hash=0x%016llX\n", frame + 1, D10K_FRAME_COUNT, (unsigned long long)hash1 );
		}
	}

	fclose( hashFile );

	// The restored world ends in the exact same poses as the uninterrupted runs.
	// A snapshot preserves the id pool, so each body keeps its index/generation and
	// only the world slot changes.
	for ( int i = 0; i < D10K_BODY_COUNT; ++i )
	{
		b2BodyId body3 = bodies1[i];
		body3.world0 = (uint16_t)( world3.index1 - 1 );
		if ( PoseEquals( bodies1[i], body3 ) == false )
		{
			printf( "  final pose mismatch on body %d (live vs restored)\n", i );
			return 1;
		}
	}

	// Sanity: a different seed must produce a different trajectory, otherwise the
	// comparison above would be vacuous
	{
		static b2BodyId otherBodies[D10K_BODY_COUNT];
		b2WorldId otherWorld = BuildScene( D10K_SEED + 1, false, otherBodies );
		b2World_Step( otherWorld, timeStep, subSteps );
		ENSURE( b2World_GetStateHash( otherWorld ) != firstFrameHash );
		b2DestroyWorld( otherWorld );
	}

	b2DestroyWorld( world1 );
	b2DestroyWorld( world2 );
	if ( b2World_IsValid( world3 ) )
	{
		b2DestroyWorld( world3 );
	}
	if ( b2World_IsValid( world4 ) )
	{
		b2DestroyWorld( world4 );
	}
	free( image1 );

	printf( "  %d bodies x %d frames compared bitwise (position + rotation)\n", D10K_BODY_COUNT, D10K_FRAME_COUNT );
	printf( "  golden digest = 0x%016llX\n", (unsigned long long)digest );
	printf( "  per-frame hashes written to %s\n", D10K_HASH_FILE );

	return 0;
}
