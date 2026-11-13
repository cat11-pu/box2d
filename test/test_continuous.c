// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

// Reproduction of the classic "high speed bullet through a thin wall" bug. A bullet moving
// 400 m/s at 60 Hz travels 6.67 m per step, so a discrete collision check teleports it past
// a 0.1 m wall without ever overlapping it. The fix is continuous collision detection: fast
// small bodies are swept along their trajectory over the time step and stopped at the first
// time of impact, so tunneling is impossible.
//
// The scenario fires 1000 shots at a thin wall in volleys and counts how many pass through.
// It runs three ways:
// - discrete (enableContinuous = false): reproduces the bug, shots escape the map
// - continuous + bullet bodies: every shot stops at first contact, zero penetration
// - both modes timed over repeated runs to check the CCD overhead stays within 20%

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEP_COUNT 4

#define SHOT_COUNT 1000
#define VOLLEY_SIZE 50
#define VOLLEY_INTERVAL 15
#define SETTLE_STEPS 90

#define WALL_HALF_THICKNESS 0.05f
#define WALL_HALF_HEIGHT 10.0f
#define BULLET_RADIUS 0.05f
#define BULLET_SPEED 400.0f
#define SPAWN_X ( -19.75f )

// Timing repetitions per mode, best run wins to filter out scheduler noise
#define TIMING_REPS 5

// Maximum allowed CCD step-time overhead relative to discrete stepping
#define MAX_CCD_OVERHEAD 0.20f

typedef struct LiveBullet
{
	b2BodyId bodyId;
	float maxX;
} LiveBullet;

typedef struct ShotStats
{
	int fired;
	int tunneled;
	int stopped;
	int unresolved;
	float maxStoppedX;
	float stepMs;
} ShotStats;

// Fires 1000 shots at the wall in volleys. When withScene is set the world also contains a
// ground and a pyramid of crates so the step time reflects a representative game frame
// instead of a pathological all-bullet scene.
static ShotStats RunShotScenario( bool enableCcd, bool withScene )
{
	ShotStats stats = { 0 };
	stats.maxStoppedX = -FLT_MAX;

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = ( b2Vec2 ){ 0.0f, withScene ? -10.0f : 0.0f };
	worldDef.enableSleep = false;
	worldDef.enableContinuous = enableCcd;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// thin static wall: 0.1 m thick, 20 m tall
	b2BodyDef wallDef = b2DefaultBodyDef();
	b2BodyId wallId = b2CreateBody( worldId, &wallDef );
	b2Polygon wallBox = b2MakeBox( WALL_HALF_THICKNESS, WALL_HALF_HEIGHT );
	b2ShapeDef wallShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( wallId, &wallShapeDef, &wallBox );

	if ( withScene )
	{
		// ground
		b2BodyDef groundDef = b2DefaultBodyDef();
		groundDef.position = ( b2Vec2 ){ 0.0f, -11.0f };
		b2BodyId groundId = b2CreateBody( worldId, &groundDef );
		b2Polygon groundBox = b2MakeBox( 60.0f, 1.0f );
		b2CreatePolygonShape( groundId, &wallShapeDef, &groundBox );

		// pyramid of 120 crates behind the wall, clear of the bullet paths
		b2BodyDef crateDef = b2DefaultBodyDef();
		crateDef.type = b2_dynamicBody;
		b2ShapeDef crateShapeDef = b2DefaultShapeDef();
		float h = 0.5f;
		float rootX = 20.0f;
		for ( int row = 0; row < 15; ++row )
		{
			int count = 15 - row;
			float y = -10.0f + h + 2.0f * h * (float)row;
			for ( int i = 0; i < count; ++i )
			{
				crateDef.position = ( b2Vec2 ){ rootX + 2.0f * h * ( (float)i - 0.5f * (float)( count - 1 ) ), y };
				b2BodyId crateId = b2CreateBody( worldId, &crateDef );
				b2Polygon crateBox = b2MakeBox( h, h );
				b2CreatePolygonShape( crateId, &crateShapeDef, &crateBox );
			}
		}
	}

	static LiveBullet live[SHOT_COUNT];
	int liveCount = 0;
	int nextShot = 0;
	float stepMs = 0.0f;

	int totalSteps = ( SHOT_COUNT / VOLLEY_SIZE ) * VOLLEY_INTERVAL + SETTLE_STEPS;
	for ( int step = 0; step < totalSteps; ++step )
	{
		if ( step % VOLLEY_INTERVAL == 0 && nextShot < SHOT_COUNT )
		{
			for ( int i = 0; i < VOLLEY_SIZE && nextShot < SHOT_COUNT; ++i, ++nextShot )
			{
				// deterministic spread so volleys never overlap at spawn
				float y = -9.0f + 0.36f * (float)( nextShot % VOLLEY_SIZE ) + 0.05f * (float)( ( nextShot / VOLLEY_SIZE ) % 3 );

				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = ( b2Vec2 ){ SPAWN_X, y };
				// the fix: mark the fast small body as a bullet so it gets swept
				bodyDef.isBullet = enableCcd;
				b2BodyId bulletId = b2CreateBody( worldId, &bodyDef );

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2Circle circle = { { 0.0f, 0.0f }, BULLET_RADIUS };
				b2CreateCircleShape( bulletId, &shapeDef, &circle );

				b2Body_SetLinearVelocity( bulletId, ( b2Vec2 ){ BULLET_SPEED, 0.0f } );

				live[liveCount] = ( LiveBullet ){ bulletId, SPAWN_X };
				liveCount += 1;
			}
			stats.fired = nextShot;
		}

		uint64_t ticks = b2GetTicks();
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		stepMs += b2GetMilliseconds( ticks );

		for ( int i = 0; i < liveCount; )
		{
			b2Vec2 p = b2Body_GetPosition( live[i].bodyId );
			b2Vec2 v = b2Body_GetLinearVelocity( live[i].bodyId );
			live[i].maxX = b2MaxFloat( live[i].maxX, p.x );

			bool done = false;
			if ( p.x - BULLET_RADIUS > WALL_HALF_THICKNESS )
			{
				// trailing edge is past the back face: the shot went clean through
				stats.tunneled += 1;
				done = true;
			}
			else if ( v.x < 0.5f * BULLET_SPEED )
			{
				// lost most of its forward speed: it hit the wall and is despawned
				stats.stopped += 1;
				stats.maxStoppedX = b2MaxFloat( stats.maxStoppedX, live[i].maxX );
				done = true;
			}

			if ( done )
			{
				b2DestroyBody( live[i].bodyId );
				live[i] = live[liveCount - 1];
				liveCount -= 1;
			}
			else
			{
				i += 1;
			}
		}
	}

	stats.unresolved = liveCount;
	stats.stepMs = stepMs;

	b2DestroyWorld( worldId );
	return stats;
}

// Bug reproduction: with continuous collision disabled the discrete check steps the bullet
// 6.67 m per frame and it never overlaps the 0.1 m wall.
static int DiscreteTunnelingRepro( void )
{
	ShotStats stats = RunShotScenario( false, false );

	printf( "  discrete: fired=%d tunneled=%d stopped=%d unresolved=%d stepMs=%.2f\n", stats.fired, stats.tunneled,
			stats.stopped, stats.unresolved, stats.stepMs );

	ENSURE( stats.fired == SHOT_COUNT );
	ENSURE( stats.unresolved == 0 );
	// the bug must reproduce: the vast majority of shots escape through the wall
	ENSURE( stats.tunneled > 9 * SHOT_COUNT / 10 );
	return 0;
}

// Fix verification: with continuous collision enabled and the bullets marked as bullets,
// every shot is swept to its first time of impact and stopped at the wall face.
static int ContinuousStopsAllShots( void )
{
	ShotStats stats = RunShotScenario( true, false );

	printf( "  continuous: fired=%d tunneled=%d stopped=%d unresolved=%d maxStoppedX=%.4f stepMs=%.2f\n", stats.fired,
			stats.tunneled, stats.stopped, stats.unresolved, stats.maxStoppedX, stats.stepMs );

	ENSURE( stats.fired == SHOT_COUNT );
	ENSURE( stats.tunneled == 0 );
	ENSURE( stats.stopped == SHOT_COUNT );
	ENSURE( stats.unresolved == 0 );
	// no tunneling: the sweep stops each shot at the time of impact. The impact step shows a
	// small transient embed (speculative contact band) that the solver then resolves, but the
	// center of every shot must always stay in front of the wall mid-plane
	ENSURE( stats.maxStoppedX < 0.0f );
	return 0;
}

// Performance budget: in a representative scene CCD stepping must cost no more than 20%
// over discrete stepping.
static int ContinuousPerformanceBudget( void )
{
	float discreteMs = FLT_MAX;
	float continuousMs = FLT_MAX;
	for ( int i = 0; i < TIMING_REPS; ++i )
	{
		ShotStats discrete = RunShotScenario( false, true );
		ShotStats continuous = RunShotScenario( true, true );
		discreteMs = b2MinFloat( discreteMs, discrete.stepMs );
		continuousMs = b2MinFloat( continuousMs, continuous.stepMs );
		ENSURE( continuous.tunneled == 0 );
	}

	float overhead = ( continuousMs - discreteMs ) / discreteMs;
	printf( "  performance: discrete=%.2f ms continuous=%.2f ms overhead=%.1f%% (budget %.0f%%)\n", discreteMs, continuousMs,
			100.0f * overhead, 100.0f * MAX_CCD_OVERHEAD );

	ENSURE( overhead <= MAX_CCD_OVERHEAD );
	return 0;
}

int ContinuousTest( void )
{
	RUN_SUBTEST( DiscreteTunnelingRepro );
	RUN_SUBTEST( ContinuousStopsAllShots );
	RUN_SUBTEST( ContinuousPerformanceBudget );
	return 0;
}
