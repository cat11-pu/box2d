// Deterministic 10k rigid-body broad-phase stress benchmark.
// Emits per-step profile timings plus contact/state hashes so two builds can be
// compared for both speed and exact result consistency.
#include "box2d/box2d.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GRID 100
#define BODY_COUNT ( GRID * GRID )
#define STEP_COUNT 300
#define SPACING 0.5f
#define HALF 0.225f
#define STATIC_SIDE 60
#define STATIC_COUNT ( STATIC_SIDE * STATIC_SIDE )

static uint32_t HashBytes( uint32_t hash, const void* data, int count )
{
	return b2Hash( hash, (const uint8_t*)data, count );
}

static void BuildWorld( b2WorldId worldId, b2BodyId* bodyIds, bool manyStatic )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	float width = GRID * SPACING;

	if ( manyStatic )
	{
		// A dense static tile terrain: thousands of separate static proxies, the target for
		// bulk insertion. Each tile is its own shape on one static body.
		b2Polygon tile = b2MakeBox( 0.5f * SPACING, 0.5f * SPACING );
		for ( int row = 0; row < STATIC_SIDE; ++row )
		{
			for ( int col = 0; col < STATIC_SIDE; ++col )
			{
				b2Vec2 pos = {
					.x = ( col - 0.5f * ( STATIC_SIDE - 1 ) ) * SPACING,
					.y = -2.0f - row * SPACING,
				};
				b2Polygon placed = b2MakeOffsetBox( 0.5f * SPACING, 0.5f * SPACING, pos, b2Rot_identity );
				(void)tile;
				b2CreatePolygonShape( groundId, &shapeDef, &placed );
			}
		}
	}
	else
	{
		b2Polygon ground = b2MakeBox( 0.5f * width + 2.0f, 1.0f );
		b2ShapeId groundShape = b2CreatePolygonShape( groundId, &shapeDef, &ground );
		(void)groundShape;
	}

	b2Polygon wallLeft = b2MakeOffsetBox( 1.0f, 40.0f, (b2Vec2){ -0.5f * width - 1.0f, 40.0f }, b2Rot_identity );
	b2Polygon wallRight = b2MakeOffsetBox( 1.0f, 40.0f, (b2Vec2){ 0.5f * width + 1.0f, 40.0f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &wallLeft );
	b2CreatePolygonShape( groundId, &shapeDef, &wallRight );

	b2Polygon box = b2MakeBox( HALF, HALF );

	int index = 0;
	for ( int row = 0; row < GRID; ++row )
	{
		for ( int col = 0; col < GRID; ++col )
		{
			b2BodyDef def = b2DefaultBodyDef();
			def.type = b2_dynamicBody;
			def.enableSleep = false;
			def.position.x = ( col - 0.5f * ( GRID - 1 ) ) * SPACING;
			def.position.y = 2.0f + row * SPACING;
			b2BodyId bodyId = b2CreateBody( worldId, &def );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
			bodyIds[index++] = bodyId;
		}
	}
}

int main( int argc, char** argv )
{
	int workers = 1;
	int repeats = 1;
	bool manyStatic = false;
	for ( int i = 1; i < argc; ++i )
	{
		if ( strncmp( argv[i], "-w=", 3 ) == 0 )
		{
			workers = atoi( argv[i] + 3 );
		}
		else if ( strncmp( argv[i], "-r=", 3 ) == 0 )
		{
			repeats = atoi( argv[i] + 3 );
		}
		else if ( strcmp( argv[i], "-s" ) == 0 )
		{
			manyStatic = true;
		}
	}

	// Per-step minimum across repeats.
	float minStep[STEP_COUNT];
	float minPairs[STEP_COUNT];
	float minCollide[STEP_COUNT];
	float minSolve[STEP_COUNT];
	float minRefit[STEP_COUNT];
	float minTransforms[STEP_COUNT];
	for ( int i = 0; i < STEP_COUNT; ++i )
	{
		minStep[i] = minPairs[i] = minCollide[i] = minSolve[i] = minRefit[i] = minTransforms[i] = 1e30f;
	}

	uint32_t contactHash = 0;
	uint32_t stateHash = 0;
	int contactCount = 0;
	int touchingCount = 0;

	b2BodyId* bodyIds = malloc( BODY_COUNT * sizeof( b2BodyId ) );

	for ( int run = 0; run < repeats; ++run )
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		worldDef.workerCount = workers;
		worldDef.enableSleep = false;
		worldDef.capacity.dynamicBodyCount = BODY_COUNT + 16;
		worldDef.capacity.dynamicShapeCount = BODY_COUNT + 16;
		worldDef.capacity.staticShapeCount = manyStatic ? STATIC_COUNT + 8 : 8;
		worldDef.capacity.contactCount = BODY_COUNT * 3;

		b2WorldId worldId = b2CreateWorld( &worldDef );
		BuildWorld( worldId, bodyIds, manyStatic );

		for ( int step = 0; step < STEP_COUNT; ++step )
		{
			b2World_Step( worldId, 1.0f / 60.0f, 4 );
			b2Profile p = b2World_GetProfile( worldId );
			if ( p.step < minStep[step] ) minStep[step] = p.step;
			if ( p.pairs < minPairs[step] ) minPairs[step] = p.pairs;
			if ( p.collide < minCollide[step] ) minCollide[step] = p.collide;
			if ( p.solve < minSolve[step] ) minSolve[step] = p.solve;
			if ( p.refit < minRefit[step] ) minRefit[step] = p.refit;
			if ( p.transforms < minTransforms[step] ) minTransforms[step] = p.transforms;
		}

		// Determinism hashes are run-independent; compute on the last run.
		b2Counters counters = b2World_GetCounters( worldId );
		contactCount = counters.contactCount;
		touchingCount = counters.awakeContactCount;

		uint32_t ch = B2_HASH_INIT;
		ch = HashBytes( ch, &counters.contactCount, sizeof( int ) );
		ch = HashBytes( ch, &counters.awakeContactCount, sizeof( int ) );

		uint32_t sh = B2_HASH_INIT;
		for ( int i = 0; i < BODY_COUNT; ++i )
		{
			b2WorldTransform t = b2Body_GetTransform( bodyIds[i] );
			sh = HashBytes( sh, &t, sizeof( b2WorldTransform ) );
		}
		contactHash = ch;
		stateHash = sh;

		b2DestroyWorld( worldId );
	}

	printf( "e2e workers=%d steps=%d bodies=%d static=%d\n", workers, STEP_COUNT, BODY_COUNT,
			manyStatic ? STATIC_COUNT + 3 : 3 );
	printf( "step,step_ms,pairs_ms,collide_ms,solve_ms,refit_ms,transforms_ms,contact_count,touching_count,contact_hash,state_hash\n" );
	for ( int step = 0; step < STEP_COUNT; ++step )
	{
		printf( "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%u,%u\n", step, minStep[step], minPairs[step], minCollide[step],
				minSolve[step], minRefit[step], minTransforms[step], contactCount, touchingCount, contactHash, stateHash );
	}

	free( bodyIds );
	return 0;
}
