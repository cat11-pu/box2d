// Deterministic micro-benchmark for the b2DynamicTree primitives that dominate the broad phase.
// The same RNG stream is used for every build, so result hashes must match bit-for-bit.
#include "dynamic_tree.h"

#include "box2d/base.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PROXY_COUNT 10000
#define QUERY_COUNT 2000
#define WORLD_EXTENT 100.0f
#define BOX_SIZE 0.45f

typedef struct Rng
{
	uint32_t state;
} Rng;

static uint32_t RngNext( Rng* rng )
{
	// xorshift32, fully deterministic across builds
	uint32_t x = rng->state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng->state = x;
	return x;
}

static float RngFloat( Rng* rng, float lo, float hi )
{
	float u = (float)( RngNext( rng ) & 0xFFFFFF ) / (float)0x1000000;
	return lo + u * ( hi - lo );
}

static uint64_t NowMs( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

typedef struct QueryResult
{
	int hits;
	uint32_t hash;
	int64_t nodeVisits;
	int64_t leafVisits;
	int* hitIds;
	int hitCap;
} QueryResult;

static bool QueryCallback( int proxyId, uint64_t userData, void* context )
{
	QueryResult* result = context;
	result->hits += 1;
	if ( result->hits > result->hitCap )
	{
		result->hitCap = result->hitCap == 0 ? 1024 : result->hitCap * 2;
		result->hitIds = realloc( result->hitIds, (size_t)result->hitCap * sizeof( int ) );
	}
	result->hitIds[result->hits - 1] = proxyId;
	uint32_t h = (uint32_t)userData;
	h = ( h << 5 ) - h + (uint32_t)proxyId;
	result->hash ^= h + 0x9e3779b9u + ( result->hash << 6 ) + ( result->hash >> 2 );
	return true;
}

static int IntCompare( const void* a, const void* b )
{
	int ia = *(const int*)a;
	int ib = *(const int*)b;
	return ( ia > ib ) - ( ia < ib );
}

int main( int argc, char** argv )
{
	bool bulk = argc > 1;

	// Deterministic AABB stream: clustered boxes, like a pile of rigid bodies on a floor.
	b2AABB* boxes = malloc( PROXY_COUNT * sizeof( b2AABB ) );
	Rng rng = { 0x12345678u };
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		float x = RngFloat( &rng, -WORLD_EXTENT, WORLD_EXTENT );
		float y = RngFloat( &rng, -WORLD_EXTENT, WORLD_EXTENT );
		boxes[i].lowerBound.x = x;
		boxes[i].lowerBound.y = y;
		boxes[i].upperBound.x = x + BOX_SIZE;
		boxes[i].upperBound.y = y + BOX_SIZE;
	}

	b2DynamicTree tree;
	b2CreateDynamicTreeInternal( &tree, PROXY_COUNT );

	uint64_t t0 = NowMs();
	if ( bulk )
	{
#ifdef NO_BULK
		// Baseline library has no bulk entry point; fall back to serial so the same harness runs.
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			b2CreateTreeProxyInternal( &tree, boxes[i], 1, (uint64_t)i, false );
		}
#else
		int* proxies = malloc( PROXY_COUNT * sizeof( int ) );
		b2DynamicTree_BulkCreate( &tree, boxes, PROXY_COUNT, proxies );
		free( proxies );
#endif
	}
	else
	{
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			b2CreateTreeProxyInternal( &tree, boxes[i], 1, (uint64_t)i, false );
		}
	}
	uint64_t t1 = NowMs();

	printf( "tree_insert bulk=%d insert_ms=%.4f height=%d area_ratio=%.4f proxy_count=%d\n", bulk,
			( t1 - t0 ) / 1e6, b2DynamicTree_GetHeight( &tree ), b2DynamicTree_GetAreaRatio( &tree ),
			b2DynamicTree_GetProxyCount( &tree ) );

	// Deterministic query stream.
	QueryResult total = { 0 };
	int totalHitCount = 0;
	t0 = NowMs();
	Rng qrng = { 0xABCDEF01u };
	for ( int q = 0; q < QUERY_COUNT; ++q )
	{
		float x = RngFloat( &qrng, -WORLD_EXTENT, WORLD_EXTENT );
		float y = RngFloat( &qrng, -WORLD_EXTENT, WORLD_EXTENT );
		b2AABB query = {
			.lowerBound = { x, y },
			.upperBound = { x + 4.0f, y + 4.0f },
		};
		QueryResult result = { 0 };
		b2TreeStats stats = b2DynamicTree_OverlapAABB( &tree, query, 1, QueryCallback, &result );
		total.hash ^= result.hash + 0x9e3779b9u + ( total.hash << 6 ) + ( total.hash >> 2 );
		total.nodeVisits += stats.nodeVisits;
		total.leafVisits += stats.leafVisits;
		if ( totalHitCount + result.hits > total.hitCap )
		{
			while ( totalHitCount + result.hits > total.hitCap )
			{
				total.hitCap = total.hitCap == 0 ? 1024 : total.hitCap * 2;
			}
			total.hitIds = realloc( total.hitIds, (size_t)total.hitCap * sizeof( int ) );
		}
		memcpy( total.hitIds + totalHitCount, result.hitIds, (size_t)result.hits * sizeof( int ) );
		totalHitCount += result.hits;
		free( result.hitIds );
	}
	t1 = NowMs();
	total.hits = totalHitCount;
	printf( "tree_query,query_ms=%.4f node_visits=%lld leaf_visits=%lld hits=%d hash=%u\n", ( t1 - t0 ) / 1e6,
			( long long )total.nodeVisits, ( long long )total.leafVisits, totalHitCount, total.hash );

	// Order-independent fingerprint of the exact hit set.
	qsort( total.hitIds, (size_t)totalHitCount, sizeof( int ), IntCompare );
	uint32_t sortedHash = B2_HASH_INIT;
	for ( int i = 0; i < totalHitCount; ++i )
	{
		sortedHash = b2Hash( sortedHash, (const uint8_t*)( total.hitIds + i ), sizeof( int ) );
	}
	printf( "tree_hit_set hits=%d sorted_hash=%u\n", total.hits, sortedHash );

	// Full rebuild from scratch (all leaves considered moved).
	t0 = NowMs();
	int sorted = b2DynamicTree_Rebuild( &tree, true );
	t1 = NowMs();
	printf( "tree_rebuild rebuild_ms=%.4f sorted=%d height=%d area_ratio=%.4f\n", ( t1 - t0 ) / 1e6, sorted,
			b2DynamicTree_GetHeight( &tree ), b2DynamicTree_GetAreaRatio( &tree ) );

	// Mark every proxy moved then refit, simulating the worst-case per-frame hierarchy update.
	int* moved = malloc( PROXY_COUNT * sizeof( int ) );
	int movedCount = b2DynamicTree_GatherMovedProxies( &tree, moved );
	(void)movedCount;
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		b2DynamicTree_MarkProxyMovedSerial( &tree, i );
	}
	t0 = NowMs();
	b2DynamicTree_Refit( &tree );
	t1 = NowMs();
	printf( "tree_refit refit_ms=%.4f\n", ( t1 - t0 ) / 1e6 );

	// Re-query after rebuild/refit; results must still match.
	QueryResult total2 = { 0 };
	Rng qrng2 = { 0xABCDEF01u };
	for ( int q = 0; q < QUERY_COUNT; ++q )
	{
		float x = RngFloat( &qrng2, -WORLD_EXTENT, WORLD_EXTENT );
		float y = RngFloat( &qrng2, -WORLD_EXTENT, WORLD_EXTENT );
		b2AABB query = {
			.lowerBound = { x, y },
			.upperBound = { x + 4.0f, y + 4.0f },
		};
		QueryResult result = { 0 };
		b2TreeStats stats = b2DynamicTree_OverlapAABB( &tree, query, 1, QueryCallback, &result );
		total2.hits += result.hits;
		total2.hash ^= result.hash + 0x9e3779b9u + ( total2.hash << 6 ) + ( total2.hash >> 2 );
		total2.nodeVisits += stats.nodeVisits;
		total2.leafVisits += stats.leafVisits;
		free( result.hitIds );
	}
	printf( "tree_requery hits=%d hash=%u node_visits=%lld\n", total2.hits, total2.hash,
			( long long )total2.nodeVisits );

	free( total.hitIds );
	free( moved );
	free( boxes );
	b2DestroyDynamicTreeInternal( &tree );
	return 0;
}
