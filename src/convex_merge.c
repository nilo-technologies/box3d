// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

// Surface aware merging of convex pieces.
//
// A volumetric convex decomposition (V-HACD and friends) hands back many small convex pieces.
// Merging them by volume alone lets a merged hull close over a doorway or a window, because an
// opening is cheap in volume. This file merges pieces the way HACD collapses the edges of its
// dual graph, with the concavity error Roblox describes in "A Search for Better Convex
// Decomposition": sample points on the source triangles, cast a ray outward from each sample,
// and measure how far the ray travels inside the candidate hull before it leaves through a back
// face. That distance is the depth of open space the hull would seal. The pair of neighbors
// with the smallest error merges first, and merging stops at the configured concavity.
//
// One refinement over the plain ray length: a ray that runs along a wall, a hand's width away
// from it, can travel far inside a hull that only covers a thin sliver of open space. Voxel
// pieces overshoot the surface by a voxel all the time, so such slivers are everywhere. The
// depth of a ray is therefore capped at twice its clearance, the farthest it gets from the
// source surface while inside the hull. A doorway still reads as its full width, a sliver as
// its thickness.

#include "core.h"
#include "math_internal.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Number of distances along a sample ray where the clearance is recorded
#define B3_MERGE_LADDER_COUNT 24

// A point on the source surface with the outward direction of its triangle
typedef struct b3MergeSample
{
	b3Vec3 point;
	b3Vec3 normal;

	// Distance from the point along the normal to the next surface of the source mesh. A hull is
	// only charged for open space, so a ray stops counting once it reenters the mesh.
	float freeDistance;

	// The farthest the ray has been from the source surface by each ladder distance
	float clearances[B3_MERGE_LADDER_COUNT];
} b3MergeSample;

// An edge of the merge graph as seen from one of its two nodes. Sealing more open space never
// makes a hull better, so what is known about a pair stays a lower bound for whatever the pair
// grows into: the merged hull of two nodes contains the merged hull of any parts of them.
typedef struct b3MergeLink
{
	int node;

	// Lower bound on the concavity of the merged hull
	float concavity;

	// Estimate of the volume the merged hull adds over its two parts
	float growth;
} b3MergeLink;

// A node of the merge graph: one convex piece and the pieces it touches
typedef struct b3MergeNode
{
	b3HullData* hull;
	b3MergeLink* links;
	int linkCount;
	int linkCapacity;

	// Deepest open space this hull seals
	float concavity;

	// Deepest open space any of the original pieces in this hull sealed on its own
	float floor;

	// Bumped on every merge so stale queue entries can be recognized
	int version;
	bool alive;
} b3MergeNode;

// A candidate merge in the priority queue
typedef struct b3MergeCandidate
{
	// Merge order. See b3ScoreMerge.
	float cost;

	// Concavity of the merged hull. Without a hull budget it is not computed past the limit of
	// the pair, since such a merge can never happen.
	float concavity;

	// Volume the merged hull adds over its two parts. Breaks ties between equal costs.
	float growth;

	int nodeA;
	int nodeB;
	int versionA;
	int versionB;

	// An exact candidate was priced with its merged hull. Otherwise the cost is a lower bound
	// carried over from the parts of the pair, and the pair is priced when it reaches the top.
	bool exact;
} b3MergeCandidate;

typedef struct b3MergeContext
{
	b3MergeSample* samples;
	int sampleCount;
	int sampleCapacity;

	// Uniform grid over the samples in compressed row form
	int* cellStarts;
	int* cellSamples;
	b3Vec3 gridLower;
	b3Vec3 gridInvCell;
	int gridCounts[3];

	// Distances along a sample ray where the clearance is recorded, growing geometrically
	float ladder[B3_MERGE_LADDER_COUNT];

	b3MergeNode* nodes;
	int nodeCount;
	int aliveCount;

	b3MergeCandidate* heap;
	int heapCount;
	int heapCapacity;

	// Scratch points for building a merged hull
	b3Vec3* scratch;

	float maxConcavity;
	float tolerance;
	int maxHullVertices;
	bool hasBudget;
} b3MergeContext;

b3ConvexMergeDef b3DefaultConvexMergeDef( void )
{
	b3ConvexMergeDef def = { 0 };
	def.maxConcavity = 0.05f * b3GetLengthUnitsPerMeter();
	def.maxHullCount = 0;
	def.maxHullVertices = 32;
	def.sampleCount = 8192;
	return def;
}

// R2 low discrepancy sequence, folded onto the triangle
static b3Vec3 b3SampleTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int index )
{
	float u = 0.5f + 0.7548776662466927f * (float)index;
	float v = 0.5f + 0.5698402909980532f * (float)index;
	u -= floorf( u );
	v -= floorf( v );
	if ( u + v > 1.0f )
	{
		u = 1.0f - u;
		v = 1.0f - v;
	}

	b3Vec3 p = b3MulAdd( a, u, b3Sub( b, a ) );
	return b3MulAdd( p, v, b3Sub( c, a ) );
}

// The source triangles and the point of a closest query
typedef struct b3MergeTriangles
{
	const b3Vec3* vertices;
	const int32_t* indices;
	b3Vec3 point;
} b3MergeTriangles;

static float b3MergeClosestCallback( float distanceSqrMin, int proxyId, uint64_t userData,
									 void* context )
{
	B3_UNUSED( distanceSqrMin );
	B3_UNUSED( proxyId );

	const b3MergeTriangles* triangles = context;
	const int32_t* index = triangles->indices + 3 * userData;
	b3Vec3 a = triangles->vertices[index[0]];
	b3Vec3 b = triangles->vertices[index[1]];
	b3Vec3 c = triangles->vertices[index[2]];
	b3TrianglePoint closest = b3ClosestPointOnTriangle( a, b, c, triangles->point );
	return b3LengthSquared( b3Sub( closest.point, triangles->point ) );
}

// The depth of open space sealed along a sample ray that runs the given distance inside a hull.
// The run counts in full only where the ray gets clear of the surface: it is capped at twice the
// clearance reached along the way, read off the ladder.
static float b3SealedDepth( const b3MergeContext* ctx, const b3MergeSample* sample, float run )
{
	const float* ladder = ctx->ladder;
	const float* clearances = sample->clearances;

	float clearance;
	if ( run <= ladder[0] )
	{
		clearance = clearances[0] * run / ladder[0];
	}
	else
	{
		int rung = 1;
		while ( rung < B3_MERGE_LADDER_COUNT - 1 && ladder[rung] < run )
		{
			rung += 1;
		}

		float alpha = b3ClampFloat(
			( run - ladder[rung - 1] ) / ( ladder[rung] - ladder[rung - 1] ), 0.0f, 1.0f );
		clearance = clearances[rung - 1] + alpha * ( clearances[rung] - clearances[rung - 1] );
	}

	return b3MinFloat( run, 2.0f * clearance );
}

// Samples spread over the mesh by area, plus one at the centroid of every triangle while the
// mesh is small enough, so a sliver that the area walk steps over still has a say.
static void b3CreateMergeSamples( b3MergeContext* ctx, const b3ConvexMergeDef* def, b3AABB bounds )
{
	const b3Vec3* vertices = def->meshVertices;
	const int32_t* indices = def->meshIndices;
	int triangleCount = def->meshTriangleCount;

	// Six times the signed volume. Negative means the triangles are wound clockwise seen from
	// outside, so the normals are flipped to keep pointing out of the solid.
	double signedVolume = 0.0;
	double totalArea = 0.0;
	b3Vec3 origin = b3AABB_Center( bounds );
	for ( int i = 0; i < triangleCount; ++i )
	{
		b3Vec3 a = b3Sub( vertices[indices[3 * i + 0]], origin );
		b3Vec3 b = b3Sub( vertices[indices[3 * i + 1]], origin );
		b3Vec3 c = b3Sub( vertices[indices[3 * i + 2]], origin );
		signedVolume += (double)b3Dot( a, b3Cross( b, c ) );
		totalArea += 0.5 * (double)b3Length( b3Cross( b3Sub( b, a ), b3Sub( c, a ) ) );
	}

	ctx->samples = NULL;
	ctx->sampleCount = 0;
	ctx->sampleCapacity = 0;
	if ( totalArea <= 0.0 )
	{
		return;
	}

	float flip = signedVolume < 0.0 ? -1.0f : 1.0f;
	int areaSampleCount = b3ClampInt( def->sampleCount, 64, 1 << 20 );
	bool addCentroids = triangleCount <= areaSampleCount;
	int capacity = areaSampleCount + ( addCentroids ? triangleCount : 0 ) + 1;
	ctx->samples = b3Alloc( (size_t)capacity * sizeof( b3MergeSample ) );

	// Ray casts against the source mesh give each sample its free distance
	b3MeshDef meshDef = { 0 };
	meshDef.vertices = (b3Vec3*)vertices;
	meshDef.indices = (int32_t*)indices;
	meshDef.vertexCount = def->meshVertexCount;
	meshDef.triangleCount = triangleCount;
	meshDef.clockWiseWinding = flip < 0.0f;
	b3MeshData* meshData = b3CreateMesh( &meshDef, NULL, 0 );
	b3Mesh mesh = { meshData, { 1.0f, 1.0f, 1.0f } };

	// Long enough to cross the whole mesh from any sample
	float reach = 2.0f * b3Length( b3Sub( bounds.upperBound, bounds.lowerBound ) );
	float rayOffset = 4.0f * ctx->tolerance;

	double step = totalArea / (double)areaSampleCount;
	double next = 0.5 * step;
	double walked = 0.0;
	int count = 0;
	for ( int i = 0; i < triangleCount; ++i )
	{
		b3Vec3 a = vertices[indices[3 * i + 0]];
		b3Vec3 b = vertices[indices[3 * i + 1]];
		b3Vec3 c = vertices[indices[3 * i + 2]];
		b3Vec3 cross = b3Cross( b3Sub( b, a ), b3Sub( c, a ) );
		float length = b3Length( cross );
		if ( length <= FLT_MIN )
		{
			continue;
		}

		b3Vec3 normal = b3MulSV( flip / length, cross );
		double area = 0.5 * (double)length;

		if ( addCentroids )
		{
			b3Vec3 centroid = b3MulSV( 1.0f / 3.0f, b3Add( a, b3Add( b, c ) ) );
			ctx->samples[count].point = centroid;
			ctx->samples[count].normal = normal;
			count += 1;
		}

		walked += area;
		while ( next < walked && count < capacity )
		{
			ctx->samples[count].point = b3SampleTriangle( a, b, c, count );
			ctx->samples[count].normal = normal;
			count += 1;
			next += step;
		}
	}

	// A tree over the triangles answers how far a point is from the source surface
	b3MergeTriangles triangles = { vertices, indices, b3Vec3_zero };
	b3DynamicTree tree = b3DynamicTree_Create( triangleCount );
	for ( int i = 0; i < triangleCount; ++i )
	{
		b3Vec3 a = vertices[indices[3 * i + 0]];
		b3Vec3 b = vertices[indices[3 * i + 1]];
		b3Vec3 c = vertices[indices[3 * i + 2]];
		b3AABB box = { b3Min( a, b3Min( b, c ) ), b3Max( a, b3Max( b, c ) ) };
		b3DynamicTree_CreateProxy( &tree, box, B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	b3DynamicTree_Rebuild( &tree, true );

	for ( int i = 0; i < count; ++i )
	{
		b3MergeSample* sample = ctx->samples + i;
		sample->freeDistance = FLT_MAX;
		if ( meshData == NULL )
		{
			continue;
		}

		b3RayCastInput input;
		input.origin = b3MulAdd( sample->point, rayOffset, sample->normal );
		input.translation = b3MulSV( reach, sample->normal );
		input.maxFraction = 1.0f;
		b3CastOutput output = b3RayCastMesh( &mesh, &input );
		if ( output.hit )
		{
			sample->freeDistance = rayOffset + output.fraction * reach;
		}
	}

	// Walk each ray up the ladder and record the farthest it has been from the surface so far.
	// A point at distance d along the ray is at most d from the surface, which bounds the search.
	for ( int i = 0; i < count; ++i )
	{
		b3MergeSample* sample = ctx->samples + i;
		float clearance = 0.0f;
		for ( int j = 0; j < B3_MERGE_LADDER_COUNT; ++j )
		{
			float distance = ctx->ladder[j];
			if ( distance < sample->freeDistance )
			{
				triangles.point = b3MulAdd( sample->point, distance, sample->normal );
				float distanceSqr = distance * distance;
				b3DynamicTree_QueryClosest( &tree, triangles.point, B3_DEFAULT_MASK_BITS, false,
											b3MergeClosestCallback, &triangles, &distanceSqr );
				clearance = b3MaxFloat( clearance, sqrtf( distanceSqr ) );
			}

			sample->clearances[j] = clearance;
		}
	}

	b3DynamicTree_Destroy( &tree );

	if ( meshData != NULL )
	{
		b3DestroyMesh( meshData );
	}

	ctx->sampleCount = count;
	ctx->sampleCapacity = capacity;
}

static int b3GridCoord( const b3MergeContext* ctx, float value, int axis )
{
	float lower =
		axis == 0 ? ctx->gridLower.x : ( axis == 1 ? ctx->gridLower.y : ctx->gridLower.z );
	float invCell =
		axis == 0 ? ctx->gridInvCell.x : ( axis == 1 ? ctx->gridInvCell.y : ctx->gridInvCell.z );
	int coord = (int)floorf( ( value - lower ) * invCell );
	return b3ClampInt( coord, 0, ctx->gridCounts[axis] - 1 );
}

static int b3GridCell( const b3MergeContext* ctx, b3Vec3 point )
{
	int x = b3GridCoord( ctx, point.x, 0 );
	int y = b3GridCoord( ctx, point.y, 1 );
	int z = b3GridCoord( ctx, point.z, 2 );
	return x + ctx->gridCounts[0] * ( y + ctx->gridCounts[1] * z );
}

// Bucket the samples into a uniform grid so a hull only visits the samples near it
static void b3CreateMergeGrid( b3MergeContext* ctx, b3AABB bounds )
{
	b3Vec3 size = b3Sub( bounds.upperBound, bounds.lowerBound );
	float longest = b3MaxFloat( size.x, b3MaxFloat( size.y, size.z ) );
	float sizes[3] = { size.x, size.y, size.z };

	// About four samples per cell on a surface that crosses the grid
	float resolution = sqrtf( 0.25f * (float)b3MaxInt( ctx->sampleCount, 1 ) );
	float invCells[3];
	int cellCount = 1;
	for ( int axis = 0; axis < 3; ++axis )
	{
		int count = longest > 0.0f ? (int)ceilf( resolution * sizes[axis] / longest ) : 1;
		count = b3ClampInt( count, 1, 64 );
		ctx->gridCounts[axis] = count;
		invCells[axis] = sizes[axis] > 0.0f ? (float)count / sizes[axis] : 0.0f;
		cellCount *= count;
	}

	ctx->gridLower = bounds.lowerBound;
	ctx->gridInvCell = ( b3Vec3 ){ invCells[0], invCells[1], invCells[2] };

	ctx->cellStarts = b3AllocZero( (size_t)( cellCount + 1 ) * sizeof( int ) );
	ctx->cellSamples = b3Alloc( (size_t)b3MaxInt( ctx->sampleCount, 1 ) * sizeof( int ) );

	// Counting sort: counts, then prefix sums, then placement
	for ( int i = 0; i < ctx->sampleCount; ++i )
	{
		ctx->cellStarts[b3GridCell( ctx, ctx->samples[i].point ) + 1] += 1;
	}

	for ( int i = 0; i < cellCount; ++i )
	{
		ctx->cellStarts[i + 1] += ctx->cellStarts[i];
	}

	int* cursors = b3Alloc( (size_t)cellCount * sizeof( int ) );
	memcpy( cursors, ctx->cellStarts, (size_t)cellCount * sizeof( int ) );
	for ( int i = 0; i < ctx->sampleCount; ++i )
	{
		int cell = b3GridCell( ctx, ctx->samples[i].point );
		ctx->cellSamples[cursors[cell]++] = i;
	}

	b3Free( cursors, (size_t)cellCount * sizeof( int ) );
}

// The deepest open space a hull seals: the deepest run of any outward sample ray inside the hull.
// A sample outside the hull does not see it. A sample on a hull face that follows the surface
// leaves at once, so a hull that hugs the source mesh scores zero.
// Stops early and returns a value over the limit once the limit is passed.
static float b3ComputeHullConcavity( const b3MergeContext* ctx, const b3HullData* hull,
									 float limit )
{
	const b3Plane* planes = b3GetHullPlanes( hull );
	int planeCount = hull->faceCount;
	float tolerance = ctx->tolerance;

	int lower[3], upper[3];
	lower[0] = b3GridCoord( ctx, hull->aabb.lowerBound.x - tolerance, 0 );
	lower[1] = b3GridCoord( ctx, hull->aabb.lowerBound.y - tolerance, 1 );
	lower[2] = b3GridCoord( ctx, hull->aabb.lowerBound.z - tolerance, 2 );
	upper[0] = b3GridCoord( ctx, hull->aabb.upperBound.x + tolerance, 0 );
	upper[1] = b3GridCoord( ctx, hull->aabb.upperBound.y + tolerance, 1 );
	upper[2] = b3GridCoord( ctx, hull->aabb.upperBound.z + tolerance, 2 );

	float concavity = 0.0f;
	for ( int z = lower[2]; z <= upper[2]; ++z )
	{
		for ( int y = lower[1]; y <= upper[1]; ++y )
		{
			int row = ctx->gridCounts[0] * ( y + ctx->gridCounts[1] * z );
			int begin = ctx->cellStarts[row + lower[0]];
			int end = ctx->cellStarts[row + upper[0] + 1];
			for ( int i = begin; i < end; ++i )
			{
				const b3MergeSample* sample = ctx->samples + ctx->cellSamples[i];
				float exitDistance = sample->freeDistance;

				// Walk the planes until the sample is found outside, or its ray is known to leave
				// the hull too early to matter.
				bool counts = true;
				for ( int j = 0; j < planeCount; ++j )
				{
					float separation = b3Dot( planes[j].normal, sample->point ) - planes[j].offset;
					if ( separation > tolerance )
					{
						counts = false;
						break;
					}

					float approach = b3Dot( planes[j].normal, sample->normal );
					if ( approach > 1.0e-6f )
					{
						float distance = -separation / approach;
						if ( distance < exitDistance )
						{
							exitDistance = distance;
							if ( exitDistance <= concavity )
							{
								counts = false;
								break;
							}
						}
					}
				}

				if ( counts && exitDistance > concavity )
				{
					float depth = b3SealedDepth( ctx, sample, exitDistance );
					if ( depth > concavity )
					{
						concavity = depth;
						if ( concavity > limit )
						{
							return concavity;
						}
					}
				}
			}
		}
	}

	return concavity;
}

static b3MergeLink* b3FindLink( b3MergeNode* node, int other )
{
	for ( int i = 0; i < node->linkCount; ++i )
	{
		if ( node->links[i].node == other )
		{
			return node->links + i;
		}
	}

	return NULL;
}

// Link a node to another, or raise what is known about an existing link
static void b3SetLink( b3MergeNode* node, int other, float concavity, float growth )
{
	b3MergeLink* link = b3FindLink( node, other );
	if ( link != NULL )
	{
		link->concavity = b3MaxFloat( link->concavity, concavity );
		link->growth = b3MaxFloat( link->growth, growth );
		return;
	}

	if ( node->linkCount == node->linkCapacity )
	{
		int capacity = node->linkCapacity < 8 ? 8 : 2 * node->linkCapacity;
		node->links =
			b3GrowAlloc( node->links, (int)( (size_t)node->linkCapacity * sizeof( b3MergeLink ) ),
						 (int)( (size_t)capacity * sizeof( b3MergeLink ) ) );
		node->linkCapacity = capacity;
	}

	node->links[node->linkCount++] = ( b3MergeLink ){ other, concavity, growth };
}

static void b3RemoveLink( b3MergeNode* node, int other )
{
	b3MergeLink* link = b3FindLink( node, other );
	if ( link != NULL )
	{
		*link = node->links[--node->linkCount];
	}
}

// A total order, so the next merge never depends on what else is in the queue. Equal costs go
// to the smaller volume, and equal volumes to the lower node indices.
static bool b3CandidateLess( const b3MergeCandidate* a, const b3MergeCandidate* b )
{
	if ( a->cost != b->cost )
	{
		return a->cost < b->cost;
	}

	if ( a->growth != b->growth )
	{
		return a->growth < b->growth;
	}

	if ( a->nodeA != b->nodeA )
	{
		return a->nodeA < b->nodeA;
	}

	if ( a->nodeB != b->nodeB )
	{
		return a->nodeB < b->nodeB;
	}

	// The same pair twice: the newer entry first, the older one is stale
	return a->versionA + a->versionB > b->versionA + b->versionB;
}

static void b3PushCandidate( b3MergeContext* ctx, b3MergeCandidate candidate )
{
	if ( ctx->heapCount == ctx->heapCapacity )
	{
		int capacity = ctx->heapCapacity < 64 ? 64 : 2 * ctx->heapCapacity;
		ctx->heap =
			b3GrowAlloc( ctx->heap, (int)( (size_t)ctx->heapCapacity * sizeof( b3MergeCandidate ) ),
						 (int)( (size_t)capacity * sizeof( b3MergeCandidate ) ) );
		ctx->heapCapacity = capacity;
	}

	int index = ctx->heapCount++;
	while ( index > 0 )
	{
		int parent = ( index - 1 ) / 2;
		if ( b3CandidateLess( &candidate, ctx->heap + parent ) == false )
		{
			break;
		}

		ctx->heap[index] = ctx->heap[parent];
		index = parent;
	}

	ctx->heap[index] = candidate;
}

static b3MergeCandidate b3PopCandidate( b3MergeContext* ctx )
{
	b3MergeCandidate top = ctx->heap[0];
	b3MergeCandidate last = ctx->heap[--ctx->heapCount];

	int index = 0;
	while ( true )
	{
		int child = 2 * index + 1;
		if ( child >= ctx->heapCount )
		{
			break;
		}

		if ( child + 1 < ctx->heapCount &&
			 b3CandidateLess( ctx->heap + child + 1, ctx->heap + child ) )
		{
			child += 1;
		}

		if ( b3CandidateLess( ctx->heap + child, &last ) == false )
		{
			break;
		}

		ctx->heap[index] = ctx->heap[child];
		index = child;
	}

	if ( ctx->heapCount > 0 )
	{
		ctx->heap[index] = last;
	}

	return top;
}

// The hull two nodes would become
static b3HullData* b3CreateMergedHull( b3MergeContext* ctx, const b3MergeNode* nodeA,
									   const b3MergeNode* nodeB )
{
	int countA = nodeA->hull->vertexCount;
	int countB = nodeB->hull->vertexCount;
	memcpy( ctx->scratch, b3GetHullPoints( nodeA->hull ), (size_t)countA * sizeof( b3Vec3 ) );
	memcpy( ctx->scratch + countA, b3GetHullPoints( nodeB->hull ),
			(size_t)countB * sizeof( b3Vec3 ) );
	return b3CreateHull( ctx->scratch, countA + countB, ctx->maxHullVertices );
}

// Merge order: the concavity of the merged hull, counted in sixteenths of the limit so pairs
// that are about equally harmless tie, and the tie goes to the pair that wastes the least
// volume. A merge within the limit of its pair never costs more than maxConcavity, and one
// over the limit always does. Those only happen to meet a hull budget.
static float b3ScoreMerge( const b3MergeContext* ctx, float concavity, float limit )
{
	float cost = concavity <= limit ? b3MinFloat( concavity, ctx->maxConcavity ) : concavity;
	float bucket = ctx->maxConcavity / 16.0f;
	return bucket > 0.0f ? bucket * ceilf( cost / bucket ) : cost;
}

// A pair may seal as much as the limit of the merge, or as much as one of its original pieces
// already sealed on its own. A piece that arrives over the limit is not stranded by it.
static float b3GetMergeLimit( const b3MergeContext* ctx, const b3MergeNode* nodeA,
							  const b3MergeNode* nodeB )
{
	return b3MaxFloat( ctx->maxConcavity, b3MaxFloat( nodeA->floor, nodeB->floor ) );
}

// Queue a pair on what is known about it so far. Pricing waits until the pair is the cheapest.
static void b3QueueEstimate( b3MergeContext* ctx, int indexA, int indexB, float concavity,
							 float growth )
{
	const b3MergeNode* nodeA = ctx->nodes + indexA;
	const b3MergeNode* nodeB = ctx->nodes + indexB;

	b3MergeCandidate candidate;
	candidate.concavity = concavity;
	candidate.cost = b3ScoreMerge( ctx, concavity, b3GetMergeLimit( ctx, nodeA, nodeB ) );

	// The volume only breaks ties, so a guess carried over from the parts will do
	candidate.growth = growth;
	candidate.nodeA = indexA;
	candidate.nodeB = indexB;
	candidate.versionA = nodeA->version;
	candidate.versionB = nodeB->version;
	candidate.exact = false;

	// Without a hull budget a merge over the limit can never happen
	if ( candidate.cost > ctx->maxConcavity && ctx->hasBudget == false )
	{
		return;
	}

	b3PushCandidate( ctx, candidate );
}

// Price a pair with its merged hull and queue it again with the exact cost
static void b3PriceCandidate( b3MergeContext* ctx, b3MergeCandidate candidate )
{
	b3MergeNode* nodeA = ctx->nodes + candidate.nodeA;
	b3MergeNode* nodeB = ctx->nodes + candidate.nodeB;

	b3HullData* merged = b3CreateMergedHull( ctx, nodeA, nodeB );
	if ( merged == NULL )
	{
		// These two cannot become one hull
		return;
	}

	// With a hull budget a merge over the limit may still happen, so it is measured in full
	float limit = b3GetMergeLimit( ctx, nodeA, nodeB );
	candidate.concavity = b3ComputeHullConcavity( ctx, merged, ctx->hasBudget ? FLT_MAX : limit );
	candidate.cost = b3ScoreMerge( ctx, candidate.concavity, limit );
	candidate.growth = merged->volume - nodeA->hull->volume - nodeB->hull->volume;
	candidate.exact = true;
	b3DestroyHull( merged );

	b3SetLink( nodeA, candidate.nodeB, candidate.concavity, candidate.growth );
	b3SetLink( nodeB, candidate.nodeA, candidate.concavity, candidate.growth );

	if ( candidate.cost > ctx->maxConcavity && ctx->hasBudget == false )
	{
		return;
	}

	b3PushCandidate( ctx, candidate );
}

// Collapse the edge between two nodes. Node A becomes the merged hull, node B dies.
static void b3MergeNodes( b3MergeContext* ctx, const b3MergeCandidate* candidate )
{
	b3MergeNode* nodeA = ctx->nodes + candidate->nodeA;
	b3MergeNode* nodeB = ctx->nodes + candidate->nodeB;

	b3HullData* merged = b3CreateMergedHull( ctx, nodeA, nodeB );
	if ( merged == NULL )
	{
		// The same points made a hull when the candidate was priced
		B3_ASSERT( false );
		return;
	}

	float concavity = candidate->concavity;

	b3DestroyHull( nodeA->hull );
	b3DestroyHull( nodeB->hull );
	nodeA->hull = merged;
	nodeA->concavity = concavity;
	nodeA->floor = b3MaxFloat( nodeA->floor, nodeB->floor );
	nodeA->version += 1;
	nodeB->hull = NULL;
	nodeB->version += 1;
	nodeB->alive = false;
	ctx->aliveCount -= 1;

	// Node A inherits the links of node B. What either part knew about a neighbor is a lower
	// bound for the merged node, and so is the concavity of the merged node itself.
	b3RemoveLink( nodeA, candidate->nodeB );
	for ( int i = 0; i < nodeB->linkCount; ++i )
	{
		b3MergeLink link = nodeB->links[i];
		if ( link.node == candidate->nodeA )
		{
			continue;
		}

		b3RemoveLink( ctx->nodes + link.node, candidate->nodeB );
		b3SetLink( nodeA, link.node, link.concavity, link.growth );
	}

	nodeB->linkCount = 0;

	for ( int i = 0; i < nodeA->linkCount; ++i )
	{
		b3MergeLink* link = nodeA->links + i;
		link->concavity = b3MaxFloat( link->concavity, concavity );
		b3SetLink( ctx->nodes + link->node, candidate->nodeA, link->concavity, link->growth );
		b3QueueEstimate( ctx, candidate->nodeA, link->node, link->concavity, link->growth );
	}
}

static void b3DestroyMergeContext( b3MergeContext* ctx, int pieceCount )
{
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		b3MergeNode* node = ctx->nodes + i;
		if ( node->hull != NULL )
		{
			b3DestroyHull( node->hull );
		}

		b3Free( node->links, (size_t)node->linkCapacity * sizeof( b3MergeLink ) );
	}

	int cellCount = ctx->gridCounts[0] * ctx->gridCounts[1] * ctx->gridCounts[2];
	b3Free( ctx->nodes, (size_t)pieceCount * sizeof( b3MergeNode ) );
	b3Free( ctx->heap, (size_t)ctx->heapCapacity * sizeof( b3MergeCandidate ) );
	b3Free( ctx->scratch, (size_t)( 2 * B3_MAX_HULL_VERTICES ) * sizeof( b3Vec3 ) );
	b3Free( ctx->cellStarts, (size_t)( cellCount + 1 ) * sizeof( int ) );
	b3Free( ctx->cellSamples, (size_t)b3MaxInt( ctx->sampleCount, 1 ) * sizeof( int ) );
	b3Free( ctx->samples, (size_t)ctx->sampleCapacity * sizeof( b3MergeSample ) );
}

b3ConvexMergeResult* b3MergeConvexPieces( const b3ConvexMergeDef* def )
{
	if ( def == NULL || def->meshVertices == NULL || def->meshIndices == NULL ||
		 def->meshVertexCount < 3 || def->meshTriangleCount < 1 )
	{
		return NULL;
	}

	if ( def->piecePoints == NULL || def->pieceOffsets == NULL || def->pieceCount < 1 )
	{
		return NULL;
	}

	for ( int i = 0; i < 3 * def->meshTriangleCount; ++i )
	{
		if ( def->meshIndices[i] < 0 || def->meshIndices[i] >= def->meshVertexCount )
		{
			return NULL;
		}
	}

	if ( def->pieceOffsets[0] != 0 )
	{
		return NULL;
	}

	for ( int i = 0; i < def->pieceCount; ++i )
	{
		if ( def->pieceOffsets[i + 1] < def->pieceOffsets[i] )
		{
			return NULL;
		}
	}

	b3AABB bounds = { def->meshVertices[0], def->meshVertices[0] };
	for ( int i = 1; i < def->meshVertexCount; ++i )
	{
		bounds.lowerBound = b3Min( bounds.lowerBound, def->meshVertices[i] );
		bounds.upperBound = b3Max( bounds.upperBound, def->meshVertices[i] );
	}

	for ( int i = 0; i < def->pieceOffsets[def->pieceCount]; ++i )
	{
		bounds.lowerBound = b3Min( bounds.lowerBound, def->piecePoints[i] );
		bounds.upperBound = b3Max( bounds.upperBound, def->piecePoints[i] );
	}

	float diagonal = b3Length( b3Sub( bounds.upperBound, bounds.lowerBound ) );
	if ( diagonal <= 0.0f || b3IsValidAABB( bounds ) == false )
	{
		return NULL;
	}

	b3MergeContext ctx = { 0 };
	ctx.maxConcavity = b3MaxFloat( def->maxConcavity, 0.0f );
	ctx.maxHullVertices = b3ClampInt( def->maxHullVertices, 4, B3_MAX_HULL_VERTICES );
	ctx.tolerance = 1.0e-4f * diagonal;
	ctx.hasBudget = def->maxHullCount > 0;

	// The ladder spans three decades up to the size of the mesh
	float ladderRatio = powf( 1024.0f, 1.0f / (float)( B3_MERGE_LADDER_COUNT - 1 ) );
	ctx.ladder[0] = b3MaxFloat( diagonal / 1024.0f, FLT_MIN );
	for ( int i = 1; i < B3_MERGE_LADDER_COUNT; ++i )
	{
		ctx.ladder[i] = ladderRatio * ctx.ladder[i - 1];
	}
	ctx.scratch = b3Alloc( (size_t)( 2 * B3_MAX_HULL_VERTICES ) * sizeof( b3Vec3 ) );

	b3CreateMergeSamples( &ctx, def, bounds );
	b3CreateMergeGrid( &ctx, bounds );

	// One node per piece that makes a hull. A piece too flat or too small to be a hull is dropped,
	// it could not become a shape either.
	ctx.nodes = b3AllocZero( (size_t)def->pieceCount * sizeof( b3MergeNode ) );
	for ( int i = 0; i < def->pieceCount; ++i )
	{
		int begin = def->pieceOffsets[i];
		int count = def->pieceOffsets[i + 1] - begin;
		b3HullData* hull =
			count >= 4 ? b3CreateHull( def->piecePoints + begin, count, ctx.maxHullVertices )
					   : NULL;
		if ( hull == NULL )
		{
			continue;
		}

		b3MergeNode* node = ctx.nodes + ctx.nodeCount++;
		node->hull = hull;
		node->concavity = b3ComputeHullConcavity( &ctx, hull, FLT_MAX );
		node->floor = node->concavity;
		node->alive = true;
	}

	ctx.aliveCount = ctx.nodeCount;

	// Pieces are neighbors when their boxes come within a margin of each other. A pair that only
	// looks close costs one hull build and is then judged like any other.
	float margin = 0.01f * diagonal;
	for ( int i = 0; i < ctx.nodeCount; ++i )
	{
		b3AABB box = ctx.nodes[i].hull->aabb;
		box.lowerBound = b3Sub( box.lowerBound, ( b3Vec3 ){ margin, margin, margin } );
		box.upperBound = b3Add( box.upperBound, ( b3Vec3 ){ margin, margin, margin } );
		for ( int j = i + 1; j < ctx.nodeCount; ++j )
		{
			if ( b3AABB_Overlaps( box, ctx.nodes[j].hull->aabb ) )
			{
				// The merged hull seals at least what its worse part seals
				float concavity = b3MaxFloat( ctx.nodes[i].concavity, ctx.nodes[j].concavity );
				b3SetLink( ctx.nodes + i, j, concavity, 0.0f );
				b3SetLink( ctx.nodes + j, i, concavity, 0.0f );
				b3QueueEstimate( &ctx, i, j, concavity, 0.0f );
			}
		}
	}

	// Collapse the cheapest edge until the next one would seal too much, or, with a hull budget,
	// until the budget is met.
	while ( ctx.heapCount > 0 )
	{
		b3MergeCandidate candidate = b3PopCandidate( &ctx );
		const b3MergeNode* nodeA = ctx.nodes + candidate.nodeA;
		const b3MergeNode* nodeB = ctx.nodes + candidate.nodeB;
		if ( nodeA->alive == false || nodeB->alive == false ||
			 nodeA->version != candidate.versionA || nodeB->version != candidate.versionB )
		{
			continue;
		}

		// Over the limit everything left in the queue is over the limit too, estimates included,
		// and only a hull budget still to be met keeps the merging going.
		bool overBudget = ctx.hasBudget && ctx.aliveCount > def->maxHullCount;
		if ( candidate.cost > ctx.maxConcavity && overBudget == false )
		{
			break;
		}

		if ( candidate.exact == false )
		{
			b3PriceCandidate( &ctx, candidate );
			continue;
		}

		b3MergeNodes( &ctx, &candidate );
	}

	// Pack the surviving hulls into one allocation
	int pointCount = 0;
	for ( int i = 0; i < ctx.nodeCount; ++i )
	{
		if ( ctx.nodes[i].alive )
		{
			pointCount += ctx.nodes[i].hull->vertexCount;
		}
	}

	size_t pointBytes = (size_t)pointCount * sizeof( b3Vec3 );
	size_t offsetBytes = (size_t)( ctx.aliveCount + 1 ) * sizeof( int32_t );
	size_t concavityBytes = (size_t)ctx.aliveCount * sizeof( float );
	size_t byteCount = sizeof( b3ConvexMergeResult ) + pointBytes + offsetBytes + concavityBytes;

	b3ConvexMergeResult* result = b3Alloc( byteCount );
	result->points = (b3Vec3*)( result + 1 );
	result->offsets = (int32_t*)( (char*)result->points + pointBytes );
	result->concavities = (float*)( (char*)result->offsets + offsetBytes );
	result->pieceCount = ctx.aliveCount;
	result->maxConcavity = 0.0f;
	result->byteCount = (int)byteCount;

	int piece = 0;
	int offset = 0;
	for ( int i = 0; i < ctx.nodeCount; ++i )
	{
		const b3MergeNode* node = ctx.nodes + i;
		if ( node->alive == false )
		{
			continue;
		}

		int count = node->hull->vertexCount;
		memcpy( result->points + offset, b3GetHullPoints( node->hull ),
				(size_t)count * sizeof( b3Vec3 ) );
		result->offsets[piece] = offset;
		result->concavities[piece] = node->concavity;
		result->maxConcavity = b3MaxFloat( result->maxConcavity, node->concavity );
		offset += count;
		piece += 1;
	}

	result->offsets[piece] = offset;

	b3DestroyMergeContext( &ctx, def->pieceCount );
	return result;
}

void b3DestroyConvexMergeResult( b3ConvexMergeResult* result )
{
	if ( result != NULL )
	{
		b3Free( result, (size_t)result->byteCount );
	}
}
