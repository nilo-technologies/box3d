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
//
// The pieces need not come from a voxel decomposition, and by default they do not: every triangle
// of the mesh starts as a flat piece, as in HACD and in the Roblox article, which gives up voxels
// because they snap surfaces to a grid. Flat pieces in one plane merge as polygons, priced by
// the area they add to their triangles, until a merge takes them out of the plane.
//
// How many hulls a mesh needs depends on the limit, and the right limit is rarely known up front.
// By default the merge therefore sweeps it: the pieces are merged at a series of loosening
// limits, each carrying on from the hulls the one before left, and the step with the best trade
// between few hulls and little sealed is returned.

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

// A ray from a sample on a hull face counts as running along the face up to this much of a turn
// into the hull, as the sine of the angle
#define B3_MERGE_GRAZING 0.01f

// The most limits a sweep tries
#define B3_MERGE_MAX_SWEEP_STEPS 64

// How often a piece may be cut in two on its way into the merge, and how many pieces all the
// cutting may add
#define B3_MERGE_MAX_SPLIT_DEPTH 6
#define B3_MERGE_MAX_SPLIT_NODES 4096

// The range a sweep covers when the definition leaves it open, as fractions of the diagonal of
// the mesh bounds
#define B3_MERGE_SWEEP_MIN 0.005
#define B3_MERGE_SWEEP_MAX 0.08

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
	// The piece as a hull. A flat piece has none: it is a polygon instead.
	b3HullData* hull;

	// A flat piece: a convex polygon in a plane, counter-clockwise seen from outside the solid
	b3Vec3* polygon;
	int polygonCount;
	b3Vec3 normal;

	// The area of the source triangles in a flat piece
	float solidArea;

	b3MergeLink* links;
	int linkCount;
	int linkCapacity;

	// Deepest open space this hull seals
	float concavity;

	// Deepest open space any of the original pieces in this hull sealed on its own
	float floor;

	// Bumped on every merge so stale queue entries can be recognized
	int version;

	// How many cuts made this piece out of the one that came in
	int splitDepth;
	bool alive;
} b3MergeNode;

// What two nodes would become: a hull, or a polygon when all their points lie in one plane
typedef struct b3MergeShape
{
	b3HullData* hull;
	b3Vec3* polygon;
	int polygonCount;
	b3Vec3 normal;
	float area;
} b3MergeShape;

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
	int nodeCapacity;
	int aliveCount;

	b3MergeCandidate* heap;
	int heapCount;
	int heapCapacity;

	// Scratch points for building a merged hull
	b3Vec3* scratch;

	// The source mesh wound so its normals point out of the solid, and the same mesh wound the
	// other way. A ray cast only sees front faces: against the first a ray stops where it enters
	// the solid, against the second where it leaves.
	b3MeshData* mesh;
	b3MeshData* meshInside;

	// Long enough to cross the whole mesh from anywhere in it
	float reach;

	// -1 if the source triangles are wound clockwise seen from outside the solid
	float flip;

	float maxConcavity;
	float tolerance;
	int maxHullVertices;
	bool hasBudget;
} b3MergeContext;

b3ConvexMergeDef b3DefaultConvexMergeDef( void )
{
	b3ConvexMergeDef def = { 0 };
	def.maxConcavity = 0.0f;
	def.minConcavity = 0.0f;
	def.hullWeight = 1.0f;
	def.maxHullCount = 0;
	def.maxHullVertices = 32;
	def.sampleCount = 8192;
	def.sweepSteps = 9;
	def.enableSplit = false;
	def.seedTriangles = true;
	def.enableSweep = true;
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
	ctx->flip = flip;
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

	float reach = 2.0f * b3Length( b3Sub( bounds.upperBound, bounds.lowerBound ) );
	float rayOffset = 4.0f * ctx->tolerance;
	ctx->mesh = meshData;
	ctx->reach = reach;

	// The same mesh seen from inside the solid, for telling whether a point is in it
	if ( meshData != NULL && ( def->enableSplit || def->seedTriangles ) )
	{
		meshDef.clockWiseWinding = flip > 0.0f;
		ctx->meshInside = b3CreateMesh( &meshDef, NULL, 0 );
	}

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
// Stops early and returns a value over the limit once the limit is passed. worstSample, if asked
// for, receives the sample with the deepest run, or -1.
static float b3ComputeHullConcavity( const b3MergeContext* ctx, const b3HullData* hull, float limit,
									 int* worstSample )
{
	if ( worstSample != NULL )
	{
		*worstSample = -1;
	}

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

					// A sample on a face of the hull whose ray runs along that face is on the
					// hull, not in it: the ray never gets inside.
					float approach = b3Dot( planes[j].normal, sample->normal );
					if ( separation > -tolerance && approach > -B3_MERGE_GRAZING )
					{
						counts = false;
						break;
					}

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
						if ( worstSample != NULL )
						{
							*worstSample = ctx->cellSamples[i];
						}

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

static const b3Vec3* b3GetNodePoints( const b3MergeNode* node, int* count )
{
	if ( node->hull != NULL )
	{
		*count = node->hull->vertexCount;
		return b3GetHullPoints( node->hull );
	}

	*count = node->polygonCount;
	return node->polygon;
}

static b3AABB b3GetNodeBounds( const b3MergeNode* node )
{
	if ( node->hull != NULL )
	{
		return node->hull->aabb;
	}

	return b3MakeAABB( node->polygon, node->polygonCount, 0.0f );
}

static float b3GetNodeVolume( const b3MergeNode* node )
{
	return node->hull != NULL ? node->hull->volume : 0.0f;
}

static void b3DestroyShape( b3MergeShape* shape )
{
	if ( shape->hull != NULL )
	{
		b3DestroyHull( shape->hull );
	}

	b3Free( shape->polygon, (size_t)shape->polygonCount * sizeof( b3Vec3 ) );
	*shape = ( b3MergeShape ){ 0 };
}

// Hand a shape over to a node, which drops what it was
static void b3SetNodeShape( b3MergeNode* node, b3MergeShape* shape )
{
	if ( node->hull != NULL )
	{
		b3DestroyHull( node->hull );
	}

	b3Free( node->polygon, (size_t)node->polygonCount * sizeof( b3Vec3 ) );
	node->hull = shape->hull;
	node->polygon = shape->polygon;
	node->polygonCount = shape->polygonCount;
	node->normal = shape->normal;
	*shape = ( b3MergeShape ){ 0 };
}

// The convex polygon of points that lie in one plane, written back over the points,
// counter-clockwise seen from the side the normal points to. Corners that stand out from their
// neighbors by less than the tolerance go, and so do the corners that matter least while there
// are more than maxCount. Returns the corner count, or 0 if the points enclose no area.
static int b3BuildPolygon( b3Vec3* points, int count, b3Vec3 normal, int maxCount, float tolerance,
						   float* area )
{
	enum
	{
		capacity = 2 * B3_MAX_HULL_VERTICES
	};

	B3_ASSERT( count <= capacity );
	float xs[capacity], ys[capacity];
	int order[capacity], chain[2 * capacity];

	b3Vec3 u = b3Perp( normal );
	b3Vec3 v = b3Cross( normal, u );
	b3Vec3 origin = points[0];
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 d = b3Sub( points[i], origin );
		xs[i] = b3Dot( d, u );
		ys[i] = b3Dot( d, v );

		int slot = i;
		while ( slot > 0 && ( xs[order[slot - 1]] > xs[i] ||
							  ( xs[order[slot - 1]] == xs[i] && ys[order[slot - 1]] > ys[i] ) ) )
		{
			order[slot] = order[slot - 1];
			slot -= 1;
		}

		order[slot] = i;
	}

#define B3_TURN( A, B, C )                                                                         \
	( ( xs[B] - xs[A] ) * ( ys[C] - ys[A] ) - ( ys[B] - ys[A] ) * ( xs[C] - xs[A] ) )

	// Monotone chain: the lower side left to right, then the upper side back
	int k = 0;
	for ( int i = 0; i < count; ++i )
	{
		while ( k >= 2 && B3_TURN( chain[k - 2], chain[k - 1], order[i] ) <= 0.0f )
		{
			k -= 1;
		}

		chain[k++] = order[i];
	}

	for ( int i = count - 2, lower = k + 1; i >= 0; --i )
	{
		while ( k >= lower && B3_TURN( chain[k - 2], chain[k - 1], order[i] ) <= 0.0f )
		{
			k -= 1;
		}

		chain[k++] = order[i];
	}

	// The last corner is the first again
	k -= 1;

	// Drop the corner that stands out least, while it hardly stands out or there are too many
	while ( k > 3 )
	{
		int least = -1;
		float leastTurn = FLT_MAX;
		float leastHeight = 0.0f;
		for ( int i = 0; i < k; ++i )
		{
			int a = chain[( i + k - 1 ) % k], b = chain[i], c = chain[( i + 1 ) % k];
			float turn = B3_TURN( a, b, c );
			if ( turn < leastTurn )
			{
				float base = sqrtf( ( xs[c] - xs[a] ) * ( xs[c] - xs[a] ) +
									( ys[c] - ys[a] ) * ( ys[c] - ys[a] ) );
				leastTurn = turn;
				leastHeight = base > 0.0f ? turn / base : 0.0f;
				least = i;
			}
		}

		if ( k <= maxCount && leastHeight > tolerance )
		{
			break;
		}

		for ( int i = least; i + 1 < k; ++i )
		{
			chain[i] = chain[i + 1];
		}

		k -= 1;
	}

	float twiceArea = 0.0f;
	for ( int i = 1; i + 1 < k; ++i )
	{
		twiceArea += B3_TURN( chain[0], chain[i], chain[i + 1] );
	}

#undef B3_TURN

	*area = 0.5f * twiceArea;
	if ( k < 3 || *area <= tolerance * tolerance )
	{
		return 0;
	}

	b3Vec3 corners[capacity];
	for ( int i = 0; i < k; ++i )
	{
		corners[i] = points[chain[i]];
	}

	memcpy( points, corners, (size_t)k * sizeof( b3Vec3 ) );
	return k;
}

// What two nodes would become. Two flat nodes in one plane make a polygon, anything else a hull.
static bool b3CreateMergedShape( b3MergeContext* ctx, const b3MergeNode* nodeA,
								 const b3MergeNode* nodeB, b3MergeShape* shape )
{
	int countA, countB;
	const b3Vec3* pointsA = b3GetNodePoints( nodeA, &countA );
	const b3Vec3* pointsB = b3GetNodePoints( nodeB, &countB );
	memcpy( ctx->scratch, pointsA, (size_t)countA * sizeof( b3Vec3 ) );
	memcpy( ctx->scratch + countA, pointsB, (size_t)countB * sizeof( b3Vec3 ) );
	*shape = ( b3MergeShape ){ 0 };

	if ( nodeA->hull == NULL && nodeB->hull == NULL )
	{
		// Flat means within the tolerance of one plane, everywhere the same tolerance
		float offset = b3Dot( nodeA->normal, pointsA[0] );
		bool flat = true;
		for ( int i = 0; i < countB && flat; ++i )
		{
			flat = b3AbsFloat( b3Dot( nodeA->normal, pointsB[i] ) - offset ) <= ctx->tolerance;
		}

		if ( flat )
		{
			float area;
			int count = b3BuildPolygon( ctx->scratch, countA + countB, nodeA->normal,
										ctx->maxHullVertices, ctx->tolerance, &area );
			if ( count < 3 )
			{
				return false;
			}

			shape->polygon = b3Alloc( (size_t)count * sizeof( b3Vec3 ) );
			memcpy( shape->polygon, ctx->scratch, (size_t)count * sizeof( b3Vec3 ) );
			shape->polygonCount = count;
			shape->normal = nodeA->normal;
			shape->area = area;
			return true;
		}
	}

	shape->hull = b3CreateHull( ctx->scratch, countA + countB, ctx->maxHullVertices );
	return shape->hull != NULL;
}

// The deepest open space the shape two nodes would become seals
static float b3ComputeShapeConcavity( const b3MergeContext* ctx, const b3MergeShape* shape,
									  const b3MergeNode* nodeA, const b3MergeNode* nodeB,
									  float limit )
{
	if ( shape->hull != NULL )
	{
		return b3ComputeHullConcavity( ctx, shape->hull, limit, NULL );
	}

	// A flat shape gives the sample rays nothing to run through. What it closes over is the area
	// it adds to the triangles it is made of, and the side of a square of that area stands in
	// for the depth.
	float extra = shape->area - nodeA->solidArea - nodeB->solidArea;
	float concavity = extra > 0.0f ? sqrtf( extra ) : 0.0f;
	return concavity > ctx->tolerance ? concavity : 0.0f;
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

	b3MergeShape merged;
	if ( b3CreateMergedShape( ctx, nodeA, nodeB, &merged ) == false )
	{
		// These two cannot become one hull
		return;
	}

	// With a hull budget a merge over the limit may still happen, so it is measured in full
	float limit = b3GetMergeLimit( ctx, nodeA, nodeB );
	bool flat = merged.hull == NULL;
	candidate.concavity =
		b3ComputeShapeConcavity( ctx, &merged, nodeA, nodeB, ctx->hasBudget ? FLT_MAX : limit );
	candidate.cost = b3ScoreMerge( ctx, candidate.concavity, limit );
	candidate.growth =
		flat ? ctx->tolerance * ( merged.area - nodeA->solidArea - nodeB->solidArea )
			 : merged.hull->volume - b3GetNodeVolume( nodeA ) - b3GetNodeVolume( nodeB );
	candidate.exact = true;
	b3DestroyShape( &merged );

	// What a flat shape closes over is no bound on what the pair grows into: more triangles of
	// the same face can fill it in.
	float bound = flat ? 0.0f : candidate.concavity;
	b3SetLink( nodeA, candidate.nodeB, bound, candidate.growth );
	b3SetLink( nodeB, candidate.nodeA, bound, candidate.growth );

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

	b3MergeShape merged;
	if ( b3CreateMergedShape( ctx, nodeA, nodeB, &merged ) == false )
	{
		// The same points made a shape when the candidate was priced
		B3_ASSERT( false );
		return;
	}

	float concavity = candidate->concavity;

	// As in pricing, what a flat node closes over bounds nothing
	float bound = merged.hull == NULL ? 0.0f : concavity;

	b3MergeShape nothing = { 0 };
	b3SetNodeShape( nodeA, &merged );
	b3SetNodeShape( nodeB, &nothing );
	nodeA->solidArea += nodeB->solidArea;
	nodeA->concavity = concavity;
	nodeA->floor = b3MaxFloat( nodeA->floor, nodeB->floor );
	nodeA->version += 1;
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
		link->concavity = b3MaxFloat( link->concavity, bound );
		b3SetLink( ctx->nodes + link->node, candidate->nodeA, link->concavity, link->growth );
		b3QueueEstimate( ctx, candidate->nodeA, link->node, link->concavity, link->growth );
	}
}

// Is the point inside the solid? A ray from it leaves the solid before it enters it. Three rays
// vote, so one that grazes an edge does not decide.
static bool b3IsInsideSolid( const b3MergeContext* ctx, b3Vec3 point )
{
	static const b3Vec3 directions[3] = {
		{ 0.0137f, 0.3113f, 0.9502f },
		{ -0.8249f, -0.5648f, 0.0241f },
		{ 0.5713f, -0.2001f, -0.7960f },
	};

	b3Mesh outside = { ctx->mesh, { 1.0f, 1.0f, 1.0f } };
	b3Mesh inside = { ctx->meshInside, { 1.0f, 1.0f, 1.0f } };
	int votes = 0;
	for ( int i = 0; i < 3; ++i )
	{
		b3RayCastInput input;
		input.origin = point;
		input.translation = b3MulSV( ctx->reach, directions[i] );
		input.maxFraction = 1.0f;
		b3CastOutput enter = b3RayCastMesh( &outside, &input );
		b3CastOutput leave = b3RayCastMesh( &inside, &input );
		if ( leave.hit && ( enter.hit == false || leave.fraction < enter.fraction ) )
		{
			votes += 1;
		}
	}

	return votes >= 2;
}

// The points of the solid inside a hull, gathered triangle by triangle
typedef struct b3FitQuery
{
	const b3Plane* planes;
	int planeCount;

	// How far inside the hull the surface has to be to count
	float margin;
	b3Vec3* points;
	int count;
	int capacity;
} b3FitQuery;

static void b3AddFitPoint( b3FitQuery* query, b3Vec3 point )
{
	if ( query->count == query->capacity )
	{
		int capacity = query->capacity < 256 ? 256 : 2 * query->capacity;
		query->points = B3_GROW( query->points, (size_t)query->capacity, (size_t)capacity );
		query->capacity = capacity;
	}

	query->points[query->count++] = point;
}

// Clip a source triangle to the hull and keep the corners of what is left
static bool b3FitTriangleCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( triangleIndex );

	enum
	{
		capacity = B3_MAX_HULL_FACES + 4
	};

	b3FitQuery* query = context;
	b3Vec3 buffers[2][capacity];
	b3Vec3* in = buffers[0];
	b3Vec3* out = buffers[1];
	in[0] = a;
	in[1] = b;
	in[2] = c;
	int count = 3;

	// Each plane can add one corner
	for ( int i = 0; i < query->planeCount && count > 0; ++i )
	{
		b3Plane plane = query->planes[i];
		int kept = 0;
		for ( int j = 0; j < count; ++j )
		{
			b3Vec3 p = in[j];
			b3Vec3 q = in[( j + 1 ) % count];
			float dp = b3Dot( plane.normal, p ) - plane.offset + query->margin;
			float dq = b3Dot( plane.normal, q ) - plane.offset + query->margin;
			if ( dp <= 0.0f )
			{
				out[kept++] = p;
			}

			if ( ( dp <= 0.0f ) != ( dq <= 0.0f ) && kept < capacity )
			{
				out[kept++] = b3Lerp( p, q, dp / ( dp - dq ) );
			}
		}

		b3Vec3* swap = in;
		in = out;
		out = swap;
		count = kept;
	}

	for ( int i = 0; i < count; ++i )
	{
		b3AddFitPoint( query, in[i] );
	}

	return true;
}

// The hull of the solid inside a hull: the source surface clipped to it, and its own corners
// that lie in the solid. Unlike the hull of a voxel piece it does not reach past the surface.
// Surface that only touches the hull from outside is not in it: a face of the solid lying in
// one of the hull's planes belongs to the hull on the solid's side of that plane alone. So the
// surface is clipped a hair inside the hull, and a corner is judged a hair toward its middle.
// Returns NULL if there is no solid in the hull to speak of.
static b3HullData* b3FitHull( const b3MergeContext* ctx, const b3HullData* hull )
{
	b3FitQuery query = { 0 };
	query.planes = b3GetHullPlanes( hull );
	query.planeCount = hull->faceCount;
	query.margin = ctx->tolerance;

	b3Mesh mesh = { ctx->mesh, { 1.0f, 1.0f, 1.0f } };
	b3QueryMesh( &mesh, b3AABB_Inflate( hull->aabb, ctx->tolerance ), b3FitTriangleCallback,
				 &query );

	const b3Vec3* corners = b3GetHullPoints( hull );
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		b3Vec3 inward = b3Sub( hull->center, corners[i] );
		float distance = b3Length( inward );
		float step = b3MinFloat( 4.0f * ctx->tolerance, 0.5f * distance );
		b3Vec3 probe =
			distance > 0.0f ? b3MulAdd( corners[i], step / distance, inward ) : corners[i];
		if ( b3IsInsideSolid( ctx, probe ) )
		{
			b3AddFitPoint( &query, corners[i] );
		}
	}

	b3HullData* fitted =
		query.count >= 4 ? b3CreateHull( query.points, query.count, ctx->maxHullVertices ) : NULL;
	b3Free( query.points, (size_t)query.capacity * sizeof( b3Vec3 ) );
	return fitted;
}

// The two parts of a hull on either side of a plane. A part with no volume comes back NULL.
static void b3SplitHull( const b3MergeContext* ctx, const b3HullData* hull, b3Plane plane,
						 b3HullData** below, b3HullData** above )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	int capacity = hull->vertexCount + hull->edgeCount / 2;
	b3Vec3* parts = b3Alloc( (size_t)( 2 * capacity ) * sizeof( b3Vec3 ) );
	b3Vec3* lower = parts;
	b3Vec3* upper = parts + capacity;
	int lowerCount = 0, upperCount = 0;

	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		float distance = b3Dot( plane.normal, points[i] ) - plane.offset;
		if ( distance <= ctx->tolerance )
		{
			lower[lowerCount++] = points[i];
		}

		if ( distance >= -ctx->tolerance )
		{
			upper[upperCount++] = points[i];
		}
	}

	// Each edge that crosses the plane gives both parts a corner
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( edges[i].twin < i )
		{
			continue;
		}

		b3Vec3 p = points[edges[i].origin];
		b3Vec3 q = points[edges[edges[i].twin].origin];
		float dp = b3Dot( plane.normal, p ) - plane.offset;
		float dq = b3Dot( plane.normal, q ) - plane.offset;
		if ( ( dp < -ctx->tolerance && dq > ctx->tolerance ) ||
			 ( dp > ctx->tolerance && dq < -ctx->tolerance ) )
		{
			b3Vec3 crossing = b3Lerp( p, q, dp / ( dp - dq ) );
			lower[lowerCount++] = crossing;
			upper[upperCount++] = crossing;
		}
	}

	*below = lowerCount >= 4 ? b3CreateHull( lower, lowerCount, ctx->maxHullVertices ) : NULL;
	*above = upperCount >= 4 ? b3CreateHull( upper, upperCount, ctx->maxHullVertices ) : NULL;
	b3Free( parts, (size_t)( 2 * capacity ) * sizeof( b3Vec3 ) );
}

// Cut a node that seals too much in two, each part fitted to the solid again. The cut goes
// through the sample with the deepest run: along the surface there, which takes an L apart at
// its inner corner, or across the surface, along or across the length of the piece, which
// halves a piece bent around a hollow. A cut is judged by what its parts seal, each weighed by
// its volume, so a cut that frees one part cleanly counts even while the other part still seals
// as much: that part is cut again. Returns false if no cut makes the piece better.
static bool b3SplitNode( b3MergeContext* ctx, int index )
{
	const b3HullData* hull = ctx->nodes[index].hull;
	int worst;
	float concavity = b3ComputeHullConcavity( ctx, hull, FLT_MAX, &worst );
	if ( worst < 0 )
	{
		return false;
	}

	b3Vec3 point = ctx->samples[worst].point;
	b3Vec3 normal = ctx->samples[worst].normal;

	// The long direction of the piece in the plane of the surface
	b3Vec3 u = b3Perp( normal );
	b3Vec3 v = b3Cross( normal, u );
	const b3Vec3* corners = b3GetHullPoints( hull );
	float sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		b3Vec3 d = b3Sub( corners[i], hull->center );
		float x = b3Dot( d, u );
		float y = b3Dot( d, v );
		sxx += x * x;
		sxy += x * y;
		syy += y * y;
	}

	float angle = 0.5f * atan2f( 2.0f * sxy, sxx - syy );
	b3Vec3 along = b3Add( b3MulSV( cosf( angle ), u ), b3MulSV( sinf( angle ), v ) );
	b3Vec3 across = b3Cross( normal, along );
	b3Vec3 cuts[3] = { normal, along, across };

	b3HullData* best[2] = { NULL, NULL };
	float bestConcavities[2] = { 0.0f, 0.0f };
	float bestScore = FLT_MAX;
	for ( int i = 0; i < 3; ++i )
	{
		b3Plane plane = { cuts[i], b3Dot( cuts[i], point ) };
		b3HullData* halves[2];
		b3SplitHull( ctx, hull, plane, halves + 0, halves + 1 );

		float concavities[2] = { 0.0f, 0.0f };
		for ( int j = 0; j < 2; ++j )
		{
			if ( halves[j] == NULL )
			{
				continue;
			}

			b3HullData* fitted = b3FitHull( ctx, halves[j] );
			b3DestroyHull( halves[j] );
			halves[j] = fitted;
			if ( fitted != NULL )
			{
				concavities[j] = b3ComputeHullConcavity( ctx, fitted, FLT_MAX, NULL );
			}
		}

		float score = 0.0f;
		for ( int j = 0; j < 2; ++j )
		{
			score += halves[j] != NULL ? concavities[j] * halves[j]->volume : 0.0f;
		}

		bool usable = halves[0] != NULL || halves[1] != NULL;
		if ( usable && score < bestScore )
		{
			for ( int j = 0; j < 2; ++j )
			{
				if ( best[j] != NULL )
				{
					b3DestroyHull( best[j] );
				}

				best[j] = halves[j];
				bestConcavities[j] = concavities[j];
			}

			bestScore = score;
		}
		else
		{
			for ( int j = 0; j < 2; ++j )
			{
				if ( halves[j] != NULL )
				{
					b3DestroyHull( halves[j] );
				}
			}
		}
	}

	if ( bestScore > 0.9f * concavity * hull->volume )
	{
		for ( int j = 0; j < 2; ++j )
		{
			if ( best[j] != NULL )
			{
				b3DestroyHull( best[j] );
			}
		}

		return false;
	}

	// The node becomes one part, the other part is a new node
	int first = best[0] != NULL ? 0 : 1;
	b3MergeNode* node = ctx->nodes + index;
	b3DestroyHull( node->hull );
	node->hull = best[first];
	node->concavity = bestConcavities[first];
	node->floor = node->concavity;
	node->splitDepth += 1;

	if ( first == 0 && best[1] != NULL )
	{
		if ( ctx->nodeCount == ctx->nodeCapacity )
		{
			int capacity = 2 * ctx->nodeCapacity;
			ctx->nodes = B3_GROW_ZERO( ctx->nodes, (size_t)ctx->nodeCapacity, (size_t)capacity );
			ctx->nodeCapacity = capacity;
		}

		b3MergeNode* other = ctx->nodes + ctx->nodeCount++;
		*other = ( b3MergeNode ){ 0 };
		other->hull = best[1];
		other->concavity = bestConcavities[1];
		other->floor = other->concavity;
		other->splitDepth = ctx->nodes[index].splitDepth;
		other->alive = true;
	}

	return true;
}

// Fit every piece to the solid inside it, then cut the ones that still seal more than the
// target until they no longer do, or cutting stops helping.
static void b3FitAndSplitNodes( b3MergeContext* ctx, float target )
{
	if ( ctx->mesh == NULL || ctx->meshInside == NULL )
	{
		return;
	}

	int count = 0;
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		b3HullData* piece = ctx->nodes[i].hull;
		b3HullData* fitted = b3FitHull( ctx, piece );
		b3DestroyHull( piece );
		ctx->nodes[i] = ( b3MergeNode ){ 0 };
		if ( fitted == NULL )
		{
			// A piece with no solid in it
			continue;
		}

		b3MergeNode* node = ctx->nodes + count++;
		node->hull = fitted;
		node->concavity = b3ComputeHullConcavity( ctx, fitted, FLT_MAX, NULL );
		node->floor = node->concavity;
		node->alive = true;
	}

	ctx->nodeCount = count;

	int limit = ctx->nodeCount + B3_MERGE_MAX_SPLIT_NODES;
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		while ( ctx->nodes[i].concavity > target &&
				ctx->nodes[i].splitDepth < B3_MERGE_MAX_SPLIT_DEPTH && ctx->nodeCount < limit )
		{
			if ( b3SplitNode( ctx, i ) == false )
			{
				break;
			}
		}
	}
}

// One flat node for every triangle of the source mesh
static void b3SeedTriangleNodes( b3MergeContext* ctx, const b3ConvexMergeDef* def )
{
	for ( int i = 0; i < def->meshTriangleCount; ++i )
	{
		b3Vec3 a = def->meshVertices[def->meshIndices[3 * i + 0]];
		b3Vec3 b = def->meshVertices[def->meshIndices[3 * i + 1]];
		b3Vec3 c = def->meshVertices[def->meshIndices[3 * i + 2]];
		b3Vec3 cross = b3Cross( b3Sub( b, a ), b3Sub( c, a ) );
		float length = b3Length( cross );
		if ( 0.5f * length <= ctx->tolerance * ctx->tolerance )
		{
			continue;
		}

		b3MergeNode* node = ctx->nodes + ctx->nodeCount++;
		node->polygon = b3Alloc( 3 * sizeof( b3Vec3 ) );
		node->polygon[0] = a;
		node->polygon[1] = ctx->flip < 0.0f ? c : b;
		node->polygon[2] = ctx->flip < 0.0f ? b : c;
		node->polygonCount = 3;
		node->normal = b3MulSV( ctx->flip / length, cross );
		node->solidArea = 0.5f * length;
		node->alive = true;
	}
}

// A flat node has to become a hull to be of use. Its polygon is pushed into the solid as far as
// the solid goes behind its middle, and less far while the hull that makes seals more than the
// limit. concavity receives what the hull seals.
static b3HullData* b3SolidifyNode( const b3MergeContext* ctx, const b3MergeNode* node,
								   float* concavity )
{
	int count = node->polygonCount;
	b3Vec3 center = b3Vec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		center = b3Add( center, node->polygon[i] );
	}

	center = b3MulSV( 1.0f / (float)count, center );

	float offset = 4.0f * ctx->tolerance;
	float minDepth = 2.0f * offset;
	float depth = minDepth;
	if ( ctx->meshInside != NULL )
	{
		b3Mesh inside = { ctx->meshInside, { 1.0f, 1.0f, 1.0f } };
		b3RayCastInput input;
		input.origin = b3MulSub( center, offset, node->normal );
		input.translation = b3MulSV( -ctx->reach, node->normal );
		input.maxFraction = 1.0f;
		b3CastOutput output = b3RayCastMesh( &inside, &input );
		if ( output.hit )
		{
			depth = b3MaxFloat( minDepth, offset + output.fraction * ctx->reach );
		}
	}

	b3Vec3 points[2 * B3_MAX_HULL_VERTICES];
	for ( int attempt = 0; attempt < 8; ++attempt )
	{
		if ( attempt == 7 )
		{
			depth = minDepth;
		}

		for ( int i = 0; i < count; ++i )
		{
			points[i] = node->polygon[i];
			points[count + i] = b3MulSub( node->polygon[i], depth, node->normal );
		}

		b3HullData* hull = b3CreateHull( points, 2 * count, ctx->maxHullVertices );
		if ( hull == NULL )
		{
			return NULL;
		}

		float sealed = b3ComputeHullConcavity( ctx, hull, FLT_MAX, NULL );
		if ( sealed <= ctx->maxConcavity || depth <= minDepth )
		{
			*concavity = b3MaxFloat( sealed, node->concavity );
			return hull;
		}

		b3DestroyHull( hull );
		depth = b3MaxFloat( minDepth, 0.5f * depth );
	}

	return NULL;
}

static void b3DestroyMergeContext( b3MergeContext* ctx )
{
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		b3MergeNode* node = ctx->nodes + i;
		if ( node->hull != NULL )
		{
			b3DestroyHull( node->hull );
		}

		b3Free( node->polygon, (size_t)node->polygonCount * sizeof( b3Vec3 ) );
		b3Free( node->links, (size_t)node->linkCapacity * sizeof( b3MergeLink ) );
	}

	if ( ctx->mesh != NULL )
	{
		b3DestroyMesh( ctx->mesh );
	}

	if ( ctx->meshInside != NULL )
	{
		b3DestroyMesh( ctx->meshInside );
	}

	int cellCount = ctx->gridCounts[0] * ctx->gridCounts[1] * ctx->gridCounts[2];
	b3Free( ctx->nodes, (size_t)ctx->nodeCapacity * sizeof( b3MergeNode ) );
	b3Free( ctx->heap, (size_t)ctx->heapCapacity * sizeof( b3MergeCandidate ) );
	b3Free( ctx->scratch, (size_t)( 2 * B3_MAX_HULL_VERTICES ) * sizeof( b3Vec3 ) );
	b3Free( ctx->cellStarts, (size_t)( cellCount + 1 ) * sizeof( int ) );
	b3Free( ctx->cellSamples, (size_t)b3MaxInt( ctx->sampleCount, 1 ) * sizeof( int ) );
	b3Free( ctx->samples, (size_t)ctx->sampleCapacity * sizeof( b3MergeSample ) );
}

// A neighbor search: the node asking, and the context its links go into
typedef struct b3MergeNeighborQuery
{
	b3MergeContext* ctx;
	int index;
} b3MergeNeighborQuery;

// Link a node to a neighbor the tree found. Each pair is linked once, from its lower node.
static bool b3MergeNeighborCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	b3MergeNeighborQuery* query = context;
	b3MergeContext* ctx = query->ctx;
	int i = query->index;
	int j = (int)userData;
	if ( j > i )
	{
		// The merged hull seals at least what its worse part seals
		float concavity = b3MaxFloat( ctx->nodes[i].concavity, ctx->nodes[j].concavity );
		b3SetLink( ctx->nodes + i, j, concavity, 0.0f );
		b3SetLink( ctx->nodes + j, i, concavity, 0.0f );
	}

	return true;
}

// Is the definition fit to merge?
static bool b3IsMergeDefValid( const b3ConvexMergeDef* def )
{
	if ( def == NULL || def->meshVertices == NULL || def->meshIndices == NULL ||
		 def->meshVertexCount < 3 || def->meshTriangleCount < 1 )
	{
		return false;
	}

	for ( int i = 0; i < 3 * def->meshTriangleCount; ++i )
	{
		if ( def->meshIndices[i] < 0 || def->meshIndices[i] >= def->meshVertexCount )
		{
			return false;
		}
	}

	// Seeded with the triangles of the mesh, the merge needs no pieces
	if ( def->seedTriangles )
	{
		return true;
	}

	if ( def->piecePoints == NULL || def->pieceOffsets == NULL || def->pieceCount < 1 )
	{
		return false;
	}

	if ( def->pieceOffsets[0] != 0 )
	{
		return false;
	}

	for ( int i = 0; i < def->pieceCount; ++i )
	{
		if ( def->pieceOffsets[i + 1] < def->pieceOffsets[i] )
		{
			return false;
		}
	}

	return true;
}

// Build what every merge of a definition shares: the samples, their grid, one node per piece
// and the links between neighbors. tightestLimit is the least the merge will be held to.
static bool b3CreateMergeContext( b3MergeContext* ctx, const b3ConvexMergeDef* def,
								  float tightestLimit )
{
	b3AABB bounds = { def->meshVertices[0], def->meshVertices[0] };
	for ( int i = 1; i < def->meshVertexCount; ++i )
	{
		bounds.lowerBound = b3Min( bounds.lowerBound, def->meshVertices[i] );
		bounds.upperBound = b3Max( bounds.upperBound, def->meshVertices[i] );
	}

	int piecePointCount = def->seedTriangles ? 0 : def->pieceOffsets[def->pieceCount];
	for ( int i = 0; i < piecePointCount; ++i )
	{
		bounds.lowerBound = b3Min( bounds.lowerBound, def->piecePoints[i] );
		bounds.upperBound = b3Max( bounds.upperBound, def->piecePoints[i] );
	}

	float diagonal = b3Length( b3Sub( bounds.upperBound, bounds.lowerBound ) );
	if ( diagonal <= 0.0f || b3IsValidAABB( bounds ) == false )
	{
		return false;
	}

	*ctx = ( b3MergeContext ){ 0 };
	ctx->maxHullVertices = b3ClampInt( def->maxHullVertices, 4, B3_MAX_HULL_VERTICES );
	ctx->tolerance = 1.0e-4f * diagonal;
	ctx->flip = 1.0f;

	// The ladder spans three decades up to the size of the mesh
	float ladderRatio = powf( 1024.0f, 1.0f / (float)( B3_MERGE_LADDER_COUNT - 1 ) );
	ctx->ladder[0] = b3MaxFloat( diagonal / 1024.0f, FLT_MIN );
	for ( int i = 1; i < B3_MERGE_LADDER_COUNT; ++i )
	{
		ctx->ladder[i] = ladderRatio * ctx->ladder[i - 1];
	}

	ctx->scratch = b3Alloc( (size_t)( 2 * B3_MAX_HULL_VERTICES ) * sizeof( b3Vec3 ) );

	b3CreateMergeSamples( ctx, def, bounds );
	b3CreateMergeGrid( ctx, bounds );

	ctx->nodeCapacity = def->seedTriangles ? def->meshTriangleCount : def->pieceCount;
	ctx->nodes = b3AllocZero( (size_t)ctx->nodeCapacity * sizeof( b3MergeNode ) );
	if ( def->seedTriangles )
	{
		b3SeedTriangleNodes( ctx, def );
	}
	else
	{
		// One node per piece that makes a hull. A piece too flat or too small to be a hull is
		// dropped, it could not become a shape either.
		for ( int i = 0; i < def->pieceCount; ++i )
		{
			int begin = def->pieceOffsets[i];
			int count = def->pieceOffsets[i + 1] - begin;
			b3HullData* hull =
				count >= 4 ? b3CreateHull( def->piecePoints + begin, count, ctx->maxHullVertices )
						   : NULL;
			if ( hull == NULL )
			{
				continue;
			}

			b3MergeNode* node = ctx->nodes + ctx->nodeCount++;
			node->hull = hull;
			node->concavity = b3ComputeHullConcavity( ctx, hull, FLT_MAX, NULL );
			node->floor = node->concavity;
			node->alive = true;
		}

		if ( def->enableSplit )
		{
			b3FitAndSplitNodes( ctx, tightestLimit );
		}
	}

	ctx->aliveCount = ctx->nodeCount;

	// Pieces are neighbors when their boxes come within a margin of each other. A pair that only
	// looks close costs one hull build and is then judged like any other. Triangles are
	// neighbors when their boxes touch. A tree over the boxes finds the pairs, so the search
	// keeps up with a mesh of many triangles.
	float margin = def->seedTriangles ? ctx->tolerance : 0.01f * diagonal;
	b3DynamicTree tree = b3DynamicTree_Create( b3MaxInt( ctx->nodeCount, 1 ) );
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		b3DynamicTree_CreateProxy( &tree, b3GetNodeBounds( ctx->nodes + i ),
								   B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	b3DynamicTree_Rebuild( &tree, true );

	b3MergeNeighborQuery query = { ctx, 0 };
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		query.index = i;
		b3AABB box = b3AABB_Inflate( b3GetNodeBounds( ctx->nodes + i ), margin );
		b3DynamicTree_Query( &tree, box, B3_DEFAULT_MASK_BITS, false, b3MergeNeighborCallback,
							 &query );
	}

	b3DynamicTree_Destroy( &tree );
	return true;
}

// Merge under a limit, starting from wherever the context stands. Run again with a looser limit
// and the merge carries on from the hulls the tighter one left.
static void b3RunMerge( b3MergeContext* ctx, float maxConcavity, int maxHullCount )
{
	ctx->maxConcavity = b3MaxFloat( maxConcavity, 0.0f );
	ctx->hasBudget = maxHullCount > 0;

	// Queue every link afresh: what a merge may cost depends on the limit. Each link is queued on
	// what earlier pricing found, a lower bound. A pair that bound already puts over the new
	// limit is not built again. One within reach is priced again when it comes up, since its
	// earlier measure stopped at the limit of the time.
	ctx->heapCount = 0;
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		const b3MergeNode* node = ctx->nodes + i;
		if ( node->alive == false )
		{
			continue;
		}

		for ( int j = 0; j < node->linkCount; ++j )
		{
			b3MergeLink link = node->links[j];
			if ( link.node > i )
			{
				b3QueueEstimate( ctx, i, link.node, link.concavity, link.growth );
			}
		}
	}

	// Collapse the cheapest edge until the next one would seal too much, or, with a hull budget,
	// until the budget is met.
	while ( ctx->heapCount > 0 )
	{
		b3MergeCandidate candidate = b3PopCandidate( ctx );
		const b3MergeNode* nodeA = ctx->nodes + candidate.nodeA;
		const b3MergeNode* nodeB = ctx->nodes + candidate.nodeB;
		if ( nodeA->alive == false || nodeB->alive == false ||
			 nodeA->version != candidate.versionA || nodeB->version != candidate.versionB )
		{
			continue;
		}

		// Over the limit everything left in the queue is over the limit too, estimates included,
		// and only a hull budget still to be met keeps the merging going.
		bool overBudget = ctx->hasBudget && ctx->aliveCount > maxHullCount;
		if ( candidate.cost > ctx->maxConcavity && overBudget == false )
		{
			break;
		}

		if ( candidate.exact == false )
		{
			b3PriceCandidate( ctx, candidate );
			continue;
		}

		b3MergeNodes( ctx, &candidate );
	}
}

// Pack the surviving hulls into one allocation, with room for the steps. A flat piece is made
// into a hull on the way, and left out if it cannot be.
static b3ConvexMergeResult* b3PackMergeResult( const b3MergeContext* ctx, int stepCount )
{
	b3HullData** solids =
		b3AllocZero( (size_t)b3MaxInt( ctx->nodeCount, 1 ) * sizeof( b3HullData* ) );
	float* sealed = b3AllocZero( (size_t)b3MaxInt( ctx->nodeCount, 1 ) * sizeof( float ) );

	int pieceCount = 0;
	int pointCount = 0;
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		const b3MergeNode* node = ctx->nodes + i;
		if ( node->alive == false )
		{
			continue;
		}

		sealed[i] = node->concavity;
		const b3HullData* hull = node->hull;
		if ( hull == NULL )
		{
			solids[i] = b3SolidifyNode( ctx, node, sealed + i );
			hull = solids[i];
		}

		if ( hull != NULL )
		{
			pieceCount += 1;
			pointCount += hull->vertexCount;
		}
	}

	size_t pointBytes = (size_t)pointCount * sizeof( b3Vec3 );
	size_t offsetBytes = (size_t)( pieceCount + 1 ) * sizeof( int32_t );
	size_t concavityBytes = (size_t)pieceCount * sizeof( float );
	size_t stepBytes = (size_t)stepCount * sizeof( b3ConvexMergeStep );
	size_t byteCount =
		sizeof( b3ConvexMergeResult ) + pointBytes + offsetBytes + concavityBytes + stepBytes;

	b3ConvexMergeResult* result = b3Alloc( byteCount );
	result->points = (b3Vec3*)( result + 1 );
	result->offsets = (int32_t*)( (char*)result->points + pointBytes );
	result->concavities = (float*)( (char*)result->offsets + offsetBytes );
	result->steps = (b3ConvexMergeStep*)( (char*)result->concavities + concavityBytes );
	result->pieceCount = pieceCount;
	result->maxConcavity = 0.0f;
	result->stepCount = stepCount;
	result->bestStep = 0;
	result->byteCount = (int)byteCount;

	int piece = 0;
	int offset = 0;
	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		const b3MergeNode* node = ctx->nodes + i;
		const b3HullData* hull = node->hull != NULL ? node->hull : solids[i];
		if ( node->alive == false || hull == NULL )
		{
			continue;
		}

		int count = hull->vertexCount;
		memcpy( result->points + offset, b3GetHullPoints( hull ),
				(size_t)count * sizeof( b3Vec3 ) );
		result->offsets[piece] = offset;
		result->concavities[piece] = sealed[i];
		result->maxConcavity = b3MaxFloat( result->maxConcavity, sealed[i] );
		offset += count;
		piece += 1;
	}

	result->offsets[piece] = offset;

	for ( int i = 0; i < ctx->nodeCount; ++i )
	{
		if ( solids[i] != NULL )
		{
			b3DestroyHull( solids[i] );
		}
	}

	b3Free( solids, (size_t)b3MaxInt( ctx->nodeCount, 1 ) * sizeof( b3HullData* ) );
	b3Free( sealed, (size_t)b3MaxInt( ctx->nodeCount, 1 ) * sizeof( float ) );
	return result;
}

// The limits to merge at, tightest first. Without a sweep that is the limit of the definition.
// A sweep spreads its limits geometrically over a range that follows the size of the mesh
// unless the definition gives it.
static int b3GetMergeLimits( const b3ConvexMergeDef* def, float* limits )
{
	if ( def->enableSweep == false )
	{
		limits[0] = b3MaxFloat( def->maxConcavity, 0.0f );
		return 1;
	}

	b3Vec3 lower = def->meshVertices[0];
	b3Vec3 upper = def->meshVertices[0];
	for ( int i = 1; i < def->meshVertexCount; ++i )
	{
		lower = b3Min( lower, def->meshVertices[i] );
		upper = b3Max( upper, def->meshVertices[i] );
	}

	double dx = (double)upper.x - (double)lower.x;
	double dy = (double)upper.y - (double)lower.y;
	double dz = (double)upper.z - (double)lower.z;
	double diagonal = sqrt( dx * dx + dy * dy + dz * dz );

	// The comparisons also turn away a NaN. maxConcavity caps the sweep, so a range given
	// upside down collapses onto it.
	double hi =
		def->maxConcavity > 0.0f ? (double)def->maxConcavity : B3_MERGE_SWEEP_MAX * diagonal;
	double lo =
		def->minConcavity > 0.0f ? (double)def->minConcavity : B3_MERGE_SWEEP_MIN * diagonal;
	lo = lo < hi ? lo : hi;

	int count = b3ClampInt( def->sweepSteps, 1, B3_MERGE_MAX_SWEEP_STEPS );
	if ( count == 1 || !( lo > 0.0 ) || lo == hi )
	{
		limits[0] = (float)hi;
		return 1;
	}

	for ( int i = 0; i < count; ++i )
	{
		limits[i] = (float)( lo * pow( hi / lo, (double)i / (double)( count - 1 ) ) );
	}

	return count;
}

// The step of a sweep to return. Each step is a point: hull count against the depth its hulls
// seal. Both are scaled by their largest value over the steps, and the step nearest to no hulls
// and nothing sealed wins. Among steps with the same hull count only the first, the tightest, is
// a candidate.
static int b3ChooseMergeStep( const b3ConvexMergeStep* steps, int count, float hullWeight )
{
	double weight = b3IsValidFloat( hullWeight ) && hullWeight > 0.0f ? (double)hullWeight : 1.0;
	int maxHulls = 0;
	float maxSealed = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		maxHulls = b3MaxInt( maxHulls, steps[i].hullCount );
		maxSealed = b3MaxFloat( maxSealed, steps[i].sealedDepth );
	}

	int best = 0;
	double bestScore = DBL_MAX;
	for ( int i = 0; i < count; ++i )
	{
		if ( i > 0 && steps[i].hullCount == steps[i - 1].hullCount )
		{
			continue;
		}

		double hulls = maxHulls > 0 ? weight * (double)steps[i].hullCount / (double)maxHulls : 0.0;
		double sealed = maxSealed > 0.0f ? (double)steps[i].sealedDepth / (double)maxSealed : 0.0;
		double score = hulls * hulls + sealed * sealed;
		if ( score < bestScore )
		{
			bestScore = score;
			best = i;
		}
	}

	return best;
}

b3ConvexMergeResult* b3MergeConvexPieces( const b3ConvexMergeDef* def )
{
	if ( b3IsMergeDefValid( def ) == false )
	{
		return NULL;
	}

	float limits[B3_MERGE_MAX_SWEEP_STEPS];
	int stepCount = b3GetMergeLimits( def, limits );

	b3MergeContext ctx;
	if ( b3CreateMergeContext( &ctx, def, limits[0] ) == false )
	{
		return NULL;
	}

	// Each limit carries on from the hulls the one before left. The step to return is only known
	// once all of them are in, so the hulls of every step that could be chosen are kept: the
	// first step at each hull count.
	b3ConvexMergeStep steps[B3_MERGE_MAX_SWEEP_STEPS];
	b3ConvexMergeResult* packed[B3_MERGE_MAX_SWEEP_STEPS] = { 0 };
	for ( int i = 0; i < stepCount; ++i )
	{
		b3RunMerge( &ctx, limits[i], def->maxHullCount );

		float sealedDepth = 0.0f;
		for ( int j = 0; j < ctx.nodeCount; ++j )
		{
			if ( ctx.nodes[j].alive )
			{
				sealedDepth = b3MaxFloat( sealedDepth, ctx.nodes[j].concavity );
			}
		}

		steps[i] = ( b3ConvexMergeStep ){ limits[i], sealedDepth, ctx.aliveCount };
		if ( i == 0 || steps[i].hullCount != steps[i - 1].hullCount )
		{
			// A flat piece made into a hull can seal more than it did as a polygon
			packed[i] = b3PackMergeResult( &ctx, stepCount );
			steps[i].sealedDepth = b3MaxFloat( sealedDepth, packed[i]->maxConcavity );
		}
		else
		{
			steps[i].sealedDepth = b3MaxFloat( sealedDepth, steps[i - 1].sealedDepth );
		}
	}

	b3DestroyMergeContext( &ctx );

	int best = b3ChooseMergeStep( steps, stepCount, def->hullWeight );
	b3ConvexMergeResult* result = packed[best];
	for ( int i = 0; i < stepCount; ++i )
	{
		if ( i != best )
		{
			b3DestroyConvexMergeResult( packed[i] );
		}
	}

	memcpy( result->steps, steps, (size_t)stepCount * sizeof( b3ConvexMergeStep ) );
	result->bestStep = best;
	return result;
}

void b3DestroyConvexMergeResult( b3ConvexMergeResult* result )
{
	if ( result != NULL )
	{
		b3Free( result, (size_t)result->byteCount );
	}
}
